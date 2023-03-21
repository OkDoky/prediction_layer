/*************************************************************************************************
 * The MIT License
 * 
 * Copyright (c) 2023, Hanyang.Univ CAI-LAB
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * 
 * Author: OkDoky
 **********************************************************************************************/


#ifndef PREDICTION_LAYER_H_
#define PRECINTION_LAYER_H_

#include <ros/ros.h>

#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <prediction_layer/obstacles_buffer.h>
#include <prediction_layer/PolygonBoundary.h>

#include <std_srvs/Trigger.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>

#include <obstacle_detector/Obstacles.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>

#include <prediction_layer/PredictionLayerConfig.h>
#include <costmap_2d/footprint.h>

// for boost::Geometry using point in polygon algorithm
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/within.hpp>

using namespace std;
using namespace obstacle_detector;
using namespace costmap_2d;

// for boost::Gometry
namespace bg = boost::geometry;
typedef bg::model::d2::point_xy<int> bpoint;
typedef bg::model::polygon<bpoint> bpolygon;
BOOST_GEOMETRY_REGISTER_POINT_2D(vector<int>, int, bg::cs::cartesian, operator[](0), operator[](1))

namespace prediction_layer
{
  class PredictionLayer : public CostmapLayer
  {
    public:
      /**
       * @brief Construct a new Prediction Layer object
       * 
       */
      PredictionLayer()
      {
        costmap_ = NULL; // this is the unsigned char* member of parent class Costmap2D.
      }

      /**
       * @brief Destroy the Prediction Layer object
       * 
       */
      virtual ~PredictionLayer();
      
      /**
       * @brief function which get called at initializing the costmap
       * define the reconfigre callback, get resolution from costmapLayer
       * and initialize subscriber for Obstacles.
       */
      virtual void onInitialize();

      /**
       * @brief this called by LayeredCostmap to poll this plugin
       * as to how much of the costmap it needs to update.
       * Each layer can increase the size of this bounds.
       * 
       * @param robot_x odom pose x
       * @param robot_y odom pose y
       * @param robot_yaw odom pose theta 
       * @param min_x 
       * @param min_y 
       * @param max_x 
       * @param max_y 
       */
      virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, 
                                double* min_x, double* min_y, double* max_x, double* max_y);
      /**
       * @brief function which get called at every cost updating produce
       * of the overlayed costmap. The before readed costs will get
       * filled
       * 
       * @param master_grid 
       * @param min_i 
       * @param min_j 
       * @param max_i 
       * @param max_j 
       */
      virtual void updateCosts(Costmap2D& master_grid, 
                               int min_i, int min_j, int max_i, int max_j);

      virtual void reset();
      virtual void activate();
      virtual void deactivate();

      void obstacleCallback(const ObstaclesConstPtr& msg,
                            const boost::shared_ptr<ObstaclesBuffer>& buffer);

    protected:
      bool resetLayerCallback(std_srvs::Trigger::Request& req, 
                              std_srvs::Trigger::Response& res);
      virtual void setupDynamicReconfigure(ros::NodeHandle& nh);
      bool getObservations(vector<DynamicObstacle>& observations) const;
      void updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                                double* min_x, double* min_y, double* max_x, double* max_y);
      void updateFootprint(double robot_x, double robot_y, double robot_yaw,
                           double* min_x, double* min_y, double* max_x, double* max_y);
      
      inline void radiusBounds(double radius, double* min_x, double* min_y,
                               double* max_x, double* max_y)
      {
        *min_x -= radius;
        *min_y -= radius;
        *max_x += radius;
        *max_y += radius;
      }

      inline bool isPointInsidePolygon(const vector<vector<int>>& polygon,
                                       const vector<int>& point)
      {
        try
        {
          bpolygon boost_polygon;
          for (const auto& vertex : polygon)
          {
            boost_polygon.outer().emplace_back(bpoint(vertex[0], vertex[1]));
          }
          boost_polygon.outer().emplace_back(bpoint(polygon[0][0],polygon[0][1]));
          bpoint boost_point(point[0], point[1]);
          return bg::within(boost_point, boost_polygon);
        }
        catch (exception& ex){
          ROS_WARN("[PredictionLayer] fail to get the point is inside polygon, %s", ex.what());
          return false;
        }
      }

      inline bool isPointInsidePolygon(const vector<geometry_msgs::Point>& polygon,
                                       int i, int j)
      {
        vector<vector<int>> c_fp;
        for (auto& p : polygon)
        {
          int mx, my;
          mx = (int)((p.x - origin_x_) / resolution_);
          my = (int)((p.y - origin_y_) / resolution_);
          vector<int> fp = {mx, my};
          c_fp.push_back(fp);
        }
        return isPointInsidePolygon(c_fp, {i,j});
      }

      inline bool isPointInsidePolygons(const vector<geometry_msgs::Polygon>& polygons,
                                        int i, int j)
      {
        bool isOccupied = false;
        for (auto& polys : polygons)
        {
          vector<vector<int>> c_vp;
          for (auto& p : polys.points)
          {
            int mx, my;
            mx = (int)((p.x - origin_x_) / resolution_);
            my = (int)((p.y - origin_y_) / resolution_);
            vector<int> vp = {mx, my};
            c_vp.push_back(vp);
          }
          isOccupied = isPointInsidePolygon(c_vp, {i,j});
          if (isOccupied)
            return isOccupied; 
        }
        return isOccupied;
      }

      inline bool isPointInsideCircles(const vector<CircleObstacle>& obs,
                                       int i, int j)
      {
        for (auto& obstacle : obs)
        {
          int mx, my;
          mx = (int)((obstacle.center.x - origin_x_) / resolution_);
          my = (int)((obstacle.center.y - origin_y_) / resolution_);
          if (mx < 0 || my < 0 || mx > size_x_ || my > size_y_)
            continue;
          double radius = obstacle.radius / resolution_;
          double sq_radius = radius*radius;
          double dx = (double)mx - (double)i;
          double dy = (double)my - (double)j;
          double sq_dist = dx*dx + dy*dy;
          if (sq_dist <= sq_radius)
            return true;
        }
        return false;
      }

      vector<geometry_msgs::Point> transformed_footprint_;
      bool footprint_clearing_enabled_;
      string global_frame_;
      
      vector<boost::shared_ptr<message_filters::SubscriberBase>> observation_subscribers_;
      vector<boost::shared_ptr<tf2_ros::MessageFilterBase>> observation_notifiers_;
      vector<boost::shared_ptr<ObstaclesBuffer>> observation_buffers_;

      // for use current observations
      vector<DynamicObstacle> observations_;

      bool rolling_window_;
      dynamic_reconfigure::Server<PredictionLayerConfig> *dsrv_;

      int combination_method_;
    
    private:
      void reconfigureCB(PredictionLayerConfig &config, uint32_t level);

      bool initialize_, enabled_;
      bool debug_mode_;
      
      ros::Time last_call_updateCosts_, last_call_updateBounds_;

      ros::Publisher pub_transformed_footprint_;
      ros::ServiceServer reset_layer_;

      // for debug boundary
      ros::Publisher pub_boundarys_;
      ros::Publisher pub_first_polygon_;

      // costmap values
      unsigned int size_x_, size_y_;
      double origin_x_, origin_y_, resolution_;
  };
}
#endif
