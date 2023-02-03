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

#include <mutex>
#include <vector>

#include <ros/ros.h>

#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/footprint.h>
#include <costmap_2d/costmap_2d.h>

#include <std_srvs/Trigger.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>

#include <obstacle_detector/Obstacles.h>
#include <geometry_msgs/Pose2D.h>

#include <prediction_layer/circle_to_circumscribe_polygon.h>
#include <prediction_layer/PredictionLayerConfig.h>


inline int getSign(double x) 
{
  return x > 0 ? 1 : -1;
}

double pointToPoint(double x1, double y1, double x2, double y2)
{
  double dx = x1-x2;
  double dy = y1-y2;
  double dist;

  dist = pow(dx,2) + pow(dy,2);
  dist = sqrt(dist);
  
  return dist;
}

double localPointToPoint(double x1, double y1)
{
  double dist;
  dist = pow(x1,2) + pow(y1,2);
  return sqrt(dist);
}

namespace prediction_layer
{
  struct PointInt
  {
    int x;
    int y;
  };

  using Polygon = std::vector<geometry_msgs::Point>;
  
  class PredictionLayer : public costmap_2d::CostmapLayer
  {
    public:
      /**
       * @brief Construct a new Prediction Layer object
       * 
       */
      PredictionLayer();

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
      virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
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
      virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

      bool rolling_window_;
      double _robot_x, _robot_y, _robot_yaw;

      std::vector<obstacle_detector::CircleObstacle> circle_obstacles;
      
      double obstacle_range_;
      ros::Time data_update_time;
      ros::Duration data_update_duration;
      double callback_data_patience_;
      bool initialize_;

    private:
      virtual void reset();
      void reconfigureCB(PredictionLayerConfig &config, 
                         uint32_t level);
      bool resetLayerCallback(std_srvs::Trigger::Request& req, 
                              std_srvs::Trigger::Response& res);
      void ObstaclesCallback(const obstacle_detector::Obstacles msg);
      void rasterizePolygon(const std::vector<PointInt> &polygon,
                            std::vector<PointInt> &polygon_cells);
      void setPolygonCost(costmap_2d::Costmap2D &master_grid, 
                          const geometry_msgs::Polygon &polygon,
                          int min_i, int min_j, int max_i, int max_j);
      void polygonOutlineCells(const std::vector<PointInt> &polygon,
                               std::vector<PointInt> &polygon_cells);
      void raytrace(int x0, int y0, int x1, int y1, 
                    std::vector<PointInt> &cells);

      obstacle_detector::Obstacles _obstacles;
      obstacle_detector::Obstacles _dist_range_obstacles, _path_range_obstacles, _emergency_range_obstcles;

      dynamic_reconfigure::Server<PredictionLayerConfig>* _dsrv; 
      std::mutex _data_mutex;

      std::string source_frame;
      double _costmap_resolution;
      double _min_x, _min_y, _max_x, _max_y;
      boost::recursive_mutex lock_;

      ros::Subscriber obstacles_sub_;
      ros::ServiceServer reset_layer_;
      ros::Publisher polygon_pub_;

      // reconfigure config values
      bool enabled_, footprint_clearing_enabled_;
      double max_obstacle_range_;

      // footprint
      std::vector<geometry_msgs::Point> transformed_footprint_;      
  };
}
#endif