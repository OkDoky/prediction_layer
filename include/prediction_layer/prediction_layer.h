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

#include <std_srvs/Trigger.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>

#include <obstacle_detector/Obstacles.h>
#include <geometry_msgs/Point.h>

#include <prediction_layer/PredictionLayerConfig.h>
#include <costmap_2d/footprint.h>

using namespace std;
using namespace obstacle_detector;
using namespace costmap_2d;

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

      // for testing purposes
      void addStaticObservation(DynamicObstacle& obs, bool marking, bool clearing);
      void clearStaticObservations(bool marking, bool clearing);

    protected:
      bool resetLayerCallback(std_srvs::Trigger::Request& req, 
                              std_srvs::Trigger::Response& res);
      //====================
      virtual void setupDynamicReconfigure(ros::NodeHandle& nh);
      bool getMarkingObservations(vector<DynamicObstacle>& marking_observations) const;
      bool getClearingObservations(vector<DynamicObstacle>& clearing_observations) const;
      virtual void raytraceFreespace(const DynamicObstacle& clearing_observation,
                                     double* min_x, double* min_y, double* max_x, double* max_y);
      void updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                                double* min_x, double* min_y, double* max_x, double* max_y);
      void updateFootprint(double robot_x, double robot_y, double robot_yaw,
                           double* min_x, double* min_y, double* max_x, double* max_y);

      vector<geometry_msgs::Point> transformed_footprint_;
      bool footprint_clearing_enabled_;
      string global_frame_;
      
      vector<boost::shared_ptr<message_filters::SubscriberBase>> observation_subscribers_;
      vector<boost::shared_ptr<tf2_ros::MessageFilterBase>> observation_notifiers_;
      vector<boost::shared_ptr<ObstaclesBuffer>> observation_buffers_;
      vector<boost::shared_ptr<ObstaclesBuffer>> marking_buffers_;
      vector<boost::shared_ptr<ObstaclesBuffer>> clearing_buffers_;

      // Used only for testing purposes
      vector<DynamicObstacle> static_clearing_observations_, static_marking_observations_;

      bool rolling_window_;
      dynamic_reconfigure::Server<PredictionLayerConfig> *dsrv_;

      int combination_method_;
    
    private:
      void reconfigureCB(PredictionLayerConfig &config, uint32_t level);
      ros::ServiceServer reset_layer_;
      bool initialize_, enabled_;
  };
}
#endif