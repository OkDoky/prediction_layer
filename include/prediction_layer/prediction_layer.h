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

#include <ros/ros.h>

#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/footprint.h>

#include <std_srvs/Trigger.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>

#include <obstacle_detector/Obstacles.h>


namespace prediction_layer
{
  class PredictionLayer : public costmap_2d::Layer
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
      virtual void updateCosts(costamp_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

      bool rolling_window_;
      double _robot_x, _robot_y, _robot_yaw;

      nav_msgs::Odometry odom;
      std::vector<obstacle_detector::CircleObstacle> circle_obstacles;
      
      double obstacle_range_;
      ros::Time data_update_time;
      ros::Duration data_update_duration;
      bool initialize_;

    private:
      virtual void reset();
      void reconfigureCB(PredictionLayerConfig &config, uint32_t level);
      void resetLayerCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
      void ObstaclesCallback(const obstacle_detector::Obstacles& msg);

      dynamic_reconfigure::Server<PredictionLayerConfig>* _dsrv; 
      std::mutex _data_mutex;
      double _costmap_resolution;
      double _min_x, _min_y, _max_x, _max_y;
      boost::recursive_mutex lock_;

      ros::Subscriber obstacles_sub_;
      ros::ServiceServer reset_layer_;
  }
}