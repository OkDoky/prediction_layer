/*
 * Copyright (c) 2008, 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Conor McGann
 * Editor: OkDoky
 */

#ifndef PREDICTION_LAYER_DYNAMIC_OBSTACLES_H_
#define PREDICTION_LAYER_DYNAMIC_OBSTACLES_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <obstacle_detector/CircleObstacle.h>

using namespace obstacle_detector;
using namespace std;

namespace prediction_layer
{
  class DynamicObstacle
  {
  public:
    /**
     * @brief  Creates an empty dynamic obstacles
     */
    DynamicObstacle() :
      obs_(), 
      transformed_(false),
      updated_time_(ros::Time::now())
    {
    }

    virtual ~DynamicObstacle()
    {
    }

    geometry_msgs::Point origin_;
    vector<CircleObstacle> obs_;
    ros::Time updated_time_;
    uint32_t seq_;
    bool transformed_;
    double pub_to_buf_;

    // add velocity boundary
    // vector<geometry_msgs::Polygon> vel_boundary_; 
    vector<vector<geometry_msgs::Point>> vel_boundary_;
    // New member variables for bounding box
    geometry_msgs::Point min_bound_;
    geometry_msgs::Point max_bound_;
    
    // Function to calculate the bounding box
    void calculateBoundingBox()
    {
      double min_x = numeric_limits<double>::max();
      double min_y = numeric_limits<double>::max();
      double max_x = numeric_limits<double>::min();
      double max_y = numeric_limits<double>::min();
      
      for(const auto& obs : obs_)
      {
        double radius = obs.radius;
        double x = obs.center.x;
        double y = obs.center.y;
        
        if(x - radius < min_x) min_x = x - radius;
        if(x + radius > max_x) max_x = x + radius;
        if(y - radius < min_y) min_y = y - radius;
        if(y + radius > max_y) max_y = y + radius;
      }
      
      min_bound_.x = min_x;
      min_bound_.y = min_y;
      max_bound_.x = max_x;
      max_bound_.y = max_y;
    }
  };  // class DynamicObstacles

}  // namespace prediction_layer
#endif  // PREDICTION_LAYER_OBSERVATION_H_
