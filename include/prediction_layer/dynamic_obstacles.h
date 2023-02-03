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
#include <geometry_msgs/Vector3.h>
#include <obstacle_detector/Obstacles.h>
#include <obstacle_detector/CircleObstacle.h>

namespace prediction_layer
{
  class DynamicObstacles
  {
  public:
    /**
     * @brief  Creates an empty dynamic obstacles
     */
    DynamicObstacles() :
      circles_(new std::vector<obstacle_detector::CircleObstacle>()), obstacle_range_(0.0), raytrace_range_(0.0)
    {
    }

    virtual ~DynamicObstacles()
    {
      delete circles_;
    }

    /**
     * @brief  Creates an observation from an origin point and a circles
     * @param pose The origin point of the observation
     * @param velocity The obstacle velocity vector
     * @param circles The point circles of the observation
     * @param obstacle_range The range out to which an observation should be able to insert obstacles
     * @param raytrace_range The range out to which an observation should be able to clear via raytracing
     */
    DynamicObstacles(geometry_msgs::Point& pose, 
                    geometry_msgs::Vector3& vel,
                    std::vector<obstacle_detector::CircleObstacle> circles,
                    double obstacle_range, 
                    double raytrace_range) :
        pose_(pose), 
        velocity_(vel),
        circles_(new std::vector<obstacle_detector::CircleObstacle>(circles)),
        obstacle_range_(obstacle_range), 
        raytrace_range_(raytrace_range),
        updated_time_(ros::Time::now())
    {
    }

    /**
     * @brief  Copy constructor
     * @param obs The observation to copy
     */
    DynamicObstacles(const Observation& obs) :
        pose_(obs.pose_), 
        velocity_(obs.velocity_),
        circles_(new std::vector<obstacle_detector::CircleObstacle>(*(obs.circles_))),
        obstacle_range_(obs.obstacle_range_), 
        raytrace_range_(obs.raytrace_range_),
        updated_time_(ros::Time::now())
    {
    }

    /**
     * @brief  Creates an observation from a point circles
     * @param circles The point circles of the observation
     * @param obstacle_range The range out to which an observation should be able to insert obstacles
     */
    DynamicObstacles(std::vector<obstacle_detector::CircleObstacle> circles, 
                    double obstacle_range) :
        circles_(new std::vector<obstacle_detector::CircleObstacle>(circles)), 
        obstacle_range_(obstacle_range), 
        raytrace_range_(0.0),
        updated_time_(ros::Time::now())
    {
    }

    geometry_msgs::Point pose_;
    geometry_msgs::Vector3 velocity_;
    std::vector<obstacle_detector::CircleObstacle> circles_;
    double obstacle_range_, raytrace_range_;
    ros::Time updated_time_;
  };  // class DynamicObstacles

}  // namespace prediction_layer
#endif  // PREDICTION_LAYER_OBSERVATION_H_
