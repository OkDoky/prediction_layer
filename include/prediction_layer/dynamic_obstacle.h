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
#include <obstacle_detector/CircleObstacle.h>

using namespace obstacle_detector;

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
      updated_time_(ros::Time::now())
    {
    }

    virtual ~DynamicObstacle()
    {
    }

    // /**
    //  * @brief  Creates an observation from an origin point and a obs
    //  * @param origin The origin point of the observation
    //  * @param obs The point obs of the observation
    //  */
    // DynamicObstacle(geometry_msgs::Point& origin,
    //                 std::vector<CircleObstacle> obs) :
    //     origin_(origin),
    //     obs_(obs),
    //     updated_time_(ros::Time::now())
    // {
    // }

    // /**
    //  * @brief  Copy constructor
    //  * @param obs The observation to copy
    //  */
    // DynamicObstacle(const DynamicObstacle& obs) :
    //     origin_(obs.origin_), 
    //     obs_(obs.obs_),
    //     seq_(obs.seq_),
    //     updated_time_(ros::Time::now())
    // {
    // }

    // /**
    //  * @brief  Creates an observation from a point obs
    //  * @param obs The point obs of the observation
    //  * @param obstacle_range The range out to which an observation should be able to insert obstacles
    //  */
    // DynamicObstacle(std::vector<CircleObstacle> obs) :
    //     origin_(),
    //     obs_(obs),
    //     updated_time_(ros::Time::now())
    // {
    // }

    geometry_msgs::Point origin_;
    std::vector<CircleObstacle> obs_;
    ros::Time updated_time_;
    uint32_t seq_;
  };  // class DynamicObstacles

}  // namespace prediction_layer
#endif  // PREDICTION_LAYER_OBSERVATION_H_
