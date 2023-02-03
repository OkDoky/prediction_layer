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


#ifndef OBSTACLES_BUFFER_H_
#define OBSTACLES_BUFFER_H_

#include <vector>
#include <list>
#include <string>
#include <ros/time.h>
#include <prediction_layer/dynamic_obstacles.h>
#include <prediction_layer/transforms.h>
#include <obstacle_detector/CircleObstacle.h>
#include <obstacle_detector/Obstacles.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// Thread support
#include <boost/thread.hpp>

namespace prediction_layer
{
  /**
   * @class ObstaclesBuffer
   * @brief Takes in obstacles from obstacle detector, transform them to the desired frame, and stores them
   */
  class ObstaclesBuffer
  {
    public:
      /**
       * @brief Construct a new Obstacles Buffer object
       * 
       * @param topic_name obstacle topic name
       * @param observation_keep_time Defines the persistance of observation in seconds, 0 means only keep the latest
       * @param expected_update_rate How often this buffer is expected to be updated, 0 means there is no limit
       * @param obstacle_range The range to which the objects to be trusted for inserting obstacles.
       * @param raytrace_range The range to which the objects to be trusted for raytracing to clear out space.
       * @param tf A reference to a TransformListener
       * @param costmap_global_frame The frame to transform Obstacles
       * @param source_frame The frame of the origin objects, can be left blank to be read from the messages.
       * @param tf_tolerance The amount of time to wait for a transform to be available when setting a new global frame
       */
      ObstaclesBuffer(std::string topic_name, double observation_keep_time, double expected_update_rate,
                      double obstacle_range, double raytrace_range, tf::TransformListener& tf, 
                      std::string costmap_global_frame, std::string source_frame, double tf_tolerance);
      
      /**
       * @brief Destroy the Obstacles Buffer object
       */
      ~ObstaclesBuffer();
      
      /**
       * @brief Set the Global Frame of an obstacles.
       * this will transform all the current cached obstacles to 
       * the new global frame.
       * @param new_global_frame the name of new global frame. 
       * @return true if operation succeded
       * @return false otherwise
       */
      bool setGlobalFrame(const std::string new_global_frame);

      /**
       * @brief Transforms a Obstacles to the global frame and buffers it
       * <b>Note: The burden is on the user to make sure the transform si available... ie they should use a MessageNotifier</b>
       * @param obstacles The Obstacles to be buffered
       */
      void buffferObstacles(const obstacle_detector::CircleObstacle& obstacles);
      
      /**
       * @brief Pushes copies of all current observations onto the end of the vector passed in.
       * @param obstacles The vector to be filled
       */
      void getObstacles(prediction_layer::DynamicObstacles& dynamic_obstacles);
      
      /**
       * @brief Check if the obstacle buffer is being update at its expected rate
       * @return true if it is being updated at the expected rate
       * @return false otherwise
       */
      bool isCurrent() const;
      
      /**
       * @brief Lock the obstacle buffer
       */
      inline void lock()
      {
        lock_.lock();
      }
      
      /**
       * @brief Unlock the obstacle buffer.
       */
      inline void unlock()
      {
        lock_.unlodk();
      }
      
      /**
       * @brief Reset last updated timestamp
       */
      void resetLastUpdated();
    private:
      /**
       * @brief Removes any stale obervations form the buffer list
       */
      void purgeStaleObstacles();

      tf::TransformListener& tf_;
      const ros::Duration observation_keep_time_;
      const ros::Duration expected_update_rate_;

      ros::Time last_updated_;
      std::string costmap_global_frame_;
      std::string source_frame_;
      std::list<obstacle_detector::CircleObstacle> circle_list_;
      std::string topic_name_;
      boost::recursive_mutex lock_;
      double obstacle_range_, raytrace_range_;
      double tf_tolerance_;
  }
}