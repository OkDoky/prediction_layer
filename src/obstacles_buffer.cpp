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

#include <prediction_layer/obstacles_buffer.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

using namespace std;
using namespace tf;

namespace prediction_layer
{
  class ObstaclesBuffer
  {
    ObstaclesBuffer(std::string topic_name, double observation_keep_time, double expected_update_rate,
                    double obstacle_range, double raytrace_range, tf::TransformListener& tf, 
                    std::string costmap_global_frame, std::string source_frame, double tf_tolerance) :
      tf_(tf), 
      observation_keep_time_(observation_keep_time),
      expected_update_rate_(expected_update_rate),
      topic_name_(topic_name),
      source_frame_(source_frame),
      costmap_global_frame_(costmap_global_frame),
      tf_tolerance_(tf_tolerance),
      obstacle_range_(obstacle_range),
      raytrace_range_(raytrace_range),
      last_updated_(ros::Time::now())
    {
    }

    
    ~ObstaclesBuffer()
    {
    }
    
    bool setGlobalFrame(const std::string new_global_frame)
    {
      ros::Time transform_time = ros::Time::now();
      string tf_error;

      // try check connection connection frames
      if (!tf_.waitForTransform(new_global_frame, transform_time, 
              ros::Duration(tf_tolerance_), ro::Duration(0.01), &tf_error))
      {
        ROS_ERROR("[ObstaclesBuffer] Transform beteen %s and %s with tolerance %.2f failed: %s.", new_global_frame.c_str(), 
                    costmap_global_frame_.c_str(), tf_tolerance_, tf_error.c_str());
        return false;
      }

      list<DynamicObstacles>::iterator dyn_obj_it;
      for (dyn_obj_it = circle_list_.begin(); dyn_obj_it != circle_list_.end(); ++dyn_obj_it)
      {
        try
        {
          DynamicObstacles& dyn_obj= *dyn_obj_it;

          geometry_msgs::PointStamped pose;
          pose.header.frame_id = costmap_global_frame_;
          pose.header.stamp = transform_time;
          pose.point = dyn_obj.pose_;

          // we need to transform the pose of the dynamic obstacles to the new global frame
          tf_.transformPoint(new_global_frame, pose, pose);
          dyn_obj.pose_ = pose.point;

          // we also need to transform the list<obstacle_dectector::CircleObstacle> to the new global frame
          // @todo transform list of circles(point, vector3)
          // *** transformCircles(new_global_frame, *dyn_obj.circles_, *dyn_obj.circles_, tf_);
        }
        catch (TransformException& ex)
        {
          ROS_ERROR("[ObstacleBuffer] TF Error attempting to transform an dynamic obstacles from %s to %s: %s", costmap_global_frame_.c_str(),
                    new_global_frame.c_str(), ex.what());
          return false;
        }
      }
      // now we need to update our global frame member
      costmap_global_frame_ = new_global_frame;
      return true;
    }


    void buffferObstacles(const obstacle_detector::Obstacles& obstacles)
    {
      Stamped < tf::Vector3 > global_pose;

      // create a new observation on the list to be populated
      circle_list_.push_front(DynamicObstacles());

      // check whether the pose frame has been set explicitly or whether we should get it from the circles
      string origin_frame = source_frame_ == "" ? obstacles.header.frame_id : source_frame_;

      try
      {
        // given these dynamic obstacles, we'll  need to store the origin obstacles
        Stamped < tf::Vector3 > local_origin(tf::Vector3(0,0,0),
                                             )
        tf_.waitForTransform(costmap_global_frame_, local_origin.frame_id_, local_origin.stamp_, ros::Duration(0.5));
        tf_.transformPoint(costmap_global_frame_, local_origin, global_pose);
        circle_list_.front().pose_.x = global_pose.getX();
        circle_list_.front().pose_.y = global_pose.getY();
        circle_list_.front().pose_.z = global_pose.getZ();

        // make sure to pass on the raytrace/obstacle range of the obstacles buffer to the obstacles
        circle_list_.front().raytrace_range_ = raytrace_range_;
        circle_list_.front().obstacle_range_ = obstacle_range_;

        obstacle_detector::Obstacles global_obstacles;

        // transform the obstacles
        // *** transformDynamicObstacles(costmap_global_frame_, obstacles, global_obstacles, tf_);
        global_obstacles.header.stamp = obstacles.header.stamp;
      }
      catch (TransformException& ex)
      {
        circle_list_.pop_front();
        ROS_ERROR("[ObstacleBuffer] TF Exception that should never happen for source frame : %s, obstacles frame : %s, %s", source_frame_.c_str(),
                    obstacles.header.frame_id.c_str(), ex.what());
        return;
      }

      // if the update was successful, we want to update the last updated time
      last_updated_ = ros::Time::now();

      // we'll also remove any stale observations from the list
      purgeStaleObstacles();
    }
    
    void getObstacles(prediction_layer::DynamicObstacles& dynamic_obstacles)
    {
      // first... let's make sure that we don't have any stale Obstacles
      purgeStaleObstacles();

      // now we'll just copy the Obstacles for the caller
      list<DynamicObstacles>::iterator dyn_obs_it;
      for (dyn_obs_it = circle_list_.begin(); dyn_obs_it != circle_list_.end(); ++dyn_obs_it)
      {
        dynamic_obstacles.push_back(*dyn_obs_it);
      }
    }
    
    bool isCurrent() const
    {
      if (expected_update_rate_ == ro::Duration(0.0))
        return true;
      double time_diff = (ros::Time::now() - last_updated_).toSec();
      bool current = time_diff <= expected_update_rate_.toSec();
      if (!current)
      {
        ROS_WARN(
          "The %s DynamicObstacles buffer has not been updated for %.2f seconds, and it should be updated every %.2f seconds.",
          topic_name_.c_str(), time_diff, expected_update_rate_.toSec());
      }
      return current;

    }

    void resetLastUpdated()
    {
      last_updated_ = ros::Time::now();
    }

    void purgeStaleObstacles()
    {
      if (!circle_list_.empty())
      {
        list<DynamicObstacles>::iterator dyn_obj_it = circle_list_.begin();

        // if we're keeping dynamic obstacles for no time... 
        // then we'll only keep one set of obstacles
        if (observation_keep_time_ == ros::Duration(0.0))
        {
          circle_list_.erase(++dyn_obj_it, circle_list_.end());
          return;
        }

        // otherwise... we'll have to loop through the 
        // dynamic obstacles to see which ones are stale
        for (dyn_obj_it = circle_list_.begin(); dyn_obj_it != circle_list_.end(); ++dyn_obj_it)
        {
          DynamicObstacles& obs = *dyn_obj_it;
          // check if the obstacle is out of date... and if it is, 
          // remove it and those that follow from the list
          ros::Duration time_diff = last_updated_ - obs.updated_time_;
          if (time_diff > observation_keep_time_)
          {
            circle_list_.erase(dyn_obj_it, circle_list_.end());
            return;
          }
        }
      }
    }

  }  // namespace prediction_layer
}