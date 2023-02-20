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


#include <prediction_layer/obstacles_buffer.h>
#include <geometry_msgs/PointStamped.h>

using namespace std;
using namespace tf2;
using namespace obstacle_detector;

namespace prediction_layer
{
  ObstaclesBuffer::ObstaclesBuffer(string topic_name, double observation_keep_time, double expected_update_rate,
                  double obstacle_range, double raytrace_range, tf2_ros::Buffer& tf2_buffer, 
                  string global_frame, string source_frame, double tf_tolerance) :
    tf2_buffer_(tf2_buffer), 
    observation_keep_time_(observation_keep_time),
    expected_update_rate_(expected_update_rate),
    topic_name_(topic_name),
    source_frame_(source_frame),
    global_frame_(global_frame),
    tf_tolerance_(tf_tolerance),
    obstacle_range_(obstacle_range),
    raytrace_range_(raytrace_range),
    last_updated_(ros::Time::now())
  {
  }
  
  ObstaclesBuffer::~ObstaclesBuffer()
  {
  }
  
  bool ObstaclesBuffer::setGlobalFrame(const string new_global_frame)
  {
    ros::Time transform_time = ros::Time::now();
    string tf_error;

    // try check connection connection frames
    if (!tf2_buffer_.canTransform(new_global_frame, global_frame_, transform_time, 
            ros::Duration(tf_tolerance_), &tf_error))
    {
      ROS_ERROR("[ObstaclesBuffer] Transform beteen %s and %s with tolerance %.2f failed: %s.", new_global_frame.c_str(), 
                  global_frame_.c_str(), tf_tolerance_, tf_error.c_str());
      return false;
    }

    list<DynamicObstacle>::iterator obs_it;
    for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
    {
      try
      {
        DynamicObstacle& dyn_obj= *obs_it;

        geometry_msgs::PointStamped origin;
        origin.header.frame_id = global_frame_;
        origin.header.stamp = transform_time;
        origin.point = dyn_obj.origin_;

        // we need to transform the origin of the dynamic obstacles to the new global frame
        tf2_buffer_.transform(origin, origin, new_global_frame);
        dyn_obj.origin_ = origin.point;

        // we also need to transform the list<obstacle_dectector::CircleObstacle> to the new global frame
        tf2_buffer_.transform(dyn_obj.obs_, dyn_obj.obs_, new_global_frame);
      }
      catch (TransformException& ex)
      {
        ROS_ERROR("[ObstacleBuffer] TF Error attempting to transform an dynamic obstacles from %s to %s: %s", global_frame_.c_str(),
                  new_global_frame.c_str(), ex.what());
        return false;
      }
    }
    // now we need to update our global frame member
    global_frame_ = new_global_frame;
    return true;
  }

  /* void ObstaclesBuffer::bufferObstacles(const Obstacles& obs)
  {
    geometry_msgs::PointStamped global_origin, local_origin;
    observation_list_.push_front(DynamicObstacle());
    string origin_frame = source_frame_ == "" ? obs.header.frame_id : source_frame_;
    // transform origin data
    //check canTransform
    try
    {
      string tf_error;
      ROS_WARN("[bufferObstacles] check can transform with global frame and local_frame g : %s, l : %s",global_frame_.c_str(), origin_frame.c_str());
      if (!tf2_buffer_.canTransform(global_frame_, origin_frame, obs.header.stamp, ros::Duration(0.1), &tf_error))
      {  
        ROS_WARN("[bufferObstacles] Transform between %s and %s with tolerence %.2f failed: %s.",
                  global_frame_.c_str(), origin_frame.c_str(), tf_tolerance_, tf_error.c_str());
        return;
      }
    } 
    catch(TransformException& ex)
    {
      ROS_WARN("[bufferObstacles] cannot Transform %s to %s, %s",origin_frame.c_str(), global_frame_.c_str(), ex.what());
    }
    catch(exception& e)
    {
      ROS_WARN("[bufferObstacles] what's the problem, %s", e.what());
    }

    try
    {
      local_origin.header.stamp = obs.header.stamp;
      local_origin.header.frame_id = origin_frame;
      local_origin.point.x = 0;
      local_origin.point.y = 0;
      local_origin.point.z = 0;
      tf2_buffer_.transform(local_origin, global_origin, global_frame_);
      ROS_WARN("[bufferObstacles] success to transform.");
      tf2::convert(global_origin.point, observation_list_.front().origin_);
      ROS_WARN("[bufferObstacles] success to convert.");
      observation_list_.front().raytrace_range_ = raytrace_range_;
      observation_list_.front().obstacle_range_ = obstacle_range_;

      Obstacles  global_frame_obs;
      ROS_WARN("[bufferObstacles] ready to transform obs..");

      geometry_msgs::TransformStamped transform;
      transform = tf2_buffer_.lookupTransform(global_frame_, origin_frame, obs.header.stamp, ros::Duration(0.1));

      // tf2_buffer_.doTransform()
      // tf2_buffer_.transform(obs, global_frame_obs, global_frame_);
      ROS_WARN("[bufferObstacles] success to transform obs..");
      global_frame_obs.header.stamp = obs.header.stamp;

    }
    catch (TransformException& ex)
    {
      observation_list_.pop_front();
      ROS_ERROR("[PredictionLayer] TF Exception that should never happen for sensor frame: %s, obs frame: %s, %s", 
                source_frame_.c_str(), obs.header.frame_id.c_str(), ex.what());
      return;
    }
    last_updated_ = ros::Time::now();
    purgeStaleObstacles();
  } */

  void ObstaclesBuffer::bufferObstacles(const Obstacles& obs)
  {
    // init update target
    geometry_msgs::TransformStamped transform;
    string origin_frame = source_frame_ == "" ? obs.header.frame_id : source_frame_;
    geometry_msgs::Point local_origin, global_origin;
    Obstacles transformed_obs = obs;
    
    // 2222
    observation_list_.push_front(DynamicObstacle());

    // get lookuptransform form tf_buffer
    try
    {
      ROS_WARN("[bufferObstacles] try to lookuptransform");
      transform = tf2_buffer_.lookupTransform(global_frame_, origin_frame, obs.header.stamp, ros::Duration(tf_tolerance_));
    } 
    catch(TransformException& ex)
    {
      ROS_WARN("[bufferObstacles] cannot Transform %s to %s, %s",origin_frame.c_str(), global_frame_.c_str(), ex.what());
      
      // 2222
      observation_list_.pop_front();
      return;
    }
    ROS_WARN("[bufferObstacles] success to lookuptransform && start transform");

    try
    {
      // DynamicObstacles.origin_

      // 2222
      doTransform(local_origin, observation_list_.front().origin_, transform);

      // re
      // doTransform(local_origin, global_origin, transform);
      ROS_WARN("[bufferObstacles] success to transform origins");
      // DynamicObstacles.obs_
      // re
      // for (auto& circle : transformed_obs.circles)
      
      // 2222
      for (auto& circle : observation_list_.front().obs_)
      {
        geometry_msgs::Point transformed_center;
        geometry_msgs::Vector3 transformed_velocity;
        doTransform(circle.center, transformed_center, transform);
        doTransform(circle.velocity, transformed_velocity, transform);
        circle.center = transformed_center;
        circle.velocity = transformed_velocity;
      }
      // re
      // transformed_obs.header = obs.header;
      

      ROS_WARN("[bufferObstacles] success to transform circles");
    }
    catch (TransformException& ex)
    {
      // observation_list_.pop_front();
      ROS_ERROR("[PredictionLayer] TF Exception that should never happen for sensor frame: %s, obs frame: %s, %s", 
                source_frame_.c_str(), obs.header.frame_id.c_str(), ex.what());
      
      // 2222
      observation_list_.pop_front();
      return;
    }
    // observation_list_.push_front(DynamicObstacle(transformed_obs));
    last_updated_ = ros::Time::now();
    purgeStaleObstacles();
  }

  void ObstaclesBuffer::getObstacles(vector<DynamicObstacle>& dynamic_obstacles)
  {
    // first... let's make sure that we don't have any stale Obstacles
    purgeStaleObstacles();

    // now we'll just copy the Obstacles for the caller
    list<DynamicObstacle>::iterator obs_it;
    for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
    {
      dynamic_obstacles.push_back(*obs_it);
    }
  }
  
  bool ObstaclesBuffer::isCurrent() const
  {
    if (expected_update_rate_ == ros::Duration(0.0))
      return true;
    double time_diff = (ros::Time::now() - last_updated_).toSec();
    bool current = time_diff <= expected_update_rate_.toSec();
    if (!current)
    {
      ROS_WARN(
        "The %s DynamicObstacle buffer has not been updated for %.2f seconds, and it should be updated every %.2f seconds.",
        topic_name_.c_str(), time_diff, expected_update_rate_.toSec());
    }
    return current;

  }

  void ObstaclesBuffer::resetLastUpdated()
  {
    last_updated_ = ros::Time::now();
  }

  void ObstaclesBuffer::purgeStaleObstacles()
    {
      if (!observation_list_.empty())
      {
        list<DynamicObstacle>::iterator obs_it = observation_list_.begin();

        // if we're keeping dynamic obstacles for no time... 
        // then we'll only keep one set of obstacles
        if (observation_keep_time_ == ros::Duration(0.0))
        {
          observation_list_.erase(++obs_it, observation_list_.end());
          return;
        }

        // otherwise... we'll have to loop through the 
        // dynamic obstacles to see which ones are stale
        for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
        {
          DynamicObstacle& obs = *obs_it;
          // check if the obstacle is out of date... and if it is, 
          // remove it and those that follow from the list
          ros::Duration time_diff = last_updated_ - obs.updated_time_;
          if (time_diff > observation_keep_time_)
          {
            observation_list_.erase(obs_it, observation_list_.end());
            return;
          }
        }
      }
    }
  
} // namespace prediction_layer