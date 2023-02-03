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

#ifndef obstacles_ROS_TRANSFORMS_H_
#define obstacles_ROS_TRANSFORMS_H_

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <prediction_layer/dynamic_obstacles.h>

namespace prediction_layer
{
  void transformPolygonWithNormals (const geometry_msgs::Polygon &polygon_in,
                                    const std::string source_frame,
                                    geometry_msgs::Polygon &polygon_out,
                                    const std::string target_frame,
                                    const tf::Transform &transform);

  void transformPolygon (const geometry_msgs::Polygon &polygon_in,
                         const std::string source_frame,
                         geometry_msgs::Polygon &polygon_out,
                         const std::string target_frame,
                         const tf::Transform &transform);
                         
  void transformAsMatrix (const tf::Transform& bt, Eigen::Matrix4f &out_mat);

  void transformAsMatrix (const geometry_msgs::Tranform& bt, Eigen::Matrix4f &out_mat);
  void transformCircles (const std::string new_global_frame, 
                         const prediction_layer::DynamicObstacles& obs_in, 
                         prediction_layer::DynamicObstacles& obs_out, 
                         const tf::TransformListener& tf_);
}