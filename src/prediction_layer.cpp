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

#include <prediction_layer/prediction_layer.h>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(prediction_layer::PredictionLayer, costmap_2d::Layer)

using costmap2d::NO_INFORMATION;
using costmap2d::LETHAL_OBSTACLE;
using costmap2d::FREE_SPACE;

using costmap2d::ObservationBuffer;
using costmap2d::Observation;

namespace prediction_layer
{
  void PredictionLayer::reset()
  {
    initialize_ = false;
  }

  void PredictionLayer::onInitialize()
  {
    ros::NodeHandle nh_param("~/" + name_);
    ros::NodeHandle nh;

    current_ = true;
    costmap_2d::Costmap2D *costmap = layered_costmap_->getCostmap();
    costmap_resolution_ = costmap->getResolution();
    rolling_window_ = layered_costmap_->isRolling();

    global_frame_ = layered_costmap_->getGlobalFrameID();

    double transform_tolerance;
    nh_param.param("transform_tolerance", transform_tolerance, 0.2);

    std::string topics_string;
    nh_param.param("object_source", topics_string, std::string(""));
    ROS_INFO("  Subscribed to Topics: %s", topics_string.c_str());

    obstacles_sub_ = nh.subscribe<obstacle_detector::Obstacles>(topics_stirng.c_str(), 0, &PredictionLayer::obstaclesCallback, this);

    initialize_ = true;
    
    dsrv_ = new dynamic_reconfigure::Server<PredictionLayerConfig>(nh_param);
    dynamic_reconfigre::Server<PredictionLayerConfig>::CallbackType cb = boost::bind(
      &PredictionLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    // added
    reset_layer_ = nh.addvertiseService("reset_layer", &PredictionLayer::resetLayerCallback, this);
  }

  PredictionLayer::~PredictionLayer()
  {
    if (dsrv_)
      delete dsrv_;
  }

  void PredictionLayer::reconfigureCB(costmap_2d::PredictionLayerConfig & config, uint32_t level)
  {
    enabled_ = config.enabled;
    footprint_clearing_enabled_ = config.footprint_clearing_enabled;
  }

  void PredictionLayer::ObstaclesCallback(const obstacle_detector::Obstacles& msg)
  {
    pass;
  }


  bool PredictionLayer::resetLayerCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
    ROS_WARN("[ObstacleLayer] Clearing by Service Call");
    reset();
    res.success = true;
    res.message = "[ObstacleLayer] Clearing by Service Call";
    return true;
  }

}
