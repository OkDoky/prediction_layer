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

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

using namespace costmap_2d;
using namespace obstacle_detector;
using namespace std;

namespace prediction_layer
{
  /* PredictionLayer::PredictionLayer() : _dsrv(NULL)
  {
  } */
  
  PredictionLayer::~PredictionLayer()
  {
    if (dsrv_)
      delete dsrv_;
  }

  void PredictionLayer::onInitialize()
  {
    ros::NodeHandle nh("~/" + name_), g_nh;
    rolling_window_ = layered_costmap_->isRolling();

    bool track_unknown_space;
    nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());
    if (track_unknown_space)
      default_value_ = NO_INFORMATION;
    else
      default_value_ = FREE_SPACE;
    default_value_ = FREE_SPACE;
    PredictionLayer::matchSize();
    current_ = true;
    initialize_ = false;
    
    global_frame_ = layered_costmap_->getGlobalFrameID();
    double transform_tolerance;
    nh.param("transform_tolerance", transform_tolerance, 0.2);

    string topics_string;
    nh.param("object_source", topics_string, string(""));
    ROS_INFO("  Subscribed to Topics: %s", topics_string.c_str());

    double observation_keep_time, expected_update_rate;
    string topic, source_frame;
    debug_mode_ = false;

    nh.getParam("object_source", topic);
    nh.getParam("source_frame", source_frame);
    nh.getParam("observation_persistence", observation_keep_time);
    nh.getParam("expected_update_rate", expected_update_rate);
    nh.getParam("combination_method", combination_method_);
    nh.getParam("debug_mode", debug_mode_);
    ROS_WARN("[PredictionLayer] object_source : %s",topic.c_str());
    ROS_WARN("[PredictionLayer] source_frame : %s",source_frame.c_str());
    ROS_WARN("[PredictionLayer] observation_persistence : %.2f",observation_keep_time);
    ROS_WARN("[PredictionLayer] expected_update_rate : %.2f",expected_update_rate);
    ROS_WARN("[PredictionLayer] debug mode : %s", to_string(debug_mode_).c_str());
    
    ROS_WARN("[ObstacleLayer] Creating an observation buffer for source %s, topic %s, frame %s", 
      topic.c_str(), topic.c_str(), source_frame.c_str());
    // create an observation buffer
    try
    {
      observation_buffers_.push_back(
        boost::shared_ptr<ObstaclesBuffer
            > (new ObstaclesBuffer(topic, observation_keep_time, expected_update_rate,
                                   *tf_, global_frame_,source_frame, transform_tolerance)));
    }
    catch (exception ex)
    {
      ROS_WARN("[PredictionLayer] %s", ex.what());
    }
    ROS_WARN(
      "[PredictionLayer] Created an observation buffer for source %s, topic %s, global frame: %s,"
      "expected update rate: %.2f, observation persistence: %.2f",
      topic.c_str(), topic.c_str(), global_frame_.c_str(), expected_update_rate, observation_keep_time);

    // create a callback for the topic
    boost::shared_ptr < message_filters::Subscriber<Obstacles>
      > sub(new message_filters::Subscriber<Obstacles>(g_nh, topic, 50));
    boost::shared_ptr < tf2_ros::MessageFilter<Obstacles>
      > filter(new tf2_ros::MessageFilter<Obstacles>(*sub, *tf_, global_frame_, 50, g_nh));
    filter->registerCallback(
      boost::bind(&PredictionLayer::obstacleCallback, this, _1, observation_buffers_.back())
    );
    observation_subscribers_.push_back(sub);
    observation_notifiers_.push_back(filter);
    observation_notifiers_.back()->setTolerance(ros::Duration(0.05));

    if (source_frame != "")
    {
      vector < string > target_frames;
      target_frames.push_back(global_frame_);
      target_frames.push_back(source_frame);
      observation_notifiers_.back()->setTargetFrames(target_frames);
    }

    dsrv_ = NULL;
    setupDynamicReconfigure(nh);

    // debug
    last_call_updateCosts_ = ros::Time::now();
    last_call_updateBounds_ = ros::Time::now();

    // added
    reset_layer_ = nh.advertiseService("reset_layer", &PredictionLayer::resetLayerCallback, this);
    ROS_WARN("[PredictionLayer] succeded to init prediction layer");
    pub_transformed_footprint_ = nh.advertise<geometry_msgs::PolygonStamped>("transformed_footprint", 10);
    pub_boundarys_ = nh.advertise<prediction_layer::PolygonBoundary>("velocity_boundarys", 10);
    pub_first_polygon_ = nh.advertise<geometry_msgs::PolygonStamped>("first_boundarys", 10);
  }

  void PredictionLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
  {
    dsrv_ = new dynamic_reconfigure::Server<prediction_layer::PredictionLayerConfig>(nh);
    dynamic_reconfigure::Server<prediction_layer::PredictionLayerConfig>::CallbackType cb = boost::bind(
      &PredictionLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }

  void PredictionLayer::reconfigureCB(PredictionLayerConfig & config, uint32_t level)
  {
    enabled_ = config.enabled;
    footprint_clearing_enabled_ = config.footprint_clearing_enabled;
  }

  void PredictionLayer::obstacleCallback(const ObstaclesConstPtr& msg,
                                          const boost::shared_ptr<ObstaclesBuffer>& buffer)
  {
    ros::Time s_t = ros::Time::now();
    buffer->lock();
    buffer->bufferObstacles(*msg);
    buffer->unlock();
    initialize_ = true;
    ros::Time e_t = ros::Time::now();
    double c_t = (e_t - s_t).toSec();
    ROS_DEBUG_COND(debug_mode_,"[obstacleCallback] %.6f", c_t);
  }

  void PredictionLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                    double *min_x, double *min_y, double *max_x, double *max_y)
  {
    ros::Time s_t = ros::Time::now();
    // not initialized
    if (!initialize_)
      return;
    // disable prediction layer
    if (!enabled_)
      return;
    
    if (rolling_window_)
      updateOrigin(robot_x - getSizeInMetersX() / 2,
                   robot_y - getSizeInMetersY() /2);
    useExtraBounds(min_x, min_y, max_x, max_y);
    bool current = true;

    vector<DynamicObstacle> observations;
    current = current && getObservations(observations);
    observations_ = observations;
    current_ = current;

    // place the new obstacles into a priority queue... each with a priority of zero to begin with
    for (vector<DynamicObstacle>::const_iterator it = observations.begin(); it != observations.end(); ++it)
    {
      const DynamicObstacle& obs = *it;

      const vector<CircleObstacle>& obj= obs.obs_;
      double max_radius = -0.0;
      for (unsigned int i = 0; i < obj.size(); i++)
      {
        double px = obj[i].center.x;
        double py = obj[i].center.y;
        max_radius = max(max_radius, obj[i].radius);
        unsigned int mx, my;
        if (!worldToMap(px, py, mx, my))
        {
          ROS_WARN("[PredictionLayer] fail to world to map in updateBounds");
          continue;
        }
        unsigned int index = getIndex(mx, my);
        costmap_[index] = LETHAL_OBSTACLE;
        touch(px, py, min_x, min_y, max_x, max_y);
      }
      radiusBounds(max_radius, min_x, min_y, max_x, max_y);
    }
    updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
    ros::Time e_t = ros::Time::now();
    double c_t = (e_t - s_t).toSec();
    ROS_DEBUG_COND(debug_mode_,"[updateBounds] %.6f", c_t);
  }

  void PredictionLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw,
                                        double* min_x, double* min_y, double* max_x, double* max_y)
  {
    if (!footprint_clearing_enabled_) return;
    transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);
    
    geometry_msgs::PolygonStamped polygon;
    polygon.header.stamp = ros::Time::now();
    polygon.header.frame_id = global_frame_;
    
    for (unsigned int i = 0; i < transformed_footprint_.size(); i++)
    {
      touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
      geometry_msgs::Point32 point;
      point.x = transformed_footprint_[i].x;
      point.y = transformed_footprint_[i].y;
      polygon.polygon.points.push_back(point);
    }

    pub_transformed_footprint_.publish(polygon);
  }

  void PredictionLayer::updateCosts(Costmap2D &master_grid, 
                                    int min_i, int min_j, int max_i, int max_j)
  {
    ros::Time s_t = ros::Time::now();
    if (!initialize_)
      return;
    if (!enabled_)
      return;

    unsigned int size_x = master_grid.getSizeInCellsX();
    unsigned int size_y = master_grid.getSizeInCellsX();
    double origin_x = master_grid.getOriginX();
    double origin_y = master_grid.getOriginY();
    double resolution = master_grid.getResolution();

    // Fill out circle obstacles
    for (auto obstacle : observations_.back().obs_)
    {
      unsigned int mx, my;
      mx = (unsigned int)((obstacle.center.x - origin_x) / resolution);
      my = (unsigned int)((obstacle.center.y - origin_y) / resolution);
      if (obstacle.center.x < origin_x || obstacle.center.y < origin_y || mx > size_x || my > size_y)
      {
        ROS_WARN("[PredcitionLayer] center pose x : %.5f, y : %.5f, mx : %d, my : %d",obstacle.center.x, obstacle.center.y, mx, my);
        continue;
      }

      // Fill out obstacle circle
      double radius = obstacle.radius / resolution;
      double sq_radius = radius * radius;
      for (unsigned int i = 0; i < size_x; i++)
      {
        for (unsigned int j = 0; j < size_y; j++)
        {
          double dx = (double)mx - (double)i;
          double dy = (double)my - (double)j;
          double sq_distance = dx*dx + dy*dy;
          if (sq_distance <= sq_radius)
          {
            master_grid.setCost(i,j, LETHAL_OBSTACLE);          
          }
        }
      }
    }
    // PolygonBoundary boundary;
    // for (auto polys : observations_.back().vel_boundary_)
    // {
    //   boundary.boundarys.push_back(polys);
    // }
    // pub_boundarys_.publish(boundary);

    // // just for debug polygon shape
    // geometry_msgs::PolygonStamped display_polygon;
    // display_polygon.header.stamp = ros::Time::now();
    // display_polygon.header.frame_id = global_frame_;
    // display_polygon.polygon = observations_.back().vel_boundary_.back();
    // pub_first_polygon_.publish(display_polygon);

    try
    {
      // Fill out velocity polygons
      for (auto polys : observations_.back().vel_boundary_)
      {
        vector<vector<int>> c_vp;
        for (int k = 0; k < polys.points.size(); k++)
        {
          int mx, my;
          mx = (int)((polys.points[k].x - origin_x) / resolution);
          my = (int)((polys.points[k].y - origin_y) / resolution);
          vector<int> p = {mx, my};
          c_vp.push_back(p);
        }

        for (int i = 0; i < size_x; i++)
        {
          for (int j = 0; j < size_y; j++)
          {
            if (isPointInsidePolygon(c_vp, {i, j}))
              master_grid.setCost(i,j, INSCRIBED_INFLATED_OBSTACLE);
          }
        }
      }
    }
    catch (exception ex)
    {
      ROS_WARN("[PredictionLayer] %s", ex.what());
    }

    // Apply combination method
    switch (combination_method_)
    {
      case 0:  // Overwrite
        updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
        break;
      case 1:  // Maximum
        updateWithMax(master_grid, min_i, min_j, max_i, max_j);
        break;
      default:  // Nothing
        break;
    }

    if (footprint_clearing_enabled_)
    {
      // init footprint to cell index
      vector<vector<int>> c_fp;
      for (int k = 0; k < transformed_footprint_.size(); k++)
      {
        int mx, my;
        mx = (int)((transformed_footprint_[k].x - origin_x) / resolution);
        my = (int)((transformed_footprint_[k].y - origin_y) / resolution);
        vector<int> p = {mx, my};
        c_fp.push_back(p);
      }

      // find which cell is inside footprint
      for (int i = 0; i < size_x; i++)
      {
        for (int j = 0; j < size_y; j++)
        {
          if (isPointInsidePolygon(c_fp, {i,j}))
          {
            master_grid.setCost(i,j, FREE_SPACE); // INSCRIBED_INFLATED_OBSTACLE ,FREE_SPACE
          }
        }
      }
    }

    ROS_DEBUG_COND(debug_mode_, "[updateBounds] buffer to updateCost : %.6f", (ros::Time::now() - observations_.back().updated_time_).toSec());
    ROS_DEBUG_COND(debug_mode_, "[updateBounds] publish to buffer : %.6f", observations_.back().pub_to_buf_);
    ros::Time e_t = ros::Time::now();
    double c_t = (e_t - s_t).toSec();
    ROS_DEBUG_COND(debug_mode_,"[updateCosts] %.6f", c_t);
  }

  bool PredictionLayer::getObservations(vector<DynamicObstacle>& observations) const
  {
    bool current = true;
    for (unsigned int i = 0; i < observation_buffers_.size(); ++i)
    {
      observation_buffers_[i]->lock();
      observation_buffers_[i]->getObstacles(observations);
      current = observation_buffers_[i]->isCurrent() && current;
      observation_buffers_[i]->unlock();
    }
    observations.insert(observations.end(), observations.begin(), observations.end());
    return current;
  }

  void PredictionLayer::activate()
  {
    //  if we're stopped we need to re-subscribe to topics
    for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
    {
      if (observation_subscribers_[i] != NULL)
        observation_subscribers_[i]->subscribe();
    }
    for (unsigned int i = 0; i < observation_buffers_.size(); ++i)
    {
      if (observation_buffers_[i])
        observation_buffers_[i]->resetLastUpdated();
    }
  }

  void PredictionLayer::deactivate()
  {
    for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
    {
      if (observation_subscribers_[i] != NULL)
        observation_subscribers_[i]->unsubscribe();
    }
  }

  void PredictionLayer::updateRaytraceBounds(double ox, double oy, double wx, double wy,
                                             double range, double* min_x, double* min_y,
                                             double* max_x, double* max_y)
  {
    double dx = wx - ox, dy = wy - oy;
    double full_distance = hypot(dx, dy);
    double scale = min(1.0, range / full_distance);
    double ex = ox + dx * scale, ey = oy + dy * scale;
    touch(ex, ey, min_x, min_y, max_x, max_y);
  }

  void PredictionLayer::reset()
  {
    deactivate();
    resetMaps();
    current_ = true;
    initialize_ = false;
    activate();
  }

  bool PredictionLayer::resetLayerCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
    ROS_WARN("[ObstacleLayer] Clearing by Service Call");
    reset();
    res.success = true;
    res.message = "[ObstacleLayer] Clearing by Service Call";
    return true;
  }

} // namespace prediction_layer