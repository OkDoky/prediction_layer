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

using costmap_2d::ObservationBuffer;
using costmap_2d::Observation;

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
    polygon_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("transformed_footprint",0);

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
    boost::recursive_mutex::scoped_lock lock(lock_);
    data_update_time = ros::Time::now();
    _obstacles = msg;
    initialize_ = true;
  }

  void PredictionLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                    double *min_x, double *min_y, double *max_x, double *max_y)
  {
    // not initialized
    if (!initialize_)
      return;
    // disable prediction layer
    if (!enabled_)
      return;

    boost::recursive_mutex::scoped_lock lock(lock_);

    data_update_duration = ros::Time::now() - data_update_time;
    // over patience of data durations
    if (data_update_duration.sec > robot_data_patience_)
      return;
    // cannot detect dynamic obstacles
    if (_obstacles.circles.size() == 0)
      return;

    // updateBound main function
    try
    {
      geometry_msgs::PoseStamped rpose;
      rpose.pose.position.x = robot_x;
      rpose.pose.position.y = robot_y;
      tf2::Quaternion robot_quat;
      robot_quat.setRPY(0,0, robot_yaw);
      tf2::convert(robot_quat, rpose.pose.orientation);

      double min_x_ = 1000, min_y_ = 1000;
      double max_x_ = -1000, max_y_ = -1000;

      for (int i = 0; i < _obstacles.circles.size(); i++)
      {
        double dist = distanceCalculate(rpose.pose.position.x,
                                        rpose.pose.position.y,
                                        _obstacles.circles[i].center.x,
                                        _obstacles.circles[i].center.y)
                                        - _obstacles.circles[i].radius;
        // distance filter (range filter)
        if (dist < max_obstacle_range_)
        {
          _dist_range_obstacles.circles.push_back(_obstacles.circles[i]);
        }
        //
        double _range_x, _range_y;
        int8_t _sign_x, _sign_y;
        _sign_x = getSign(_obstacles.circles[i].center.x);
        _sign_y = getSign(_obstacles.circles[i].center.y);
        _range_x = _obstacles.circles[i].center.x + _sign_x * _obstacles.circles[i].radius;
        _range_y = _obstacles.circles[i].center.y + _sign_y * _obstacles.circles[i].radius;
        if (min_x_ > _range_x)
          min_x_ = _range_x;
        if (min_y_ > _range_y)
          min_y_ = _range_y;
        if (max_x_ < _range_x)
          max_x_ = _range_x;
        if (max_y_ < _range_y)
          max_y_ = _range_y;
      }

      *min_x = std::min(*min_x, min_x_);
      *min_y = std::min(*min_y, min_y_);
      *max_x = std::max(*max_x, max_x_);
      *max_y = std::max(*max_y, max_y_);
      ROS_DEBUG("[PredictionLayer] udateBounds Costmap size : [[%f, %f],[%f,%f]]",
                *min_x, *min_y, *max_x, *max_y);
    }
    catch (tf2::LookupException& ex){
      ROS_ERROR("[PredictionLayer] No Transform available Error: %s\n", ex.what());
      return;
    }
    catch (tf2::ConnectivityException& ex){
      ROS_ERROR("[PredictionLayer] Connectivity Error: %s", ex.what());
      return;
    }
    catch (tf2::ExtrapolationException& ex){
      ROS_ERROR("[PredictionLayer] Extrapolation Error: %s", ex.what());
      return;
    }
  }

  void PredictionLayer::updateCosts(costmap_2d::Costmap2D &master_grid, 
                                    int min_i, int min_j, int max_i, int max_j)
  {
    if (!initialize_)
      return;
    if (!enabled_)
      return;

    boost::recursive_mutex::scoped_lock lock(lock_);

    data_update_duration = ros::Time::now() - data_update_time;
    if (data_update_duration.sec > robot_data_patience_)
      return;
    if (_dist_range_obstacles.circles.size() == 0)
      return;

    // get costmap informations
    costmap_2d::Costmap2D *costmap = layered_costmap_->getCostmap();
    _costmap_resolution = costmap->getResolution();

    unsigned int mx, my;
    unsigned int min_x, min_y;
    unsigned int max_x, max_y;

    for (int i = 0; i < _dist_range_obstacles.circles.size(); i++)
    {
      if (master_grid.worldToMap(_dist_range_obstacles.circles[i].center.x - _dist_range_obstacles.circles[i].radius,
                                  _dist_range_obstacles.circles[i].center.y - _dist_range_obstacles.circles[i].radius,
                                  min_x, min_y)
          && master_grid.worldToMap(_dist_range_obstacles.circles[i].center.x + _dist_range_obstacles.circles[i].radius,
                                    _dist_range_obstacles.circles[i].center.y + _dist_range_obstacles.circles[i].radius,
                                    max_x, max_y))
        ROS_DEBUG("[PredictionLayer] updateCosts CostMap size : [[%f,%f],[%f,%f]]", min_x, min_y, max_x, max_y);
      
      min_x = std::max(int(min_x), min_i);
      min_y = std::max(int(min_y), min_j);
      max_x = std::min(int(max_x), max_i);
      max_y = std::min(int(max_y), max_j);

      if (min_x >= max_x || min_y >= max_y)
        ROS_ERROR("[PredictionLayer] Prediction Costmap size: [[%d,%d],[%d,%d]]", min_x, min_y, max_x, max_y);
      
      // fillin cost
      for (int x = min_x; x < max_x: x++)
      {
        for (int y = min_y; y < max_y; y++)
          master_grid.setCost(x, y, LETHAL_OBSTACLE);
      }
    }

    for (int i = 0; i < _dist_range_obstacles.circles.size(); i++)
    {
      if (master_grid.worldToMap(_dist_range_obstacles.circles[i].center.x - _dist_range_obstacles.circles[i].radius,
                                  _dist_range_obstacles.circles[i].center.y - _dist_range_obstacles.circles[i].radius,
                                  min_x, min_y)
          && master_grid.worldToMap(_dist_range_obstacles.circles[i].center.x + _dist_range_obstacles.circles[i].radius,
                                    _dist_range_obstacles.circles[i].center.y + _dist_range_obstacles.circles[i].radius,
                                    max_x, max_y))
        ROS_DEBUG("[PredictionLayer] updateCosts Costmap size : [[%f,%f],[%f,%f]]",min_x, min_y, max_x, max_y);
      std::vector<geometry_msgs::Point> _polygon;
      circle_to_circumscribe_polygon::circleToCircumscribePolygon(_dist_range_obstacles.circles[i], _polygon);
      geometry_msgs::PolygonStamped footprint;
      footprint.header.frame_id = "map";
      footprint.header.stamp = ros::Time::now();
      geometry_msgs::Polygon polygon_point;
      for (int i = 0; i < _polygon.size(); i++)
      {
        polygon_point.points.push_back(_polygon[i]);
      }
      footprint.polygon = polygon_point;
      polygon_pub_.publish(footprint);
      setPolygonCost(master_grid, polygon_point, min_i, min_j, max_i, max_j);
    }
  }

  bool PredictionLayer::resetLayerCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
    ROS_WARN("[ObstacleLayer] Clearing by Service Call");
    reset();
    res.success = true;
    res.message = "[ObstacleLayer] Clearing by Service Call";
    return true;
  }

  void PredictionLayer::raytrace(int x0, int y0, int x1, int y1, std::vector<PointInt> &cells)
  {
    return;
  }

  void PredictionLayer::polygonOutlineCells(const std::vector<PointInt> &polygon,std::vector<PointInt> &polygon_cells)
  {
    return;
  }

  void PredictionLayer::rasterizePolygon(const std::vector<PointInt> &polygon,
                                         std::vector<PointInt> &polygon_cells,
                                         bool fill)
  {
    return;
  }

  void PredictionLayer::setPolygonCost(costmap_2d::Costmap2D &master_grid, 
                                       const geometry_msgs::Polygon &polygon,
                                       int min_i, int min_j, int max_i, int max_j)
  {
    std::vector<PointInt> map_polygon;
    for (unsigned int i = 0; i < polygon.size(); ++i)
    {
      PointInt loc;
      master_grid.worldToMapNoBounds(polygon[i].x, polygon[i].y, loc.x, loc.y);
      map_polygon.push_back(loc);
    }

    std::vector<PointInt> polygon_cells;
    rasterizePolygon(map_polygon, polygon_cells, fill_polygon);
    
    for (unsigned int i = 0; i < polygon_cells.size(); ++i)
    {
      int mx = polygon_cells[i].x;
      int my = polygon_cells[i].y;
      if (mx < min_i || mx >= max_i)
        continue;
      if (my < min_j || my >= max_j)
        continue;
        master_grid.setCost(mx, my, LETHAL_OBSTACLE);
    }
  }

}
