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

using namespace costmap_2d;

namespace prediction_layer
{
  PredictionLayer::PredictionLayer() : _dsrv(NULL)
  {
  }
  
  PredictionLayer::~PredictionLayer()
  {
    if (_dsrv)
      delete _dsrv;
  }
  void PredictionLayer::reset()
  {
    initialize_ = false;
  }

  void PredictionLayer::onInitialize()
  {
    ros::NodeHandle nh_param("~/" + name_);
    ros::NodeHandle nh;

    current_ = true;
    PredictionLayer::matchSize();
    
    costmap_2d::Costmap2D *costmap = layered_costmap_->getCostmap();
    _costmap_resolution = costmap->getResolution();
    rolling_window_ = layered_costmap_->isRolling();

    double transform_tolerance;
    nh_param.param("transform_tolerance", transform_tolerance, 0.2);

    std::string topics_string;
    nh_param.param("object_source", topics_string, std::string(""));
    ROS_INFO("  Subscribed to Topics: %s", topics_string.c_str());

    obstacles_sub_ = nh.subscribe<obstacle_detector::Obstacles>(topics_string.c_str(), 1, &PredictionLayer::ObstaclesCallback, this);
    polygon_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("transformed_footprint",0);
    source_frame = "";
    initialize_ = true;
    
    _dsrv = new dynamic_reconfigure::Server<PredictionLayerConfig>(nh);
    dynamic_reconfigure::Server<PredictionLayerConfig>::CallbackType cb = boost::bind(
      &PredictionLayer::reconfigureCB, this, _1, _2);
    _dsrv->setCallback(cb);

    // added
    reset_layer_ = nh.advertiseService("reset_layer", &PredictionLayer::resetLayerCallback, this);
  }

  void PredictionLayer::reconfigureCB(PredictionLayerConfig & config, uint32_t level)
  {
    enabled_ = config.enabled;
    footprint_clearing_enabled_ = config.footprint_clearing_enabled;
    max_obstacle_range_ = config.max_obstacle_range;
  }

  void PredictionLayer::ObstaclesCallback(const obstacle_detector::Obstacles msg)
  {
    boost::recursive_mutex::scoped_lock lock(lock_);
    source_frame = msg.header.frame_id;
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
    if (data_update_duration.sec > callback_data_patience_)
      return;
    // cannot detect dynamic obstacles
    if (_obstacles.circles.size() == 0)
      return;

    if (rolling_window_)
      updateOrigin(robot_x - getSizeInMetersX() / 2.0, robot_y - getSizeInMetersY() /2.0);
    useExtraBounds(min_x, min_y, max_x, max_y);
    bool current = true;
    
    // marking

    // clearing

    // update current status

    // raytrace freespace
    for (unsigned int i = 0; i < _obstacles.circles.size(); i++)
    {
      raytrace()
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
    if (data_update_duration.sec > callback_data_patience_)
      return;
    if (_dist_range_obstacles.circles.size() == 0)
      return;

    // clear inside footprint costs 
    if (footprint_clearing_enabled_)
    {
      setConvexPolygonCost(transformed_footprint_, costmap::FREE_SPACE);
    }

    // // get costmap informations
    // costmap_2d::Costmap2D *costmap = layered_costmap_->getCostmap();
    // _costmap_resolution = costmap->getResolution();

    unsigned int mx, my;
    unsigned int min_x, min_y;
    unsigned int max_x, max_y;

    for (int i = 0; i < _dist_range_obstacles.circles.size(); i++)
    {
      // if (master_grid.worldToMap(_dist_range_obstacles.circles[i].center.x - _dist_range_obstacles.circles[i].radius,
      //                             _dist_range_obstacles.circles[i].center.y - _dist_range_obstacles.circles[i].radius,
      //                             min_x, min_y)
      //     && master_grid.worldToMap(_dist_range_obstacles.circles[i].center.x + _dist_range_obstacles.circles[i].radius,
      //                               _dist_range_obstacles.circles[i].center.y + _dist_range_obstacles.circles[i].radius,
      //                               max_x, max_y))
      //   ROS_DEBUG("[PredictionLayer] updateCosts CostMap size : [[%d,%d],[%d,%d]]", min_x, min_y, max_x, max_y);
      
      min_x = std::max(int(min_x), min_i);
      min_y = std::max(int(min_y), min_j);
      max_x = std::min(int(max_x), max_i);
      max_y = std::min(int(max_y), max_j);

      if (min_x >= max_x || min_y >= max_y)
        ROS_ERROR("[PredictionLayer] Prediction Costmap size: [[%d,%d],[%d,%d]]", min_x, min_y, max_x, max_y);
      
      // fillin cost
      for (int x = min_x; x < max_x; x++)
      {
        for (int y = min_y; y < max_y; y++)
          master_grid.setCost(x, y, LETHAL_OBSTACLE);
      }
    }

    for (int i = 0; i < _dist_range_obstacles.circles.size(); i++)
    {
      // if (master_grid.worldToMap(_dist_range_obstacles.circles[i].center.x - _dist_range_obstacles.circles[i].radius,
      //                             _dist_range_obstacles.circles[i].center.y - _dist_range_obstacles.circles[i].radius,
      //                             min_x, min_y)
      //     && master_grid.worldToMap(_dist_range_obstacles.circles[i].center.x + _dist_range_obstacles.circles[i].radius,
      //                               _dist_range_obstacles.circles[i].center.y + _dist_range_obstacles.circles[i].radius,
      //                               max_x, max_y))
      //   ROS_DEBUG("[PredictionLayer] updateCosts Costmap size : [[%d,%d],[%d,%d]]",min_x, min_y, max_x, max_y);
      geometry_msgs::Polygon _polygon;
      circleToCircumscribePolygon(_dist_range_obstacles.circles[i], _polygon);
      geometry_msgs::PolygonStamped footprint;
      footprint.header.frame_id = "r1/base_scan";
      footprint.header.stamp = ros::Time::now();
      geometry_msgs::Polygon polygon_point;
      for (int i = 0; i < _polygon.points.size(); i++)
      {
        polygon_point.points.push_back(_polygon.points[i]);
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
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    PointInt pt;
    pt.x = x0;
    pt.y = y0;
    int n = 1 + dx + dy;
    int x_inc = (x1 > x0) ? 1 : -1;
    int y_inc = (y1 > y0) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;

    for (; n > 0; --n) {
        cells.push_back(pt);

        if (error > 0) {
            pt.x += x_inc;
            error -= dy;
        } else {
            pt.y += y_inc;
            error += dx;
        }
    }
  }

  void PredictionLayer::raytraceFreespace(const DynamicObstacles& clearing_obstacles,
                                          double* min_x, double* min_y, double* max_x, double* max_y)
  {
    // double ox = clearing_obstacles.pose_.x;
    // double oy = clearing_obstacles.pose_.y;
    obstacle_detector::Obstacles obs = *(clearing_obstacles);

    

  }

  // void PredictionLayer::polygonOutlineCells(const std::vector<PointInt> &polygon,
  //                                           std::vector<PointInt> &polygon_cells)
  // {
  //   for (unsigned int i = 0; i < polygon.size() - 1; ++i) {
  //       raytrace(polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y, polygon_cells);
  //   }
  //   if (!polygon.empty()) {
  //       unsigned int last_index = polygon.size() - 1;
  //       // we also need to close the polygon by going from the last point to the first
  //       raytrace(polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y, polygon_cells);
  //   }
  // }

  // void PredictionLayer::rasterizePolygon(const std::vector<PointInt> &polygon,
  //                                        std::vector<PointInt> &polygon_cells)
  // {
  //   // this implementation is a slighly modified version of Costmap2D::convexFillCells(...)

  //   //we need a minimum polygon of a traingle
  //   if (polygon.size() < 3)
  //       return;

  //   //first get the cells that make up the outline of the polygon
  //   polygonOutlineCells(polygon, polygon_cells);

  //   //quick bubble sort to sort points by x
  //   PointInt swap;
  //   unsigned int i = 0;
  //   while (i < polygon_cells.size() - 1) {
  //       if (polygon_cells[i].x > polygon_cells[i + 1].x) {
  //           swap = polygon_cells[i];
  //           polygon_cells[i] = polygon_cells[i + 1];
  //           polygon_cells[i + 1] = swap;

  //           if (i > 0)
  //               --i;
  //       } else
  //           ++i;
  //   }

  //   i = 0;
  //   PointInt min_pt;
  //   PointInt max_pt;
  //   int min_x = polygon_cells[0].x;
  //   int max_x = polygon_cells[(int)polygon_cells.size() - 1].x;

  //   //walk through each column and mark cells inside the polygon
  //   for (int x = min_x; x <= max_x; ++x) {
  //       if (i >= (int)polygon_cells.size() - 1)
  //           break;

  //       if (polygon_cells[i].y < polygon_cells[i + 1].y) {
  //           min_pt = polygon_cells[i];
  //           max_pt = polygon_cells[i + 1];
  //       } else {
  //           min_pt = polygon_cells[i + 1];
  //           max_pt = polygon_cells[i];
  //       }

  //       i += 2;
  //       while (i < polygon_cells.size() && polygon_cells[i].x == x) {
  //           if (polygon_cells[i].y < min_pt.y)
  //               min_pt = polygon_cells[i];
  //           else if (polygon_cells[i].y > max_pt.y)
  //               max_pt = polygon_cells[i];
  //           ++i;
  //       }

  //       PointInt pt;
  //       //loop though cells in the column
  //       for (int y = min_pt.y; y < max_pt.y; ++y) {
  //           pt.x = x;
  //           pt.y = y;
  //           polygon_cells.push_back(pt);
  //       }
  //   }
  // }

  // void PredictionLayer::setPolygonCost(costmap_2d::Costmap2D &master_grid, 
  //                                      const geometry_msgs::Polygon &polygon,
  //                                      int min_i, int min_j, int max_i, int max_j)
  // {
  //   std::vector<PointInt> map_polygon;
  //   for (unsigned int i = 0; i < polygon.points.size(); ++i)
  //   {
  //     PointInt loc;
  //     master_grid.worldToMapNoBounds(polygon.points[i].x, polygon.points[i].y, loc.x, loc.y);
  //     map_polygon.push_back(loc);
  //   }

  //   std::vector<PointInt> polygon_cells;
  //   rasterizePolygon(map_polygon, polygon_cells);
    
  //   for (unsigned int i = 0; i < polygon_cells.size(); ++i)
  //   {
  //     int mx = polygon_cells[i].x;
  //     int my = polygon_cells[i].y;
  //     if (mx < min_i || mx >= max_i)
  //       continue;
  //     if (my < min_j || my >= max_j)
  //       continue;
  //       master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  //   }
  // }

  //======================================================= Like a obstacle layer
  void PredictionLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw,
                                        double* min_x, double* min_y, double* max_x, double* max_y)
  {
    if (!footprint_clearing_enabled_) return;
    transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

    for (unsigned int i = 0; i < transformed_footprint_.size(); i++)
    {
      touch(transformed_footprint_[i].x, transfromed_footprint_[i].y, min_x, min_y, max_x, max_y);
    }
  }


}
