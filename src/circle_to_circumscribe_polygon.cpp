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

#include <prediction_layer/circle_to_circumscribe_polygon.h>


void circleToCircumscribePolygon(const obstacle_detector::CircleObstacle circle, 
                                        geometry_msgs::Polygon &polygon)
{
    double c_x, c_y, c_theta;
    c_x = circle.center.x;
    c_y = circle.center.y;
    c_theta = atan2(circle.velocity.y, circle.velocity.x);
    
    double radius;
    radius = circle.radius;
    
    double front_offset, rear_offset, right_offset, left_offset;
    front_offset = 0.0;
    rear_offset = 0.0;
    right_offset = 0.0;
    left_offset = 0.0;
    
    double _right, _left, _rear, _front;
    _right = radius + right_offset;
    _left = radius + left_offset;
    _front = radius + front_offset;
    _rear = radius + rear_offset;

    // int _edge = 4;
    // polygon.assign(_edge, geometry_msgs::Point);

    geometry_msgs::Point32 point_rear_left, point_rear_right, point_front_right, point_front_left;
    point_front_left.x  = c_x - (_front) * cos(c_theta) - (_left) * sin(c_theta);
    point_front_left.y  = c_y - (_front) * sin(c_theta) + (_left) * cos(c_theta);
    point_front_right.x = c_x - (_front) * cos(c_theta) + (_right) * sin(c_theta);
    point_front_right.y = c_y - (_front) * sin(c_theta) - (_right) * cos(c_theta);
    point_rear_right.x  = c_x + (_rear) * cos(c_theta)  + (_right) * sin(c_theta);
    point_rear_right.y  = c_y + (_rear) * sin(c_theta)  - (_right) * cos(c_theta);
    point_rear_left.x   = c_x + (_rear) * cos(c_theta)  - (_left) * sin(c_theta);
    point_rear_left.y   = c_y + (_rear) * sin(c_theta)  + (_left) * cos(c_theta);
    polygon.points.push_back(point_rear_left);
    polygon.points.push_back(point_rear_right);
    polygon.points.push_back(point_front_right);
    polygon.points.push_back(point_front_left);
}

void circleToCircumscribePolygon(const obstacle_detector::CircleObstacle circle, 
                                        geometry_msgs::Polygon &polygon,
                                        std::map<std::string, double> &offsets)
{
    double c_x, c_y, c_theta;
    c_x = circle.center.x;
    c_y = circle.center.y;
    c_theta = atan2(circle.velocity.y, circle.velocity.x);

    double radius;
    radius = circle.radius;

    double front_offset, rear_offset, right_offset, left_offset;
    front_offset = offsets["front"];
    rear_offset = offsets["rear"];
    right_offset = offsets["right"];
    left_offset = offsets["left"];

    double _right, _left, _rear, _front;
    _right = radius + right_offset;
    _left = radius + left_offset;
    _front = radius + front_offset;
    _rear = radius + rear_offset;

    // int _edge = 4;
    // polygon.assign(_edge, geometry_msgs::Point);

    geometry_msgs::Point32 point_rear_left, point_rear_right, point_front_right, point_front_left;
    point_front_left.x  = c_x - (_front) * cos(c_theta) - (_left) * sin(c_theta);
    point_front_left.y  = c_y - (_front) * sin(c_theta) + (_left) * cos(c_theta);
    point_front_right.x = c_x - (_front) * cos(c_theta) + (_right) * sin(c_theta);
    point_front_right.y = c_y - (_front) * sin(c_theta) - (_right) * cos(c_theta);
    point_rear_right.x  = c_x + (_rear) * cos(c_theta)  + (_right) * sin(c_theta);
    point_rear_right.y  = c_y + (_rear) * sin(c_theta)  - (_right) * cos(c_theta);
    point_rear_left.x   = c_x + (_rear) * cos(c_theta)  - (_left) * sin(c_theta);
    point_rear_left.y   = c_y + (_rear) * sin(c_theta)  + (_left) * cos(c_theta);
    polygon.points.push_back(point_rear_left);
    polygon.points.push_back(point_rear_right);
    polygon.points.push_back(point_front_right);
    polygon.points.push_back(point_front_left);
    std::cout << "rear left : x : " << point_rear_left.x << ", y : " << point_rear_left.y << std::endl;
    std::cout << "rear right : x : " << point_rear_right.x << ", y : " << point_rear_right.y << std::endl; 
    std::cout << "front right : x : " << point_front_right.x << ", y : " << point_front_right.y << std::endl; 
    std::cout << "front left : x : " << point_front_left.x << ", y : " << point_front_left.y << std::endl;
}