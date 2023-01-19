#!/usr/bin/python

import math
import traceback
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

class Point:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        
class Vector3:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

class CircleObstacle:
    def __init__(self):
        self.center = Point()
        self.velocity = Vector3()
        self.radius = 0.0
        self.true_radius = 0.0

def circle_to_polygon(circle):
    try:
        c_x = circle.center.x
        c_y = circle.center.y
        c_theta = math.atan2(circle.velocity.y, circle.velocity.x)
        
        radius = circle.radius
        
        front_offset = 0.0
        rear_offset = 1.0
        right_offset = 1.0
        left_offset = 0.0
        
        _right = radius + right_offset
        _left = radius + left_offset
        _front = radius + front_offset
        _rear = radius + rear_offset
        
        polygon = []
        point_rear_left, point_rear_right, point_front_right, point_front_left = Point(), Point(), Point(), Point()
        point_front_left.x   = c_x - _front * math.cos(c_theta) - _left  * math.sin(c_theta)  
        point_front_left.y   = c_y - _front * math.sin(c_theta) + _left  * math.cos(c_theta)
        point_front_right.x  = c_x - _front * math.cos(c_theta) + _right * math.sin(c_theta)
        point_front_right.y  = c_y - _front * math.sin(c_theta) - _right * math.cos(c_theta)
        point_rear_right.x  = c_x + _rear  * math.cos(c_theta) + _right * math.sin(c_theta)
        point_rear_right.y  = c_y + _rear  * math.sin(c_theta) - _right * math.cos(c_theta)
        point_rear_left.x   = c_x + _rear  * math.cos(c_theta) - _left  * math.sin(c_theta)
        point_rear_left.y   = c_y + _rear  * math.sin(c_theta) + _left  * math.cos(c_theta)

        polygon.append(point_rear_left)
        polygon.append(point_rear_right)
        polygon.append(point_front_right)
        polygon.append(point_front_left)
        
        print("rear left \n\
              x : %s,\n y : %s\n\
              rear right \n\
              x : %s,\n y : %s\n\
              front right \n\
              x : %s,\n y : %s\n\
              front left \n\
              x : %s,\n y : %s\n"\
              %(point_rear_left.x, point_rear_left.y\
                ,point_rear_right.x, point_rear_right.y\
                ,point_front_right.x, point_front_right.y\
                ,point_front_left.x, point_front_left.y))
        return polygon
        
    except Exception as e:
        print(traceback.format_exc())


if __name__ == "__main__":
    # rospy.init_node("test circle to polygon")
    
    circle_data_set = CircleObstacle()
    circle_data_set.center.x = 1.0
    circle_data_set.center.y = 1.0
    circle_data_set.radius = 1.0
    circle_data_set.true_radius = 0.8
    circle_data_set.velocity.x = 1.0
    circle_data_set.velocity.y = 1.0
    
    
    
    # print(circle_data_set)
    
    polygon = circle_to_polygon(circle_data_set)
    
    fig, ax = plt.subplots()
    # print(type(ax))
    ax.set_title('circle to ploygon',fontsize=16)
    ax.set_xlim(-5.0, 5.0)
    ax.set_ylim(-5.0, 5.0)
    plt.xticks(np.arange(-5.0,5.0,0.5))
    plt.yticks(np.arange(-5.0,5.0,1.0))
    # ax.hlines("-Y")
    # ax.vlines("X")
    x = np.array([]); y = np.array([])
    for i in range(len(polygon)):
        x = np.append(x, np.array([-polygon[i].y]))
        y = np.append(y, np.array([polygon[i].x]))
    x = np.append(x, np.array([-polygon[0].y]))
    y = np.append(y, np.array([polygon[0].x]))
    plt.title('circle to polygon')
    plt.plot(x,y, '.-')
    plt.grid(True, alpha=0.1)
    
    ax.add_patch(
        patches.Circle(
            xy=(-circle_data_set.center.y,circle_data_set.center.x),
            radius=circle_data_set.radius
        )
    )
    ax.add_patch(
        patches.Arrow(
            -circle_data_set.center.y, circle_data_set.center.x,
            -circle_data_set.velocity.y, circle_data_set.velocity.x,
            width = 0.1,
            edgecolor = 'deeppink',
            facecolor = 'lightgray'
        )
    )
    # ax.set(xlable='-Y',ylable='X')
    # ax.set_xlable('-Y')
    # ax.set_ylable('X')
    
    # ax.plot(x,y,'ro',alpha=0.5)
    
    # plot_ploygon = np.array([])
    # for i in range(len(polygon)):
    #     plot_ploygon = np.append(plot_ploygon, np.array([[polygon[i].x,polygon[i].y]]))
    
    # ax.add_patch(patches.Polygon(
    #     plot_ploygon,
    #     closed = True,
    #     edgecolor = 'red',
    #     fill = False,
    # ))
    plt.show()
    
    print("close program")
    
    