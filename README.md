# ***PredictionLayer***

---

### *contents*

1. Concepts
2. Getting Started
3. Parameters
4. Example guide
5. ToDo



### 1. Concepts

---

This package is only for `ros-melodic` user.

using obstacle_detector package for perception function, So you have to use 2D Lidar and obstacle_detector package.





### 2. Getting Started

---

install guide before getting started

1. install source from github and install dependency packages

   1. [obstacle_detector|!https://github.com/tysik/obstacle_detector.git] is default reference dependency package for this layer, so you have to install this package first

2. After, install obstacle_detector package, follow the guide below

   1. install git source from github

      ```bash
      cd ~/usr_ws/src
      git clone https://github.com/OkDoky/prediction_layer.git
      cd ~/usr_ws
      ```

   2. install dependency packages

      ```bash
      rosdep install -r -y --from-paths src --ignore-src
      catkin_make
      ```



### 3. Parameters

---

- `~enabled` (`bool`, default : true) - enable/disable layer function
- `~footprint_clearing_enabled` (`bool`, default : true) - enable/disable clear costs inside robot footprint
- `~object_source` (`string`, default : "") - topic name for input data, the input data type is `obstacle_detector/Obstacles`, default name is `obstacles`
- `~observation_persistence` (double, default : 0.2) - 
- `~expected_update_rate` (double, default : 0.2) - 
- `~clearing` (`bool`, default : true) - enable/disable clear the marked buffer
- `~marking` (`bool`, defualt : true) - 
- `~transform_tolerance` (`double`, default : 0.2) - 
- `~track_unknown_space` (`bool`, default : true) - 
- `~combination_method` (`int`, default : 0) - 



### 4. Example guide

---

example for costmap layer

1. example guide



### 5. ToDo

---

- [ ] non-buffer layer
- [ ] reduce cycle time using reactiveX (rxcpp)
- [ ] refactoring