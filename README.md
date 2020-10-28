# **Kidnapped Vehicle** 

##  Implement localization through a 2D particle filter in C++

### Recover vehicle's state from uncertain control and measurement environment, given known map data.

---

**Kidnapped Vehicle Project**

The goals / steps of this project are the following:
* The robot has been kidnapped and transported to a new location
* Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data
* Implement a 2 dimensional particle filter in C++
* Particle filter will be given a map and some initial localization information (analogous to what a GPS would provide)
* At each time step, the filter will also get observation and control data.

[//]: # (Image References)

[image1]: ./Images_forReadMe/particlefilter.gif "particle filter"
[image2]:  ./Images_forReadMe/particlefilter.mp4 "particle filter"

---
### README

- A 2D particle filter is implemented in C++ for localization which can be found in the [src folder](./src)

- Below is the result of the localization process through the particle filter:

![alt text][image1]

- [Full video](./Images_forReadMe/particlefilter.mp4)

- **Accuracy**: The particle filter localized vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in [src/main.cpp](./src/main.cpp).

- **Performance**: The particle filter completed execution within the time of 100 seconds.

### Repo contained

#### 1. uWebSockets

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO.

#### 2. Simulator

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

#### 3. Functional Code

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions

### Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```
#### Inputs to the Particle Filter
* Inputs to the particle filter can be found in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

#### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.

### Conclusion
* Particle filters are the easiest to program and most flexible
* Below is a quick comparison chart of differenet filters:

|         | state space | belief | efficiency | in robotics |
| ------------- | ------------- | ------------- | ------------- | ------------- |
| histogram filters | discrete | multimodal | exponential | approximate |
| kalman filters | continuous | unimodal | quadratic | approximate |
| particle filters | continuous | multimodal | - | approximate |

### Additional resources/ information

* Histogram Filter, Kalman Filter and Particle Filter are all Bayes filters and can be used to solve localization problem, they differ in the fact that different mathematical representation of system state distribution is assumed in each filter. Please read [here](https://www.deepideas.net/robot-localization-recursive-bayesian-estimation/) for more information.
* The [Robust and Precise Vehicle Localization based on Multi-sensor Fusion in Diverse City Scenes](https://arxiv.org/abs/1711.05805) paper published by Baidu Appolo team fuses information from complementary sensors such as GNSS, Lidar, and IMU to achieve centimeter level accuracy in various challenging scenes including urban downtown, highways, and tunnels.
* This [paper](http://robots.stanford.edu/papers/thrun.mapping-tr.pdf) is from Udacity's founder Sebastian Thrun, while from 2002, is still relevant for many different methods of mapping used today in robotics.

#### Simultaneous Localization and Mapping (SLAM)

* The below papers cover Simultaneous Localization and Mapping (SLAM) - which as the name suggests, combines localization and mapping into a single algorithm without a map created beforehand.
- [Past, Present, and Future of Simultaneous Localization And Mapping: Towards the Robust-Perception Age by C. Cadena, et. al.](https://arxiv.org/abs/1606.05830)
- [Navigating the Landscape for Real-time Localisation and Mapping for Robotics and Virtual and Augmented Reality by S. Saeedi, et. al.](https://arxiv.org/abs/1808.06352)























