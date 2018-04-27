# Term2 Project1: Extended Kalman Filter
Self-Driving Car Engineer Nanodegree Program

## Overview:
This project implemented Extended Kalman Filter to estimate states with measurements from Radar and Lidar. The implementation is based on C++, with basically revised "FusionEKF.cpp", "kalman_filter.cpp", and "tools.cpp". 

The code can be well compiled without errors, and the RMSE values are [0.0961, 0.0846, 0.4484, 0.4330] after running Dataset1 on the simulator, which are all within the limit of requirements.

## Procedure:
To meet all criteria, the implementation is based on the knowledge and code taught in the class. In the "FusionEKF.cpp", I have added initialization of matrices F, P, and H_laser in the constructor. It can handle the first measurement by checking the bool variable. In ProcessMeasurement function, I have added polar to cartesian conversion, Q matrix update, and H & R update depending which sensor. 

In the kalman_filter.cpp, I have added the Kalman filter loop to predict and update, with respect to two types of sensor. Also according to the notes, I have added the angle check to make it in between -pi and pi. 

In the tools.cpp, according to the code taught in class, I have added the function to calculate Jacobian matrix, as well as the function to calculate RMSE. 

## Test Result:
After running the "ExtendedKF" with the simulator's Dataset1, the filter can track the states well. Initially the RMSE value was a bit larger, and I have noticed that especially when the data has a relative spreaded points it has a little larger RMSE. Then I have changed the noise_ax and noise_ay from 5.0 to 10.0, and then the RMSE reduced about roughly 10%, and all of them are within the requirements. A sample screenshot is also included in the repo, "RMSE.png".

## Others:
After implemented the kalman_filter.cpp, I have noticed that for two kinds of sensors, there are parts of codes the same with each other, which is the Kalman filter loop. So I think in the future, this part can be merged to a new function to make the code more clear.

## Quoted from original README to build:
In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp, kalman_filter.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `




