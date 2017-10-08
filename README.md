# Traget Tracking with Sensor Fusion
# Extended-Kalman-Filter implementation

## Overview:
This project uses sensor measurments from LIDAR and RADAR for target tracking with EKF.

## Simulator Info:
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

## Results:
#### Dataset 1 From the simulator:
![Dataset1](https://github.com/aranga81/Extended-Kalman-Filter/blob/master/results/dataset1.JPG)

#### Dataset 2 From the simulator:
![Dataset2](https://github.com/aranga81/Extended-Kalman-Filter/blob/master/results/dataset2.JPG)

## Running the Code:
Follow the steps:
- Clone this repository
- change working directory to this repo: cd <repo>
- Make new directory build or cd <existing-build-dir>
- Run cmake .. and make [Assuming you already have the correct version of Cmake and make]
- Run the code ./ExtendedKF
- Open the simulator from the directory you dowloaded to and run the 1/1 EKF - dataset 1 and 2

