# Autonomous Navigation 

Application and simulation code for autonomous robot navigation. 

## Ground Station 
Embedded code for the controller that operates the robots ground station. The ground station is the system that stays with the operator and allows for remote control and communication with the robot. The code is written for the STM32F4, the operator communicates with the system using a serial terminal, and the system talks to the robot over radio. 

Within this folder there is also a subdirectory containing scripts for mission planning. These scripts allow the operator to send and receive mission data. They act as an interface between the operator and the ground station. 

## Boat 
Embedded application code for a remotely operated and autonomous boat. The boat runs on an STM32F4 and communicates with the operators ground station using radio. 

## Simulation 
Scripts that perform navigation calculations such as using the current and desired GPS coordinates to determining the heading between the two points. These are used to test calculations before implementation in real systems. 
