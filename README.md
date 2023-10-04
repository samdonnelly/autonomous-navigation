# Autonomous Navigation 

Application and simulation code for an autonomous robot boat. 

## Auto Nav
Embedded application code for an autonomous boat and its ground station. The boat runs on an STM32F4 and communicates with the operators ground station using radio. The boat runs autonomous waypoint missions but can also be remotely operated via the ground station if needed. 

The ground station is the system that stays with the operator and allows for remote control and communication with the robot. The code is written for the STM32F4, the operator communicates with the system using a serial terminal, and the system talks to the robot over radio. 

## Missing Planning 
Scripts that allow the operator to send and receive mission data. They act as an interface between the operator and the ground station. 

## Simulation 
Scripts that perform navigation calculations such as using the current and desired GPS coordinates to determining the heading between the two points. These are used to test calculations before implementation in real systems. 
