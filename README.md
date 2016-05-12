# fri-color-detect
A project for Freshmen Research Initiative that uses camera images to detect a person's shirt color and display it using an LED strip.

*********************
 Getting started 
*********************
The first step to this project is to download the rosserial package http://wiki.ros.org/rosserial_arduino

then download the Arduino IDE and install the ros_lib library a tutorial can be found here: 
http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

ros_lib allows the Arduino IDE to correctly compile ROS commands 

***********************************************
 Add code to package and upload to arduino 
***********************************************
Add the color_finder.cpp , and color_subscriber.ino files to your ROS workspace

Open the Arduino IDE and upload color_subscriber.ino

*****************
 Run the code
*****************
roscore

//new terminal

{play bag or start camera}

//If working with UT Segway bots and using kinnect then also kill arduino_driver and battery_diagnostics

//new terminal

rosrun {package name} color_finder.cpp

//new terminal

rosrun rosserial_python serial_node.py {the arduino serial port}

***********************
 File descriptions
***********************
when switching image sources change the subscriber in the color_finder file

color_finder.cpp is a node which uses images from either a bag or a webcam to detect people, once a person is found, 
a box will be drawn around them then a circle drawn in their midsection. The circle is where the code gets its rgb values.
The valued are then published through publisher "/color_chatter"

serial_node.py is a file from rosserial which acts as a sort of bridge between ROS and Arduino. It allows the arduino to communicate with ROS

