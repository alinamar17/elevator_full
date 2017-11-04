# elevator_full
## General description
The package contains two different parts allowing the robot to use elevator:
1. Button recognition and pushing allows robot to order the elevator.
2. Door opening recognition and moving forward allows the robot to enter the elevator (or any other place) after the door is open.

## Button recognition and pushing
In this particular project we assume that the button is the only red object the robot can see with his Kinect2 camera. There are some troubles with recognition if the button isn't the only red object on the image so make sure it's the only one.
There are several restrictions on the button location caused by robot arm abilities: 
1. The button must be at most 1.3 meter high from the floor so that the robot can get it.
2. The robot have to be on distance 0.5-0.8 meter from the wall button located on.

Also, if you don't run the code from the robot's computer (ssh or another remote control) make sure that the delay of transformation between kinect2_rgb_optical_frame and base_footprint < 200. You can check it by running:
$ rosrun tf tf_monitor /kinect2_rgb_optical_frame /base_footprint 
Even if the delay is less than 200 msec it will take time to syncronize (can take few minutes) so it's strongly recommended to run the code from the robot computer.

## Door open recognition
This part is using the laser scaner to recognize while the elevator's door is open and move 1 meter forward.
'gmapping:=true' part of the roslaunch is necessary for correct work. 
Before you run this code on the real robot we strongly recommend to give the robot some time to build the map with gmapping so it will recognize things and changes properly.
