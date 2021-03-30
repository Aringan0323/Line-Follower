# Line-Follower

## Launch file
The launch file included is <code>line_follower.launch</code>. In order to run this gazebo simulation, type the following command into the terminal
<code>roslaunch (PACKAGE_NAME) line_follower.launch</code>. This will start the gazebo simulation with a yellow line and a wafflebot.

## Main
The file <code>main.py</code> in the <code>src</code> folder initiates the main control loop that publishes commands to the wafflebot to follow the yellow line in the world.
To run this file, type the following command into a new terminal <code>rosrun (PACKAGE_NAME) main.py</code>. What the robot will do is first check to see if there is a yellow line
directly in front of it. If there is no yellow line directly in front of it, it will rotate and search its entire field of vision until it spots a yellow line. Once a yellow line
is spotted, it will move towards it until the yellow line is directly in front of it. When the yellow line is directly in front of it, the wafflebot will begin to follow the line.
Once the robot reaches the end of the line, it will rotate around and follow the yellow line in the opposite direction. The robot will continue to repeat the last two steps until
the program has been terminated.

## Video Demonstration
https://drive.google.com/file/d/1cdc1yIa2zvzS-jpOmWqf_urmHvpf0Jqg/view?usp=sharing
