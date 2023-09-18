# CompRobo FA2023: Warmup Project

Brooke Moss, Lauren Thorbecke, Swasti Jain

## Project Description:

As the warm-up project for A Computational Introduction to Robotics ("CompRobo") Fall 2023, we were tasked to use ROS2 tools to allow our Neato robotic vacuum to perform a number of tasks of increasing complexity.

The tasks are as follows:

-   Teleop: Control the Neato using keystrokes on a laptop. Using `WASD` keys, control the robot's forward/backward motion and turning.

-   Driving in a Square: Write a simple ROS node where the robot drives forward a set distance, turns 90 degrees, and repeats until it has drawn a square.

-   Wall Following: Have the Neato autonomously drive parallel to a wall, at a set distance from the wall.

-   Person Following: Have the Neato detect and follow a person walking in front of it, staying a set distance behind.

-   Obstacle Avoidance:

-   Multi-Behaviour:

## Implementation:

### Teleop

For the tele-op section of the project, the goal was to allow keystrokes on a laptop to manually control the robot's motion and orientation. To implement this, we decided to use a toggle-style control scheme. For example, a `W` keypress would cause the robot to move forward at a set speed until it received further instruction.

#### Code Structure

Within the Node object, we included the initialization method and two additional methods. One method, `drive()`, was a helper, taking a linear or angular velocity and sending a command to the Neato to move as such. To capture the keystrokes on the laptop, we defined a `getKey()` function (not within the Node object) that waited for the next keystroke and returned which key was pressed.

Then, within the method `run_loop()`, a loop continually calls `getKey()` to update the currently-pressed key. If the stop code `Ctrl + C` is pressed, the loop exits and the robot stops. Otherwise, a `W` keypress calls the `drive()` function to drive straight forward, `S` drives straight back, `A` turns left, and `D` turns right. When any key other than these 4 are pressed, the robot stops driving, but remains enabled and able to drive.

#### Issues

Initially, we wanted to use the keystrokes in a different manner, where the robot would move in a given direction only while the key was held down, and would stop when the key was released. However, the key-listener code we were given was restrictive here. Because the listener would just wait until a key was pressed, the execution of the node would be stuck within this function, preventing us from stopping the robot when no keys are pressed.

As a workaround, we allowed any keys other than `WASD` to stop the robot, rather than just a key-release.

### Drive Square

### Wall Following

### Person Following

### Obstacle Avoidance
Our obstacle avoidance code uses the Neato’s built-in LiDAR and a simple program to avoid obstacles. Our code is split into three main functions: processing the LiDAR scan data, making a decision as to how to move based on that data, and a function that turns the robot if the path ahead is blocked off. The ObstacleAvoidance node has a subscription to the Neato’s LiDAR scan node and a publisher to the Neato’s velocity control node.

We divided the LiDAR data into two groups: a slice of LiDAR data directly ahead of the robot, and periphery LiDAR data of the sides of the robot. These two groups were each broken into left and right, creating a total of four sets of LiDAR data. The function with the role of processing the data intakes a full 360 degree scan and discards unneeded data before it splits it into these four lists. Before feeding the four lists into the movement decision-making function, it checks if there is anything directly ahead of the robot (in which case the robot enters the function for turning the robot until the forward bearing is clear).
The Neato chooses a path forward by choosing an angular velocity about the center of the Neato and a linear forward velocity. The angular velocity is determined by the minimum values of the periphery scan. If either periphery scan list has an obstacle within 0.7 meters, the angular velocity given a non-zero value that is proportional to the distance from the obstacle. The closer an obstacle is, the higher the angular velocity is. The direction to turn is determined by which side a closer obstacle is detected on. The linear velocity is determined by how close an object is detected in the datasets for directly ahead of the robot. If there is no obstacle within 0.5 meters, the Neato drives forward at 0.2 meters per second. If There is an obstacle closer than that, the Neato adjusts its speed based on how close the obstacle is, such that it slows down the closer it gets to an obstacle.

If the Neato gets too close to an obstacle directly ahead or next to it, the previous angular and linear velocities are overridden and the Neato stops moving before entering the function that causes it to back up and turn. The turning function uses periphery data to decide which direction to turn in: it will turn away from the side with the closer obstacles.

These functions run in a constantly updating loop.


### Multi-Behaviour

## Takeaways:
