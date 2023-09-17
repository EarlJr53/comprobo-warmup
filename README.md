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

### Multi-Behaviour

## Takeaways: