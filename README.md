# ME495 Embedded Systems Homework 2
Author: ${Jialu Yu}

${The package is divided into 3 packages: 1. Arena node 2.Turtle node 3. Catcher node and will visually represent as a robot and a turtle on the map. The turtle's location will be the same as robot's location on each map. The brick will show in indicated location and drop, the robot will catch it if it prediction suggests the robot can catch it with the maximum velocity designed.}

## Quickstart
1. Use `ros2 launch turtle_brick turtle_arena.launch.xml` to start the arena and turtle simulation
2. Use `ros2 service call drop std_srvs/srv/Empty` to drop a brick
3. Here is a video of the turtle when the brick is within catching range
<video src=https://github.com/ME495-EmbeddedSystems/homework2-NuCapybara/assets/144244355/206a95ef-48d3-4c80-aaee-f2a646de54da>
</video>


4. Here is a video of the turtle when the brick cannot be caught
<video src=https://github.com/ME495-EmbeddedSystems/homework2-NuCapybara/assets/144244355/5c7e4c18-0e2b-40e8-9af5-dfcd2e2d01dc>
</video>