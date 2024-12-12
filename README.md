# Docker and controller for multiple turtlebot4 robots

this readme is under construction. 


Requirement 
- docker is installed
- Visual Studio Code with the following extensions: Remote Development, Dev Containers, Docker

note: the basic setup follow this [Youtube tutorial](https://www.youtube.com/watch?v=dihfA7Ol6Mw&list=PLunhqkrRNRhaqt0UfFxxC_oj7jscss2qe&index=6&ab_channel=ArticulatedRobotics)




Basic steps
- copy repo to local 
- open the folder in visual studio code
- Ctrl+P --> Reopen in Containers
- In this phase you should be able to see the topics of the robots

- direct the terminal into ros2_ws folder and build the workspace via `colcon build --symlink-install`
- source the install folder inside the ros2_ws
- then you should be able to run the basic controller via `ros2 run tb4_controller test_controller`

