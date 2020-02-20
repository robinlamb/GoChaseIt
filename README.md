# Go Chase It

 https://www.youtube.com/watch?v=Fu-AxAy8Y5g&feature=youtu.be

This project is a simulation in Gazebo and RViz is which the robot follows a white colored object.  The robot model is made using URDF.  There are two C++ ROS nodes.  

One receieves values to drive the wheels of the robot.  The other node continuously checks the images from the robot's camera and determines if there is a white object present, by checking each pixel in the image to see if it is white.  If no white pixel is found, the robot will not move. 

If a white pixel is found, the node will determine if it is in the left, center, or right of the screen.  If it is to the left or the right, values to turn the robot in the appropriate direction will be sent to the drive_bot node.  If it is in the center of the screen, values will be sent to drive the robot forward.   
