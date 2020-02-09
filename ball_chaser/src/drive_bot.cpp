#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

//  Declare ROS publisher
ros::Publisher motor_command_publisher;


// Function to handle a request to drive the robot
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{
    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;

    // Set values for wheels with the requested values
    motor_command.linear.x = (float)req.linear_x;
    motor_command.angular.z = (float)req.angular_z;

    // Publish values to drive the robot
    motor_command_publisher.publish(motor_command);

    //Return a message that the values have been set
     res.msg_feedback = "Wheel velocities have been set.  Linear X: " + std::to_string(req.linear_x) + ", Angular Z: " + std::to_string(req.angular_z);
    
    return true;
}

int main(int argc, char** argv)
{
    // Initialize an ROS node
    ros::init(argc, argv, "drive_bot");

    // Create an ROS NodeHandle object
    ros::NodeHandle n;

    // Initialize publisher with type geometry_msgs::Twist and a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Service to move robot that calls handle_drive_request
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);


    // Handle ROS communication events
    ros::spin();

    return 0;
}
