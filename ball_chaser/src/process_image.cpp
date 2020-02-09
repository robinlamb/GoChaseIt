#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>


// Define a global client that can request services
ros::ServiceClient client;

//Variable to hold ball position
int current_ball_position;

//Constants to hold possible ball positions
const int NO_BALL = 0;
const int BALL_FORWARD = 1;
const int BALL_LEFT = 2;
const int BALL_RIGHT = 3; 


// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
     // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

     if (!client.call(srv))
        ROS_ERROR("Failed to call service DriveToTarget.");
}//End method


// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    //R, G, and B value of white pixel
    int white_pixel = 255; 

    // Initialize current_ball_position as NO_BALL
    current_ball_position = NO_BALL; 

    // Loops to search for white pixel in image and find its location on the screen

     //Outer loop to search number of rows
     for (int i = 0; i < img.height * img.step; i += img.step) {

        //Inner loop to search through each row
        for (int j = i; j < (i + img.step); j += 3){

            //Check rgb data of each pixel
            if ((img.data[j] == white_pixel) && (img.data[j+1] == white_pixel) && (img.data[j+2] == white_pixel)) {

               //If white pixel is found to the left of screen, set ball position as left
               if (j < (i + (img.step / 3.0))){
                       current_ball_position = BALL_LEFT;
                      }//End if

               //Set ball position as right if the pixel is to the right
               else if (j > (i + ((img.step / 3.0) * 2))){
                      current_ball_position = BALL_RIGHT;
                      }//End else if

               //Set position as forward if it is in the center of the screen
               else {
                       current_ball_position = BALL_FORWARD;
                     }//End else

      	      }//End if

          }//End for

       }//End for

    //Compares the ball's current position with possible positions and sends appropriate
    //drive command
    switch (current_ball_position) {
    
       case NO_BALL:
         drive_robot(0.0, 0.0);
         break;
       case BALL_FORWARD:
         drive_robot(0.5, 0.0);
         break;
       case BALL_LEFT:
         drive_robot(0.0, 0.5);
         break;
       case BALL_RIGHT:
         drive_robot(0.0, -0.5);
         break;

    }//End switch

}//End method

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}//End method
