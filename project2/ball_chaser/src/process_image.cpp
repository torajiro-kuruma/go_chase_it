#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Service call: drive the robot move");
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
   
    if (!client.call(srv))
        ROS_ERROR("Failed to call service drive_bot!!");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    int idx = 0;
    int total_byte = img.step * img.height;
    while(idx < total_byte)
    {
	if(img.data[idx] == white_pixel)
        {
		int pos = (idx % img.step);
		if(pos <= (img.step / 3))
                {
                	ROS_INFO_STREAM("Pos: Left");
                        drive_robot(0.0, 0.1);
                }
		else if(pos <= (img.step * 2 / 3)) 
                {
                	ROS_INFO_STREAM("Pos: Forward");
      			drive_robot(0.1, 0);
		}
                else
                {
      			ROS_INFO_STREAM("Pos: Right");
			drive_robot(0.0, -0.1);
		}
		break;
        }
	idx++;
    }
    if(idx == total_byte) // white pix is not found
    {
	drive_robot(0.0, 0.0);
    }
}

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
}
