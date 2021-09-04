#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;
ball_chaser::DriveToTarget srv;
float Kp_a = .5;
float Kd_a = .5;
float MAXVELOCITY = .3;
float error_old_a = 0;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    if (client.call(srv)){
        //log success
        ROS_INFO("Call Success!");
    }else{
        //log failure
        ROS_WARN("Call Failed!");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255; //255 would be pure white but camera seems to be giving lower values
    int height = img.height;
    int width = img.width;
    int step = img.step;
    bool ballFound = false;
    int pixelsFound = 0;
    float posX = 0;
    float posY = 0;
    int pixelCount = 0;
    //ROS_INFO("Data step size is [%i]",sizeof(img.step));
    for (int pixel = 0; pixel<width*step;pixel=pixel+3){
        if (img.data[pixel]>=white_pixel & img.data[pixel+1]>=white_pixel & img.data[pixel+2]>=white_pixel){
            ballFound = true;
            int X = (pixel/3)%width;
//            int Y = pixel/width;
            // Keep updating the ball position as we find pixels. Note that we are assuming every white pixel is part of the ball.
            posX = (posX*pixelsFound + X) / (pixelsFound+1);
//            posY = (posY*pixelsFound + Y) / (pixelsFound+1);
            pixelsFound = pixelsFound + 1;
        }
        //if (pixel%200000 == 0){
        //    ROS_INFO("Pixel [%i] has value [%i]",pixel,img.data[pixel]);
        //}
        pixelCount++;
    }
    // By default we stop (default is no ball found)
    float lin_x = 0;
    float ang_z = 0;
    float error_a = 0;
    if (ballFound){ //P controller
        ROS_INFO("Ball Found, X Position [%f] with [%i] ball pixels. Calculating Request",posX,pixelsFound);
        //Ball detected, we need to calculate a velocity and rotation so that we can follow the ball
        float ballPercent_inverse = 1-(float(pixelsFound)/float(width*height));
        lin_x = MAXVELOCITY*ballPercent_inverse*ballPercent_inverse; // We want to go faster if the ball is further away
        if (ballPercent_inverse<.993){
            lin_x=0;
            ROS_INFO("Close enough to stop! ballPercent_inverse is [%f]",ballPercent_inverse);
            } //give us a stop condition
        float center = width / 2;
        error_a = (center - posX)/width;
        ang_z = (Kp_a * error_a + Kd_a * (error_a-error_old_a))*ballPercent_inverse*ballPercent_inverse; //left turn is positive, posX < center. We turn sharper when off by more
    } else {
        ROS_INFO("No Ball found. Checked [%i] pixels!",pixelCount);
    }
    error_old_a = error_a;
    drive_robot(lin_x,ang_z);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/drive_bot/handle_drive_request");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 1, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}