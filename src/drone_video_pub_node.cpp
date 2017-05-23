//From ardrone node we get video, then we transform and construct
//a new node to publish it
//Zhang Peike
//Time:20170520-
#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<geometry_msgs/Twist.h>

#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui.hpp>

#include"ARDrone.h"
//#include"ardrone_autonomy/"

using namespace std;
using namespace cv;

static const string OPENCV_WINDOW="My Image";

image_transport::Publisher image_pub;
image_transport::Subscriber image_sub;
ros::Publisher cmd_vel_pub;

geometry_msgs::Twist twist_msg;
std_msgs::Empty empty_msg;

float MOVE_X=0.5;
float MOVE_Y=0.5;
float MOVE_YY=0.7;
float TURN_Z=0.1;
float TURN_ZZ=0.3;

void CallBack(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception e)
    {
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return;
    }
    Mat img_rgb,img_gray;
    img_rgb=cv_ptr->image;
    cvtColor(img_rgb,img_gray,CV_RGB2GRAY);
    //Display
    imshow(OPENCV_WINDOW,img_gray);
    waitKey(3);
    //Node publishing
    sensor_msgs::ImagePtr msg_pub;
    //Turn opencv image to ros image
    msg_pub=cv_bridge::CvImage(std_msgs::Header(),"mono8",img_gray).toImageMsg();
    //Publish
    image_pub.publish(msg_pub);
    vector<Vec3f> circles,RealCircles;
    HoughCircles(img_gray,circles,CV_HOUGH_GRADIENT,1,img_gray.rows/8,200,100,0,0);
    //
    cout<<"How many circles:"<<circles.size()<<endl;
    //Get the two side circles

    float Minx=circles[0][0],Maxx=circles[0][0],Miny=circles[0][1],Maxy=circles[0][1];
    int FLAG_X_MIN=0,FLAG_Y_MIN=0,FLAG_X_MAX=0,FLAG_Y_MAX=0;
    for(size_t i=0;i<circles.size();i++)
    {
        if(Minx>circles[i][0])
        {
            Minx=circles[i][0];
            FLAG_X_MIN=i;
        }
        if(Maxx<circles[i][0])
        {
            Maxx=circles[i][0];
            FLAG_X_MAX=i;
        }
        if(Miny>circles[i][1])
        {
            Miny=circles[i][1];
            FLAG_Y_MIN=i;
        }
        if(Maxy<circles[i][1])
        {
            Maxy=circles[i][1];
            FLAG_Y_MAX=i;
        }
        cout<<"Center x:"<<circles[i][0]<<"y:"<<circles[i][1]<<endl;
        cout<<"Radius"<<circles[i][2]<<endl;        
    }
    cout<<"Min x:"<<Minx<<"Min y:"<<Miny<<endl;
    cout<<"Max x:"<<Maxx<<"Max y:"<<Maxy<<endl;
    /*Fuzzy Control*/
    /*suppose we get the target circle.*/
    //Choose_Circles(circles,Real_Circle);
    size_t Img_Width=img_gray.cols;
    //control the drone
    //the position is near the center
    if((Minx+Maxx)>0.9*Img_Width&(Minx+Maxx)<1.1*Img_Width)
    {
        //the direction is parallel
        if((Maxx-Minx)/(Maxy-Miny)<0.1)
        {
            twist_msg.linear.x=0.5;
            twist_msg.linear.y=0.0;
            twist_msg.linear.z=0.0;
            twist_msg.angular.x=0.0;
            twist_msg.angular.y=0.0;
            twist_msg.angular.z=0.0;
            ROS_INFO("GO");
        }
        if((Maxx-Minx)/(Maxy-Miny)>=0.1&(Maxx-Minx)/(Maxy-Miny)<1)
        {
            twist_msg.linear.x=0.2;
            twist_msg.linear.y=0.0;
            twist_msg.linear.z=0.0;
            twist_msg.angular.x=0.0;
            twist_msg.angular.y=0.0;
            if(FLAG_X_MAX==FLAG_Y_MAX||FLAG_X_MIN==FLAG_Y_MIN)
            {

                twist_msg.angular.z=TURN_Z;
                ROS_INFO("TURN Left");
            }
            else
            {
                twist_msg.angular.z=-TURN_Z;
                ROS_INFO("TURN Right");
            }
        }
        if((Maxx-Minx)/(Maxy-Miny)>=1)
        {
            twist_msg.linear.x=0.1;
            twist_msg.linear.y=0.0;
            twist_msg.linear.z=0.0;
            twist_msg.angular.x=0.0;
            twist_msg.angular.y=0.0;
            if(FLAG_X_MAX==FLAG_Y_MAX||FLAG_X_MIN==FLAG_Y_MIN)
            {

                twist_msg.angular.z=TURN_ZZ;
                ROS_INFO("TURN LeftX2");
            }
            else
            {
                twist_msg.angular.z=-TURN_ZZ;
                ROS_INFO("TURN RightX2");
            }
        }
    }
    //the target is on the right
    if((Minx+Maxx)<=0.9*Img_Width)
    {
        if((Minx+Maxx)<=0.6*Img_Width)
        {
            twist_msg.linear.x=0.1;
            twist_msg.linear.y=MOVE_YY;
            twist_msg.linear.z=0.0;
            twist_msg.angular.x=0.0;
            twist_msg.angular.y=0.0;
            twist_msg.angular.z=0.0;
            ROS_INFO("MOVE LEFTX");
        }
        if((Minx+Maxx)>0.5*Img_Width)
        {
            twist_msg.linear.x=0.1;
            twist_msg.linear.y=MOVE_Y;
            twist_msg.linear.z=0.0;
            twist_msg.angular.x=0.0;
            twist_msg.angular.y=0.0;
            twist_msg.angular.z=0.0;
            ROS_INFO("MOVE LEFT");
        }
    }
    //the target is on the left
    if((Minx+Maxx)>=1.1*Img_Width)
    {
        if((Minx+Maxx)>=1.6*Img_Width)
        {
            twist_msg.linear.x=0.1;
            twist_msg.linear.y=-MOVE_YY;
            twist_msg.linear.z=0.0;
            twist_msg.angular.x=0.0;
            twist_msg.angular.y=0.0;
            twist_msg.angular.z=0.0;
            ROS_INFO("MOVE RIGHTX");
        }
        if((Minx+Maxx)<=1.6*Img_Width)
        {
            twist_msg.linear.x=0.1;
            twist_msg.linear.y=MOVE_Y;
            twist_msg.linear.z=0.0;
            twist_msg.angular.x=0.0;
            twist_msg.angular.y=0.0;
            twist_msg.angular.z=0.0;
            ROS_INFO("MOVE RIGHT");
        }
    }
    //cmd_vel_pub.publish(twist_msg);
}

int main(int argc, char ** argv)
{
    /**/
    ros::init(argc,argv,"drone_video_pub_node");
    ARDrone * ardrone = new ARDrone();
    ardrone->setup();
    ROS_INFO("Hello");

    ros::Rate loop_rate(10);
    ros::NodeHandle nh;
    ros::NodeHandle nh_cmd_vel;
    image_transport::ImageTransport it(nh);

    //Initialization
    image_sub=it.subscribe("/ardrone/image_raw",1,CallBack);
    image_pub=it.advertise("drone_video_pub_node",1);
    cmd_vel_pub=nh_cmd_vel.advertise<geometry_msgs::Twist>("/cmd_vel", 1);;

    namedWindow(OPENCV_WINDOW);
    ros::spin();
    destroyWindow(OPENCV_WINDOW);
    char key=cv::waitKey(0.5);
    while(ros::ok())
    {
        if(key==27)
        {
            ardrone->land();
            cout<<"Esc"<<endl;
        }

        loop_rate.sleep();
    }

    return 0;
}

