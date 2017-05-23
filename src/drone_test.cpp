#include<stdio.h>
#include<stdlib.h>
#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<sensor_msgs/image_encodings.h>
#include<cv_bridge/cv_bridge.h>

#include<opencv2/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
//Something wrong with nonfree
//#include<opencv2/nonfree/features2d.hpp>
//#include<opencv2/nonfree/nonfree.hpp>

#include<ardrone_autonomy/CamSelect.h>

#include"ARDrone.h"

using namespace std;
using namespace cv;
static const char Window[] ="RGB Img";
static const char Window1[] ="Grey Img";
//The pointer to the message of topic
void process(const sensor_msgs::ImageConstPtr & cam_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr=cv_bridge::toCvCopy(cam_img,sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception & e)
    {
        ROS_ERROR("cv_bridge error: %s",e.what());
        return;
    }
    Mat img_rgb=cv_ptr->image;
    Mat img_gray;
    cvtColor(img_rgb,img_gray,CV_RGB2GRAY);
    imshow(Window,img_rgb);
    imshow(Window1,img_gray);
    cvWaitKey(0);
    vector<Vec3f> circles;
    HoughCircles(img_gray,circles,CV_HOUGH_GRADIENT,1,img_gray.rows/8,200,100,0,0);
    for(size_t i=0;i<circles.size();i++)
    {
        cout<<"Center x:"<<circles[i][0]<<"y:"<<circles[i][1]<<endl;
        cout<<"Radius"<<circles[i][2]<<endl;
    }

}

int main(int argc, char ** argv)
{

    ros::init(argc,argv,"drone_test");
    ARDrone* ardrone=new ARDrone();
    char key;
    ros::Rate loop_rate(50);
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub =it.subscribe("/ardrone/image_raw",20,process);
    cv::namedWindow(Window);
    cv::namedWindow(Window1);
    ros::spin();
    return 0;
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        key=cv::waitKey(0.5);
        if(key==27)
        {
            ardrone->land();
            cout<<"Esc"<<endl;
        }
    }
}
