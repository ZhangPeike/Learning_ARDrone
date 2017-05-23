#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/image_encodings.h>
#include <ardrone_autonomy/Navdata.h>
//ros package

#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

//opencv 

#include <iostream>
#include <math.h>
#include <string.h>
//C++ standard lib

using namespace cv;
using namespace std;
//////////////////////////////////////variable declaration//////////////////////////////////////
	///////////////////////////////image process variable//////////////////////////////////
bool is_send=true;//send
int thresh = 50, N = 11;
const char* wndname = "view";
double lastImg=0;
double thisImg=0;
int altitude=0;
double x_center_dev=0;
double y_center_dev=0;
bool find_origin=false;
int no_origin=0;
double angle_dev=0;
double square_area=0;
enum bound_direction{NONE,FRONT,BACK,LEFT,RIGHT};
int find_bound_num=0;
bound_direction bound_dir[2]={NONE,NONE};
bool find_red=false;
int no_red=0;
double red_x=0;
double red_y=0;
int red_area=0;
double theta_circle=0;
bool find_angle=false;
	///////////////////////////////image process variable end//////////////////////////////////



	//////////////////////////////motion control variable///////////////////////////////////////////
geometry_msgs::Twist twist_msg;
geometry_msgs::Twist twist_msg_hover;
std_msgs::Empty emp_msg;
const int thresh_angle=6;
const int thresh_pos=50;
const int thresh_red=30;
const int thresh_down=140;
const int thresh_backward=70;
bool can_release=false;
int altitude_drone=0;

vector<Point> points_plan_track;
enum DIRECTION{DIR_R,DIR_RF,DIR_F,DIR_LF,DIR_L,DIR_LB,DIR_B,DIR_RB};
const int length_irobot=80;
const int length_drone=80;
const double factor_track[8]={1,1.414,1,1.414,1,1.414,1,1.414};

int down_time=0;

bool lose_irobot=false;
	//////////////////////////////motion control variable end///////////////////////////////////////


	//////////////////////////////node process variable///////////////////////////////////////////

const int frequency=50;
enum work_state{WAIT_IROBOT,CATCH_IROBOT,RELEASE_IROBOT};
enum wait_state{ALTITUDE,ANGLE,ORIGIN,BOUNDARY};
	//////////////////////////////node process variable end///////////////////////////////////////

//////////////////////////////////////variable declaration end//////////////////////////////////////



/////////////////////////////////////function about track planning///////////////////////////
double sq(double a)
{
	return a*a;
}

double distance(Point2f p1,Point2f p2)
{
	double dist=sqrt(sq(p1.x-p2.x)+sq(p1.y-p2.y));
	return dist;
}

double distance(double x1,double y1,double x2,double y2)
{
	double dist=sqrt(sq(x1-x2)+sq(y1-y2));
	return dist;
}

int planTrack(double x,double y,double theta_angle,bool isMove)
{
	for(int i=0;i<8;i++)
	{
		double beta=CV_PI*45*i/180;
		Point p(50*factor_track[i]*cos(beta),50*factor_track[i]*sin(beta));
		points_plan_track.push_back(p);	
	}
	//calculate next point
	double x_new,y_new;
	Point2f point_next;
	if(isMove)
	{
		double theta=CV_PI*theta_angle/180;
		x_new=x+length_irobot*cos(theta);
		y_new=y+length_irobot*sin(theta);
		point_next.x=x_new;
		point_next.y=y_new;
	}
	else
	{
		x_new=x;
		y_new=y;
		point_next.x=x_new;
		point_next.y=y_new;
	}
	//calculate distance to dir points
	double dists[8];
	for(int i=0;i<8;i++)
	{
		dists[i]=distance(point_next,points_plan_track[i]);
	}
	//find the mininum
	double min=100000000;
	int k=0;
	for(int i=0;i<8;i++)
	{
		if(dists[i]<min)
		{
			min=dists[i];
			k=i;	
		}
	}
	return k;
}


int planDirection(double x1,double y1,double theta_angle,double &dist_toline,bool &is_front)
{
	//cal k,b
	double theta=CV_PI*(theta_angle+0.000001)/180;
	double k=tan(theta);
	double b=y1-k*x1;
	ROS_INFO("K,B IS %lf,%lf",k,b);
	dist_toline=fabs(b)/distance(k,1,0,0);
	//find dir point to line
	vector<Point> point_dir;
	vector<Point> point_toline;
	vector<int> point_id;
	for(int i=0;i<8;i++)
	{
		double beta=CV_PI*45*i/180;
		Point p(50*factor_track[i]*cos(beta),50*factor_track[i]*sin(beta));
		point_dir.push_back(p);
	}

	for(int i=0;i<8;i++)
	{
		int x=point_dir[i].x;
		int y=point_dir[i].y;
		if(k>=0)
		{
			if(b<=0)
			{	
				if((k*x-y)>0)
				{
					point_toline.push_back(point_dir[i]);
					point_id.push_back(i);	
					ROS_INFO("I IS %d,x,y is :(%d,%d)",i,x,y);
				}
			}
			else
			{
				if((k*x-y)<0)
				{
					point_toline.push_back(point_dir[i]);
					point_id.push_back(i);	
					ROS_INFO("I IS %d,x,y is :(%d,%d)",i,x,y);	
				}	
			}
		}
		else
		{
			if(b>=0)
			{
				if((k*x-y)<0)
				{
					point_toline.push_back(point_dir[i]);
					point_id.push_back(i);	
					ROS_INFO("I IS %d,x,y is :(%d,%d)",i,x,y);	
				}
			}
			else
			{
				if((k*x-y)>0)
				{
					point_toline.push_back(point_dir[i]);	
					point_id.push_back(i);	
					ROS_INFO("I IS %d,x,y is :(%d,%d)",i,x,y);
				}
			}
		}
	}//find point end
	/*if(point_id.size()>0)
	{
		for(int i=0;i<point_id.size();i++)
			ROS_INFO("%d-------------",point_id[i]);
	}
	else
	{
		ROS_INFO("no point");
	}*/
	//90 degree line
	double k0=tan(CV_PI*(theta_angle+90.000001)/180);
	double b0=y1-k0*x1;
	ROS_INFO("K0,B0 IS %lf,%lf",k0,b0);
	if(b0>=0)//in front of irobot
	{
		ROS_INFO("in front of irobot");
		is_front=true;
		double max_cos=-2;
		int k_max=point_id[0],k_second=point_id[0];
		double x_irobot=cos(theta);
		double y_irobot=sin(theta);
		for(int i=0;i<point_toline.size();i++)
		{
			int alpha=45*point_id[i];
			double x_p=cos(CV_PI*alpha/180);
			double y_p=sin(CV_PI*alpha/180);
			double dot_product=x_p*x_irobot+y_p*y_irobot;
			ROS_INFO("DOT PRODUCT with %d IS %lf",point_id[i],dot_product);
			//waitKey();
			if(dot_product>max_cos)
			{
				max_cos=dot_product;
				k_max=point_id[i];
				ROS_INFO("current point is %d,max is %lf",k_max,max_cos);
			}
		}
		
		if(max_cos<0.866)
		{
			ROS_INFO("max");
			return k_max;
		}
		else
		{
			max_cos=-2;
			for(int i=0;i<point_toline.size();i++)
			{
				if(point_id[i]==k_max)
					continue;
				int alpha=45*point_id[i];
				double x_p=cos(CV_PI*alpha/180);
				double y_p=sin(CV_PI*alpha/180);
				double dot_product=x_p*x_irobot+y_p*y_irobot;
				if(dot_product>max_cos)
				{
					max_cos=dot_product;
					k_second=point_id[i];
					ROS_INFO("current point is %d,max is %lf",k_max,max_cos);
				}
				ROS_INFO("DOT PRODUCT with %d IS %lf",point_id[i],dot_product);
			}
			ROS_INFO("second");
			return k_second;
		}

	}
	else//behind irobot
	{
		ROS_INFO("behind irobot");
		is_front=false;
		double max_cos=-10;
		int k_max=point_id[0];
		double x_irobot=cos(theta);
		double y_irobot=sin(theta);
		for(int i=0;i<point_toline.size();i++)
		{
			//
			/*static int a=0;
			a++;
			if(a>20)
				waitKey();*/
			

			int alpha=45*point_id[i];
			double x_p=cos(CV_PI*alpha/180);
			double y_p=sin(CV_PI*alpha/180);
			double dot_product=x_p*x_irobot+y_p*y_irobot;
			ROS_INFO("DOT PRODUCT with %d IS %lf",point_id[i],dot_product);
			if(dot_product>max_cos)
			{
				max_cos=dot_product;
				k_max=point_id[i];
				ROS_INFO("current point is %d,max is %lf",k_max,max_cos);
			}
		}
		return k_max;	
	}
}
/////////////////////////////////////////track planning end/////////////////////////////////

/////////////////////////////////////////function:angle////////////////////////////////////////////////
static double angle( Point pt1, Point pt2, Point pt0 )    
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    double ratio=(dx1*dx1+dy1*dy1)/(dx2*dx2+dy2*dy2);
    if (ratio<0.8 || 1.25<ratio)//根据边长平方的比过小或过大提前淘汰这个四边形，如果淘汰过多，调整此比例数字
    return 1.0;//根据边长平方的比过小或过大提前淘汰这个四边形
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}
/////////////////////////////////////////function:angle end////////////////////////////////////////////////

/////////////////////////////////////////function:find circle///////////////////////////////////////////////'
void findCircles(Mat& src)
{
	find_angle=false;
	vector<Mat> bgr;
	split(src, bgr);
	cv::Mat green=bgr.at(1);
	//use green channels to find white circle
	Mat green_thresh;
	static int threshold_green=180;
	createTrackbar("green","contour view", &threshold_green,255);
	threshold(green, green_thresh,threshold_green, 255, THRESH_BINARY );
	Mat temp_green;
	green_thresh.copyTo(temp_green);
	vector<vector<Point> > contours_g;
	findContours(temp_green, contours_g,CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	src.copyTo(temp_green);
	for(int i=0;i<contours_g.size();i++)
	{
		drawContours(temp_green,contours_g,i, Scalar(255,255,0),3,8);
		cv::Rect rect= boundingRect( Mat(contours_g[i]) );
		int rect_area=rect.width*rect.height;
		float r;
		Point2f c;
		minEnclosingCircle(contours_g[i],c,r);
		double area_enclose_circle=CV_PI*r*r;
		double area_contour=contourArea(contours_g[i]);
		double ratio_area=area_contour/area_enclose_circle;
		//if(rect_area>4000&&rect_area<20000&&ratio_area>0.3)
		if(rect_area>4000&&rect_area<20000&&ratio_area>0.3)
		{
			circle(temp_green,c,(int)r,Scalar(0,255,255), 5, 8, 0 );
			drawContours(temp_green,contours_g,i, Scalar(0,255,100),6,8);
			//ROS_INFO("GREEN AREA:%d",rect_area);
			//ROS_INFO("WHITE CIRCLE:(%lf,%lf)",x,y);
			double x=rect.x+rect.width/2-src.cols/2;
			double y=-(rect.y+rect.height/2-src.rows/2);
			theta_circle=180*atan(y/x)/CV_PI;
			if(x<0)
			{
				theta_circle+=180;
			}
			if(x>0&&y<0)
			{
				theta_circle+=360;
			}
			find_angle=true;
			ROS_INFO("THETA IS:%lf",theta_circle);
			if(!is_send)
			{
				if(theta_circle>0&&theta_circle<45)
				{
					can_release=true;
					//ROS_INFO("RELEASE!!!!!!!!!!!!!!!!!!!");
				}
			}
			else
			{
				if(theta_circle>40&&theta_circle<110)
				{
					can_release=true;
					//ROS_INFO("RELEASE!!!!!!!!!!!!!!!!!!!");
				}
			}
				
		}
		break;
	}
	imshow("view",temp_green);
	imshow("find_red_window",green_thresh);
	
}

/////////////////////////////////////////function:find circle end///////////////////////////////////////////////'

///////////////////////////////////////function:find red area ////////////////////////////////////////////
void findRedArea( Mat& src,Mat& image_draw )
{
	find_red=false;	
	red_area=0;
	cv::Mat srcImage;
	src.copyTo(srcImage);

	vector<Mat> channels;
	split(srcImage,channels);

	//提取用于找红色的红色区域
	Mat src_gray,src_gray1,src_gray2;
	addWeighted( channels.at(2), 1.5,channels.at(1) , -1.5, 0.0,src_gray1 );
	addWeighted( src_gray1, 1,channels.at(0) , -1, 0.0,src_gray );
	

	//二值化，降噪
	Mat threshold_output,src_erode1,src_erode,src_gray4;
	static int thresh_red_img=90;
	cv::createTrackbar("red thresh","contour view", &thresh_red_img,255);
	threshold(src_gray, threshold_output,thresh_red_img, 255, THRESH_BINARY );

	cv::Mat element = getStructuringElement( 0,Size(3,3));
	Mat src_dilate;
	dilate(threshold_output,src_dilate, element,Point(-1,-1),1);
	//find contours
	src_dilate.copyTo(src_gray1);
	vector<vector<Point> > contours;
	findContours(  src_gray1, contours,CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	for(int i=0;i<contours.size();i++)
		drawContours(image_draw,contours,i, Scalar(255,0,0),3,8);
	//找出面积最大的并且符合条件的轮廓
     	if(contours.size()!=0)
	{
		int max_area=0;
		int k=0;
		for(int i=0;i<contours.size();i++)
		{
			/*float r;
			Point2f c;
			minEnclosingCircle(contours[i],c,r);
			double area_enclose_circle=CV_PI*r*r;
			double area_contour=contourArea(contours[i]);
			double ratio_area=area_contour/area_enclose_circle;
			if(ratio_area<0.5)
			{
				continue;
			}*/
			double area_contour=contourArea(contours[i]);
			if(area_contour>max_area)
			{
				max_area=area_contour;
				k=i;
			}
		}
		cv::Rect rect= boundingRect( Mat(contours[k]) );
		int area_bounding_rect=rect.width*rect.height;
		
		if(area_bounding_rect>1600)//&&ratio<4
		{
			float radius;
			Point2f center;
			minEnclosingCircle( contours[k], center, radius);
		    	circle( image_draw, center,(int)radius,Scalar(0,0,255), 5, 8, 0 );
			find_red=true;
			red_x=center.x-src.cols/2;
			red_y=-(center.y-src.rows/2)-20;	
			red_area=area_bounding_rect;
			//ROS_INFO("(%lf,%lf)",red_x,red_y);
			//ROS_INFO("RED AREA:%d",red_area);
			//if the whole red area can be see,extract the red area and do calculation about the black  and white circle 
			int ratio=MAX(rect.width,rect.height)/MIN(rect.width,rect.height);
			if(ratio<1.04)
			{
				Mat src_roi,src_roi_resize;
				src(rect).copyTo(src_roi);
				resize(src_roi,src_roi_resize, cv::Size(400,400));
				findCircles(src_roi_resize);
			}
			//calculation end
		}
		
	}
	circle(image_draw,Point(image_draw.cols/2,image_draw.rows/2-20),10, Scalar(0,0,255), 5, 8, 0);
	imshow("contour view",image_draw);
	
	//cv::waitKey();
}
/////////////////////////////////////////function:find red area end ////////////////////////////////////////////

//////////////////////////////////////function:find square//////////////////////////////////////////////////
static void findSquares(Mat& image, vector<vector<Point> >& squares )
{
///////////////////////////////////////find sqaure contour///////////////////////////////////////////
    squares.clear();

    Mat pyr, timg, gray0(image.size(), CV_8U), gray;

    // down-scale and upscale the image to filter out the noise
    pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    pyrUp(pyr, timg, image.size());
    vector<vector<Point> > contours;

    // find squares in every color plane of the image
    for( int c = 0; c < 3; c++ )
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        for( int l = 0; l < N; l++ )
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                Canny(gray0, gray, 0, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                dilate(gray, gray, Mat(), Point(-1,-1));
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
            }

            // find contours and store them all as a list
            findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

            vector<Point> approx;

            // test each contour
            for( size_t i = 0; i < contours.size(); i++ )
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( approx.size() == 4 &&
                    fabs(contourArea(Mat(approx))) > 2000 &&fabs(contourArea(Mat(approx)))<115200*2/3&&   
                    isContourConvex(Mat(approx)) )
                {
		        double cosine[4]={0,0,0,0};
              		for( int j = 0; j <=3; j++ )
			{
				// find cosine of the angle between joint edges
				cosine[j] = fabs(angle(approx[j], approx[(j+2)%4], approx[(j+1)%4]));
				//maxCosine = MAX(maxCosine, cosine);
			}

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                   if( cosine[0] < 0.06&&cosine[1] < 0.06&&cosine[2] < 0.06&&cosine[3] < 0.06 ) 
		    {  
		        RotatedRect box = minAreaRect(Mat(approx));
			if(fabs(box.angle)>1.0)
		   		squares.push_back(approx);	
		    }			      
                }
            }
        }
    }
///////////////////////////////////////find sqaure contour end ///////////////////////////////////////////

///////////////////////////////////////draw sqaure and calculate angle//////////////////////////////
	int num_square=0;
	angle_dev=0;
	square_area=0;
	for( size_t i = 0; i < squares.size(); i++ )
    	{
		RotatedRect box = minAreaRect(Mat(squares[i]));
		int sameNum=0;
		for(int j=0;j<i;j++)
		{
			RotatedRect box1 = minAreaRect(Mat(squares[j]));
			float angleDifference=fabs(box.angle-box1.angle);
			float centerDifference=fabs(box.center.x-box1.center.x)+fabs(box.center.y-box1.center.y);
			if(centerDifference<100)
			{
				sameNum++;
			}	
			if(sameNum!=0)
				break;
		}
		if(sameNum==0)
		{
			num_square++;
			Point2f vtx[4];
			box.points(vtx);
			line(image,vtx[0],vtx[1], Scalar(0,255,0),3,8,0);
			line(image,vtx[1],vtx[2], Scalar(0,255,0),3,8,0);
			line(image,vtx[2],vtx[3], Scalar(0,255,0),3,8,0);
			line(image,vtx[3],vtx[0], Scalar(0,255,0),3,8,0);
			int angleError=box.angle;
			if(angleError<-45)
			{
			  	angleError+=90;	
			}
			angle_dev+=angleError;
			square_area+=box.size.width*box.size.height;
			//ROS_INFO("the %d square's angle is %lf",num_square,angle_dev);
		}
    	}   
	if(square_area!=0)
	{
		angle_dev=angle_dev/num_square;
		square_area=square_area/num_square;
		//ROS_INFO("angle is %lf",angle_dev);
		ROS_INFO("square area is %lf",square_area);
	}
///////////////////////////////////////draw sqaure and calculate angle end//////////////////////////////
}
////////////////////////////////////// function:find square end //////////////////////////////////////////////////

////////////////////////////function:find origin , square and boundary///////////////////////////
static void findOrigin( Mat &image )
{
	//////////////////////////////////////find origin//////////////////////////////////////////////
	find_origin=false;
	//split image to rgb channel
	vector<Mat> bgrImg;
	split(image, bgrImg);
	cv::Mat blueImg=bgrImg.at(0);
	cv::Mat greenImg=bgrImg.at(1);
	cv::Mat redImg=bgrImg.at(2);	
	// binarization of image
	cv::Mat binImg;
	static int binThresh=0;
	cv::createTrackbar("binarization thresh",wndname, &binThresh,255);
	cv::threshold(redImg,binImg,binThresh,255,THRESH_BINARY);
	//dilate
	cv::Mat element = getStructuringElement( 0,Size(3,3));
	cv::Mat image_erode;
	static int dilateTime=2;
	createTrackbar("dilate time","contour view", &dilateTime,50);
	dilate(binImg,image_erode, element,Point(-1,-1),dilateTime);
	//find contour and get the origin  blue circle
	cv::Mat temp;
	image_erode.copyTo(temp);
	vector<vector<Point> > contours;
	findContours(temp,contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	//get the origin blue circle
	vector<vector<Point> > origin;
	for(int i=0;i<contours.size();i++)
	{
		//judge is contour's area and shape,eliminate the small one
		static int areaDown=500;
		//createTrackbar("area down","contour view", &areaDown,3000);
		static int areaUp=20000;
		//createTrackbar("area up","contour view", &areaUp,100000);
		double area_contour=contourArea(contours[i],false);
		double radius_area=sqrt(area_contour/CV_PI);
		double length_contour=arcLength(contours[i],true);
		double radius_length=length_contour/(2*CV_PI);
		double diff_radius=fabs(radius_area-radius_length);
		double thresh_radius=20;
		
		if(area_contour>areaDown&&area_contour<areaUp&&diff_radius<thresh_radius)//&&diff_radius<ratioThresh
			origin.push_back(contours[i]);
			
		//ROS_INFO("THE %d contour's area is %lf",i+1,area_contour);
	}
	const int shift_origin=100;
	int center_x,center_y;
	if(origin.size()>=1)
	{
		find_origin=true;
		for(int i=0;i<origin.size();i++)
			drawContours(image, origin,0, Scalar(106,106,255),6,8);
		//calculate the center of blue circle and deviation
		cv::Rect bRect=boundingRect(origin[0]);
		center_x=bRect.x+bRect.width/2;
		center_y=bRect.y+bRect.height/2;
		x_center_dev=center_x-image.cols/2;
		y_center_dev=-(center_y-image.rows/2)+shift_origin;
	}
	//draw the origin blue circle contour
	circle(image,Point(image.cols/2,image.rows/2+shift_origin),10, Scalar(106,106,255), 5, 8, 0);
	
////////////////////////////////////// find origin end //////////////////////////////////////////////




///////////////////////////////////// find suqare and angle error/////////////////////////////////////////////////////////
	vector<vector<Point> > squares;
	findSquares(image, squares);
	
///////////////////////////////////// find square and angle error end/////////////////////////////////////////////////////////




///////////////////////////////////////find boundary///////////////////////////////////////////////////
	//find boundary using hough line dectation
	find_bound_num=0;
	for(int i=0;i<2;i++)
	{
		bound_dir[i]=NONE;
	}
	cv::Mat image_bin_inv;
	threshold(image_erode,image_bin_inv,100,255,THRESH_BINARY_INV);
	vector<Vec4i> lines;
        HoughLinesP(image_bin_inv, lines, 2, CV_PI/90, 80, 200, 10 );
	//use mid point and line direction vector to judge similar line
	vector<Vec4i> boundaries;
        for( size_t i = 0; i < lines.size(); i++ )
        {
		//mid point difference
		int mid_point_x=(lines[i][0]+lines[i][2])/2;
		int mid_point_y=(lines[i][1]+lines[i][3])/2;
		//line direction vector difference
		Vec2i dir_vec;
		dir_vec[0]=lines[i][2]-lines[i][0];
		dir_vec[1]=-(lines[i][3]-lines[i][1]);
		if(dir_vec[0]<0)
			dir_vec=-dir_vec;
		double tan_line=dir_vec[1]/(dir_vec[0]+1e-10);
		//similar line number
		int same_number=0;
		for(int j=0;j<i;j++)
		{	
			//mid point difference
			int mid_point_x_p=(lines[j][0]+lines[j][2])/2;
			int mid_point_y_p=(lines[j][1]+lines[j][3])/2;
			int diff_x=fabs(mid_point_x-mid_point_x_p);
			int diff_y=fabs(mid_point_y-mid_point_y_p);
			int diff_mid=diff_x+diff_y;
			//line direction vector difference
			Vec2i dir_vec_p;
			dir_vec_p[0]=lines[i][2]-lines[i][0];
			dir_vec_p[1]=-(lines[i][3]-lines[i][1]);
			if(dir_vec_p[0]<0)
				dir_vec_p=-dir_vec_p;
			double tan_line_p=dir_vec_p[1]/(1e-10+dir_vec_p[0]);
			double diff_dir=fabs(tan_line-tan_line_p);
			if(diff_mid<100||diff_dir<1)
				same_number++;
			if(same_number!=0)
				break;
		}
		if(same_number==0)
		{
			find_bound_num++;
			boundaries.push_back(lines[i]);
			line( image, Point(lines[i][0], lines[i][1]),Point(lines[i][2], lines[i][3]), Scalar(255,255,0), 3, 8 );
			if(fabs(tan_line)>1.5)
			{
				/*if(mid_point_x>image.cols/2)
					bound_dir[find_bound_num-1]=RIGHT;
				if(mid_point_x<image.cols/2)
					bound_dir[find_bound_num-1]=LEFT;
				ROS_INFO("OLD %d",bound_dir[find_bound_num-1]);*/
				double bound_left=MIN(lines[i][0],lines[i][2]);
				double bound_right=MAX(lines[i][0],lines[i][2]);
				Rect rect_left(0,0,bound_left,image.rows);
				Rect rect_right(bound_right,0,image.cols-bound_right-1,image.rows);
				Mat image_l,image_r;
				redImg(rect_left).copyTo(image_l);
				redImg(rect_right).copyTo(image_r);
				Scalar s_l=mean(image_l);
				Scalar s_r=mean(image_r);
				if(s_l[0]>s_r[0])
					bound_dir[find_bound_num-1]=RIGHT;
				if(s_l[0]<s_r[0])
					bound_dir[find_bound_num-1]=LEFT;
				ROS_INFO("NEW %d",bound_dir[find_bound_num-1]);
				ROS_INFO("sl and sr is %lf %lf",s_l[0],s_r[0]);
			}
			if(fabs(tan_line)<0.6777)
			{
				if(mid_point_y>image.rows/2)
					bound_dir[find_bound_num-1]=BACK;
				if(mid_point_y<image.rows/2)
					bound_dir[find_bound_num-1]=FRONT;
				
			}
			//ROS_INFO("tan:%lf",tan_line);
			//ROS_INFO("boundary is %d",bound_dir[find_bound_num-1]);
			if(find_bound_num>=2)
				break;
		}
        }
///////////////////////////////////////find boundary end ///////////////////////////////////


///////////////////////////////////////origin must be inside boundary/////////////////////////////////
	//judge if the origin found is inside the boundary,if not,it means that incorrect origin is found and drone should not move toward it!!!!
	if(find_origin&&find_bound_num!=0)
	{
		bool is_correct=true;
		for(int i=0;i<find_bound_num;i++)
		{
			int mid_point_x=(boundaries[i][0]+boundaries[i][2])/2;
			int mid_point_y=(boundaries[i][1]+boundaries[i][3])/2;
			//ROS_INFO("MID POINT IS:(%d,%d)",mid_point_x,mid_point_y);
			//ROS_INFO("CENTER IS (%d,%d)",center_x,center_y);
			switch(bound_dir[i])
			{
				case FRONT:
				{
					if(center_y<mid_point_y)
					{
						is_correct=false;
						ROS_INFO("OUTSIDE FRONT");
					}
					break;
				}

				case BACK:
				{
					if(center_y>mid_point_y)
					{
						is_correct=false;
						ROS_INFO("OUTSIDE BACK");
					}
					break;
				}

				case LEFT:
				{
					if(center_x<mid_point_x)
					{
						is_correct=false;
						ROS_INFO("OUTSIDE LEFT");
					}
					break;
				}

				case RIGHT:
				{
					if(center_x>mid_point_x)
					{
						is_correct=false;
						ROS_INFO("OUTSIDE RIGHT");
					}
					break;
				}

			}
			if(!is_correct)
			{
				ROS_INFO(" WRONG WRONG WRONG WRONG ORIGIN!!!!!!!!!!!!!!!!!!!");
				find_origin=false;
				x_center_dev=0;
				y_center_dev=0;
			}
		}//for i end
	}//judgement end
///////////////////////////////////////origin must be inside boundary end /////////////////////////////////


	/*vector<vector<Point > > boundary;
	for(int i=0;i<contours.size();i++)
	{
		RotatedRect bound_rect = minAreaRect(Mat(contours[i]));
		double max_edge=0;
		double min_edge=0;
		if(bound_rect.size.width>bound_rect.size.height)
		{
			max_edge=bound_rect.size.width;
			min_edge=bound_rect.size.height;
		}
		else
		{
			min_edge=bound_rect.size.width;
			max_edge=bound_rect.size.height;
		}
		double ratio=max_edge/min_edge;
		if(ratio>5)
		{
			boundary.push_back(contours[i]);
		}
	}
	for(int i=0;i<boundary.size();i++)
	{
		RotatedRect bound_rect = minAreaRect(Mat(boundary[i]));
		Point2f vtx[4];
		bound_rect.points(vtx);
		line(image,vtx[0],vtx[1], Scalar(255,255,0),3,8,0);
		line(image,vtx[1],vtx[2], Scalar(255,255,0),3,8,0);
		line(image,vtx[2],vtx[3], Scalar(255,255,0),3,8,0);
		line(image,vtx[3],vtx[0], Scalar(255,255,0),3,8,0);
	}*/
	
	//cv::imshow("find_red_window",image_bin_inv);
	cv::imshow("contour view",image);
	//imshow("view",image_erode);
	//cv::waitKey();
}
////////////////////////////function:find origin , square and boundary end ///////////////////////////

////////////////////////////function:callback of image//////////////////////////////////
void imageCallback(const sensor_msgs::Image::ConstPtr msg)
{	 
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg,
        	sensor_msgs::image_encodings::BGR8);

	//cv::imshow(wndname,cv_ptr->image);
	cv::Mat image = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8 )->image;
	Mat image_p;
	image.copyTo(image_p);
	findOrigin(image);
	//if(square_area<30000&&square_area!=0)
    findRedArea(image_p,image);
	//ROS_INFO("%lf",(double)ros::Time::now().toSec());	
	
}
////////////////////////////function:callback of image end//////////////////////////////////

////////////////////////////function:callback of navadata/////////////////////////////////////////

void callback_navadata(const ardrone_autonomy::Navdata& state_msg_in)
{
	altitude_drone=state_msg_in.altd;
	ROS_INFO("ALTITUDE:%d",altitude_drone);
}

////////////////////////////function:callback of navadata end/////////////////////////////////////////

/////////////////////////////function:main////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
////////////////////////////////////setup///////////////////////////////////////////////////////////
    int mode=0;

    ros::init(argc, argv, "image_listener");
    ros::NodeHandle node;
    ros::Rate loop_rate(frequency);

    ros::Publisher pub_twist;
    ros::Publisher pub_empty_takeoff;
    ros::Publisher pub_empty_land;
    ros::Subscriber sub_navadata;

    cv::namedWindow(wndname);
    cv::namedWindow("find_red_window");
    cv::namedWindow("contour view");
    //cv::namedWindow("view4");
    cv::startWindowThread();

    image_transport::ImageTransport it1(node);
    image_transport::ImageTransport it2(node);


    image_transport::Subscriber sub1 = it1.subscribe("camera/image", 1, imageCallback);

    image_transport::Subscriber sub2 = it2.subscribe("ardrone/image_raw", 1, imageCallback);

    pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1);
    sub_navadata=node.subscribe("ardrone/navdata",1,callback_navadata);

    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;

    twist_msg_hover.linear.x=0.0;
    twist_msg_hover.linear.y=0.0;
    twist_msg_hover.linear.z=0.0;
    twist_msg_hover.angular.x=0.0;
    twist_msg_hover.angular.y=0.0;
    twist_msg_hover.angular.z=0.0;

    work_state work_s=WAIT_IROBOT;
    wait_state wait_proc=BOUNDARY;

//==================
	if(mode==0)
	{
		for(int i=0;i<5;i++)
		{
			ros::spinOnce();
			system("rosservice call /ardrone/setcamchannel 1");
			
		}
		ROS_INFO("WAITKEY %d",int(!is_send));
		waitKey();
		for(int i=0;i<20;i++)
		{
			pub_empty_takeoff.publish(emp_msg);
			pub_twist.publish(twist_msg_hover);
			ROS_INFO("takeoff");
			loop_rate.sleep();
			ros::spinOnce();
			system("rosservice call /ardrone/setcamchannel 1");
		}
		for(int i=0;i<3;i++)
		{
			int upTime=30;
			twist_msg.linear.z =1;
			for(int j=0;j<upTime;j++)
			{	
				pub_twist.publish(twist_msg);
				loop_rate.sleep();
				ROS_INFO("setup setup up up ");
			}	
			twist_msg.linear.z =0;
		}
		for(int j=0;j<10;j++)
		{				
			pub_twist.publish(twist_msg_hover);
			loop_rate.sleep();
			//ROS_INFO("hover");
		}
	}
////////////////////////////////////setup end///////////////////////////////////////////////////////////



////////////////////////////////////main loop/////////////////////////////////////////////////////////////
	while(ros::ok())
	{
		//////////////////////////wait irobot////////////////////////////////
		if(work_s==WAIT_IROBOT)
		{
			//in the state of WAIT_IROBOT,drone will fly on higher altitude and near the origin,waiting for irobot to pass the front boundary	

			//BOUNDARY->ALTITUDE->ANGLE->ORIGIN->FIND_IROBOT->BOUNDARY
			
			//process BOUNDARY,position control using boundary
			if(wait_proc==BOUNDARY)
			{
				ROS_INFO("IN BOUNDARY");
				if(find_bound_num!=0&&(!find_origin))
				{
					//judge direction
					for(int i=0;i<1;i++)
					{
						switch (bound_dir[i])
						{
							case FRONT:
							{
								twist_msg.linear.x=-1;
								break;		
							}
							case BACK:
							{
								twist_msg.linear.x=1;
								break;		
							}
							case LEFT:
							{
								twist_msg.linear.y=-1;
								break;		
							}
							case RIGHT:
							{
								twist_msg.linear.y=1;
								break;		
							}
						}
						//test_boundary
						int time_fly=25;
						int time_hover=25;
						for(int j=0;j<time_fly;j++)
						{	
							pub_twist.publish(twist_msg);
							loop_rate.sleep();
						}
						ROS_INFO("boundary %d %d %d %d %d ",bound_dir[i],bound_dir[i],bound_dir[i],bound_dir[i],bound_dir[i]);
						for(int j=0;j<time_hover;j++)
						{				
							pub_twist.publish(twist_msg_hover);
							loop_rate.sleep();
							//ROS_INFO("hover");
						}
						twist_msg.linear.x=0;
						twist_msg.linear.y=0;
					}//for(int i=0;i<1;i++)END
				}//if(find_bound_num!=0&&(!find_origin)) END 
				else
				{
					wait_proc=ALTITUDE;
					ROS_INFO("BOUNDARY OK");
				}
			}
			//process BOUNDARY end

			//process ALTITUDE,use suqare area to control altitude
			if(wait_proc==ALTITUDE)
			{
				ROS_INFO("IN ALTITUDE");
				if(square_area==0||square_area>13000)
				{
					int time_up=8;
					int time_hover=1;
					twist_msg.linear.z =1;
					for(int j=0;j<time_up;j++)
					{	
						pub_twist.publish(twist_msg);
						loop_rate.sleep();
					}	
					ROS_INFO("up up up up ");
					for(int j=0;j<time_hover;j++)
					{				
						pub_twist.publish(twist_msg_hover);
						loop_rate.sleep();
						//ROS_INFO("hover");
					}
					twist_msg.linear.z =0;
				}
				if(square_area<9500&&square_area!=0)
				{
					int time_down=8;
					int time_hover=1;
					twist_msg.linear.z =-1;
					for(int j=0;j<time_down;j++)
					{	
						pub_twist.publish(twist_msg);
						loop_rate.sleep();
					}	
					ROS_INFO("down down down down ");
					for(int j=0;j<time_hover;j++)
					{				
						pub_twist.publish(twist_msg_hover);
						loop_rate.sleep();
						//ROS_INFO("hover");
					}
					twist_msg.linear.z =0;
				}
				if(square_area<=12000&&square_area>=8500)
				{
					ROS_INFO("ALTITUDE OK,AREA IS:%lf,altitude is :%d",square_area,altitude_drone);
				}
				wait_proc=ANGLE;
			}
			//process ALTITUDE end

			//process ANGLE,angle control using angle of square
			if(wait_proc==ANGLE)
			{
				ROS_INFO("IN ANGLE");
				if(fabs(angle_dev)>thresh_angle)
				{
					int turnTime=3;
					int time_hover=3;
					twist_msg.angular.z =-angle_dev/fabs(angle_dev);
					for(int j=0;j<turnTime;j++)
					{	
						pub_twist.publish(twist_msg);
						loop_rate.sleep();
					}
					ROS_INFO("angle angle angle angle ====");
					for(int j=0;j<time_hover;j++)
					{				
						pub_twist.publish(twist_msg_hover);
						loop_rate.sleep();
						//ROS_INFO("hover");
					}
					twist_msg.angular.z=0;
				}
				if(fabs(angle_dev)<=thresh_angle)
				{
					//ROS_INFO("ANGLE OK");
					wait_proc=ORIGIN;
				}
			}
			//process ANGLE end

			//process ORIGIN,position control by origin
			if(wait_proc==ORIGIN)
			{
				ROS_INFO("IN ORIGIN");
				if(distance(x_center_dev,y_center_dev,0,0)>thresh_pos)
				{
					int direc=planTrack(x_center_dev,y_center_dev,0,false);
					switch(direc)
					{
						case DIR_R:
						{
							twist_msg.linear.y=-1;	
							break;
						}
						case DIR_RF:
						{
							twist_msg.linear.y=-1;	
							twist_msg.linear.x=1;
							break;
						}	
						case DIR_F:
						{
							twist_msg.linear.x=1;
							break;
						}	
						case DIR_LF:
						{
							twist_msg.linear.y=1;	
							twist_msg.linear.x=1;
							break;
						}	
						case DIR_L:
						{
							twist_msg.linear.y=1;	
							break;
						}	
						case DIR_LB:
						{
							twist_msg.linear.y=1;
							twist_msg.linear.x=-1;
							break;
						}	
						case DIR_B:
						{
							twist_msg.linear.x=-1;
							break;
						}	
						case DIR_RB:
						{
							twist_msg.linear.x=-1;
							twist_msg.linear.y=-1;
							break;
						}				
					}

					int k_p=15;
					int time_fly=k_p*distance(x_center_dev,y_center_dev,0,0)/350;
					if(time_fly<5)
						time_fly=5;
					int time_hover=4;
					for(int j=0;j<time_fly;j++)
					{	
						pub_twist.publish(twist_msg);
						//ROS_INFO("fly fly fly");
						loop_rate.sleep();
					}
					//ROS_INFO("dir:%d",direc);
					for(int j=0;j<time_hover;j++)
					{				
						pub_twist.publish(twist_msg_hover);
						loop_rate.sleep();
						//ROS_INFO("hover");
					}
					twist_msg.linear.x=0;
					twist_msg.linear.y=0;					

				}
				else
				
{
					wait_proc=BOUNDARY;
					ROS_INFO("X Y POSITION OK");
				}
				if(!find_origin)
				{
					no_origin++;
				}
				if(no_origin>0)
				{
					no_origin=0;
					wait_proc=BOUNDARY;
					ROS_INFO("lose lose lose lose lose lose origin");
				}
					
			}
			//process ORIGIN end

			

			//monitor irobot by red area
			if(find_red)
			{
				work_s=CATCH_IROBOT;
				ROS_INFO("FIND irobot irobot irobot irobot irobot ");
			}
			else
			{
				ROS_INFO("DO NOT FIND IROBOT");
			}
			//MONITOR end
			ROS_INFO("WAITING WAITING WAITING WAITING======================");
		}
		//////////////////////////wait irobot end////////////////////////////////


		//////////////////////////catch irobot////////////////////////////////
		if(work_s==CATCH_IROBOT)
		{
			//drone will catch the irobot according by following red object and circle,keep upward it and calculate angle until it can be released
				//x y
				if(distance(red_x,red_y,0,0)>thresh_red)
				{
					int k_p=15,time_min=5,time_max=15,time_fly,time_hover;//test_catch_irobot			
					int dirc;
					double dist_toline;
					bool is_front=false;
					bool is_wait=false;
					bool is_online=false;

					if(find_angle&&is_send&&distance(red_x,red_y,0,0)>80)
					{
						ROS_INFO("0000000000000THETA IS IS %lf",theta_circle);

						dirc=planDirection(red_x,red_y,theta_circle,dist_toline,is_front);

						time_fly=k_p*dist_toline/350;
						if(time_fly>time_max)
							time_fly=time_max;
						if(time_fly<time_min)
							time_fly=time_min;
						const int thresh_line=70;
						if(dist_toline<thresh_line&&is_front)
						{
							is_wait=true;
							is_online=true;
						}
						//dirc=planTrack(red_x,red_y,theta_circle,true);
						time_hover=4;
						ROS_INFO("distace to line is %lf,is front irobot %d",dist_toline,int(is_front));
					}
					else	
					{
						if(!is_send||distance(red_x,red_y,0,0)<=80)
						{
							if(!is_send)							
								ROS_INFO("send ball sssssssssssssssssssssssssssssssssss");			
							if(distance(red_x,red_y,0,0)<=70)
								ROS_INFO("near target nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn");				
							dirc=planTrack(red_x,red_y,theta_circle,false);
							time_fly=k_p*distance(red_x,red_y,0,0)/350;
							if(time_fly>time_max)
								time_fly=time_max;
							if(time_fly<time_min)
								time_fly=time_min;
							time_hover=4;
						}
						else
						{
							ROS_INFO("DO NOT FIND ANGLE00000000000000000");
							time_fly=10;
							if(red_y<0)
							{
								dirc=planTrack(red_x,red_y,theta_circle,false);
							}
							else
							{
								is_wait=true;
							}
							time_hover=4;
						}
					}
					if(!is_wait)
					{
						switch(dirc)
						{
							case DIR_R:
							{
								twist_msg.linear.y=-1;	
								break;
							}
							case DIR_RF:
							{
								twist_msg.linear.y=-1;	
								twist_msg.linear.x=1;
								break;
							}	
							case DIR_F:
							{
								twist_msg.linear.x=1;
								break;
							}	
							case DIR_LF:
							{
								twist_msg.linear.y=1;	
								twist_msg.linear.x=1;
								break;
							}	
							case DIR_L:
							{
								twist_msg.linear.y=1;	
								break;
							}	
							case DIR_LB:
							{
								twist_msg.linear.y=1;
								twist_msg.linear.x=-1;
								break;
							}	
							case DIR_B:
							{
								twist_msg.linear.x=-1;
								break;
							}	
							case DIR_RB:
							{
								twist_msg.linear.x=-1;
								twist_msg.linear.y=-1;
								break;
							}				
						}
						ROS_INFO("DIR IS %d ",int(dirc));
						//z
						if(is_send)
						{
							if(fabs(red_x)<thresh_down&&(red_y+80)>0)
							{
								if(red_area<14000&&red_area!=0)
								{
									twist_msg.linear.z=-1;
									ROS_INFO("CATCH DOWN CATCH DOWN DOWN ");
								}
								if(red_area>16000||no_red>=5)
								{
									twist_msg.linear.z=1;	
									ROS_INFO("CATCH UP CATCH UP UP UP");
								}
							}
						}
						if(!find_red)
						{
							time_fly=15;
							time_hover=4;
						}
						for(int j=0;j<time_fly;j++)
						{	
							pub_twist.publish(twist_msg);
							loop_rate.sleep();
						
						}
						for(int j=0;j<time_hover;j++)
						{				
							pub_twist.publish(twist_msg_hover);
							loop_rate.sleep();
							//ROS_INFO("hover");
						}
						twist_msg.linear.x=0;
						twist_msg.linear.y=0;
						twist_msg.linear.z=0;
						//x y end
						if(is_send&&red_area<14000)
						{
							if(down_time<3)
							{
								twist_msg.linear.z=-1;
								for(int j=0;j<30;j++)
								{	
									pub_twist.publish(twist_msg);
									loop_rate.sleep();
								}
								twist_msg.linear.z=0;
								ROS_INFO("DOWN TO WAIT");
								down_time++;
								
							}	
						}
					}	
					else
					{
						if(is_online&&down_time<3)
						{
							twist_msg.linear.z=-1;
							for(int j=0;j<30;j++)
							{	
								pub_twist.publish(twist_msg);
								loop_rate.sleep();
							}
							twist_msg.linear.z=0;
							ROS_INFO("DOWN TO WAIT");
							down_time++;
							
						}
						for(int j=0;j<time_hover;j++)
						{				
							pub_twist.publish(twist_msg_hover);
							loop_rate.sleep();
							ROS_INFO("hover");
						}	
					}
				}
				if(distance(red_x,red_y,0,0)<50&&find_red)
				{
					if(red_area<14000&&red_area!=0)
					{
						twist_msg.linear.z=-1;
						ROS_INFO("CATCH DOWN CATCH DOWN DOWN ");
					}
					if(red_area>25000||no_red>=5)
					{
						twist_msg.linear.z=1;	
						ROS_INFO("CATCH UP CATCH UP UP UP");
					}
					for(int j=0;j<15;j++)
					{	
						pub_twist.publish(twist_msg);
						loop_rate.sleep();
					}
					for(int j=0;j<5;j++)
					{				
						pub_twist.publish(twist_msg_hover);
						loop_rate.sleep();
						ROS_INFO("hover");
					}	
					twist_msg.linear.z=0;
				}
				if(!find_red)
				{
					no_red++;
					//red_x=red_x*2;
					//red_y=2*red_y;
					/*if(fabs(red_x)>100||(fabs(red_y>60)&&red_y<0))
					{
						for(int j=0;j<10;j++)
						{				
							pub_twist.publish(twist_msg_hover);
							loop_rate.sleep();
							//ROS_INFO("hover");
						}
					}
					red_x=10*red_x/fabs(red_x);
					red_y=10*red_y/fabs(red_y);*/
					
				}
				if(no_red>=7)
				{
					no_red=0;
					work_s=WAIT_IROBOT;
					ROS_INFO("lose irobot lose irobot lose irobot lose irobot lose irobot lose irobot ");
				}
				ROS_INFO("CATCHING CATCHING- CATCHING CATCHING>>>>>>>>>>>>>>>>>>>>>>");
				if(can_release)
				{
					work_s=RELEASE_IROBOT;		
				}
				find_red=false;
				ROS_INFO("RED AREA:%d,ALTITUDE:%d",red_area,altitude_drone);

		}
		//////////////////////////catch irobot end////////////////////////////////


		//////////////////////////release irobot////////////////////////////////
		if(work_s==RELEASE_IROBOT)
		{
			down_time=0;
			//drone release the irobot,delay for a while and shift work state----go to boundary waiting for the irobot
			twist_msg.linear.x=-1;
			for(int j=0;j<30;j++)
			{	
				pub_twist.publish(twist_msg);
				loop_rate.sleep();
				ROS_INFO("RELEASE!!!!!!!!!!!!!");
			}
			for(int j=0;j<150;j++)
			{				
				pub_twist.publish(twist_msg_hover);
				loop_rate.sleep();
				ROS_INFO("RELEASE FINISH===  ===  ====");
			}
			twist_msg.linear.x=1;
			for(int j=0;j<30;j++)
			{	
				pub_twist.publish(twist_msg);
				loop_rate.sleep();
				ROS_INFO("GO BACK GO BACK===  ===  ===  ===");
			}
			twist_msg.linear.x=0;
			for(int j=0;j<20;j++)
			{				
				pub_twist.publish(twist_msg_hover);
				loop_rate.sleep();
				ROS_INFO("BACK BACK===  ===  ====");
			}
			for(int j=0;j<30;j++)
			{	
				pub_twist.publish(twist_msg);
				loop_rate.sleep();
				ROS_INFO("GO BACK GO BACK===  ===  ===  ===");
			}
			for(int j=0;j<20;j++)
			{				
				pub_twist.publish(twist_msg_hover);
				loop_rate.sleep();
				ROS_INFO("BACK BACK===  ===  ====");
			}
		
			for(int j=0;j<10;j++)
			{				
				pub_twist.publish(twist_msg_hover);
				loop_rate.sleep();
				//ROS_INFO("hover");
			}	
			work_s=WAIT_IROBOT;
			can_release=false;
			theta_circle=0;
			is_send=true;//send 
		}
		//////////////////////////release irobot end////////////////////////////////
		ros::spinOnce();
		loop_rate.sleep();
	}
////////////////////////////////////main loop end/////////////////////////////////////////////////////////////




////////////////////////////////////destruction/////////////////////////////////////////////////////////
	cv::destroyWindow(wndname);
	cv::namedWindow("find_red_window");
	cv::namedWindow("contour view");
	//cv::namedWindow("view4");
////////////////////////////////////destruction end/////////////////////////////////////////////////////////
 
}
/////////////////////////////function:main end////////////////////////////////////////////////////////

