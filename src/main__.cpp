/*
 * test.cpp
 *
 *  Created on: May 14, 2013
 *      Author: shengzhe
 */

#include <iostream>
#include <iomanip>
#include "ros/ros.h"
#include "math/SL_Matrix.h"

#include "std_msgs/String.h"

#include "imgproc/SL_Image.h"
#include "imgproc/SL_ImageIO.h"

#include "tools/GUI_ImageViewer.h"
#include "tools/SL_Print.h"
#include "tools/SL_DrawCorners.h"

#include "ROSThread.h"
#include "VideoRecorder.h"
#include "IMURecorder.h"
#include "GridDetector.h"
#include "PIDController.h"

#include <opencv2/opencv.hpp>
#include <vector>
///////////////////////////////////
//#include <ardrone_autonomy/Navdata.h>
//#include <std_msgs/Empty.h>
//#include <geometry_msgs/Twist.h>
//#include "precomp.hpp"
#include "opencv2/legacy/blobtrack.hpp"
#include "math.h"
#define PI 3.14159265


using namespace std;

static int mGrids = 5;
static int nGrids = 6;
//////////////////////////////////
/*geometry_msgs::Twist twist_msg;
geometry_msgs::Twist twist_msg_neg;
geometry_msgs::Twist twist_msg_hover;
geometry_msgs::Twist twist_msg_up;
std_msgs::Empty emp_msg;*/
/////////////////////////////////

bool detectAndShowGrid(cv::Mat& img, vector<cv::Point2f>& featPts) {
    if (detectCircleGrids(img, mGrids, nGrids, featPts, 50.0f)) {
        cv::Mat tmpImg = img.clone();

        for (size_t i = 0; i < featPts.size(); i++) {
            cv::circle(tmpImg, featPts[i], 3, cv::Scalar(255, 0, 0), 2, CV_AA, 0);
        }

        cv::imshow("img", tmpImg);
        cv::waitKey(100);
        return true;
    }
    return false;
}

void showSnapshot(const ImgRGB& img){
    ImgRGB tmp;
    cloneImg(img, tmp);

    int len = tmp.m*tmp.n;
    for( int i = 0; i < len ; i++){
        tmp.data[3*i] = 255;
    }

    imshow("img", tmp);
    cv::waitKey(200);
}

void writeFeatPts(const vector<cv::Point2f>& featPts, const char* filePath) {
    ofstream file(filePath);
    if (!file)
        repErr("cannot open '%s' to write!", filePath);

    for (size_t i = 0; i < featPts.size(); i++) {
        file << featPts[i].x << " " << featPts[i].y << endl;
    }
}

void writeIMUMeas(const vector<IMUData>& imumeas, const char* filePath){
    std::ofstream file(filePath);
    if(!file)
        repErr("cannot open '%' to write!", filePath);

    for( size_t i = 0; i < imumeas.size(); i++){
        imumeas[i].write(file);
    }
    file.close();
}

int ROS_main(int argc, char** argv) {
    ros::init(argc, argv, "ardrone_data_logger");

    IMURecorder imureader("/home/shengzhe/Record/imu.txt");
    VideoRecorder videoreader("/home/shengzhe/Record/video_ts.txt",
            "/home/shengzhe/Record/video.avi");

    ROSThread thread(imureader, videoreader);
    thread.showVideo = true;

    ImgRGB img(400, 400);
    int ts = 30;
    char ch = cv::waitKey(ts);

    int k_old = 0;
    TimeMeasurer tm;
    while (ch != 'q') {
        if (ch == 'r') {
            tm.tic();
            imureader._recording = true;
            videoreader._recording = true;
            img.fill(255, 0, 0);
        }
        if( ch == 's'){
            //shoot calibration patterns to calibrate the relative orientation
            //between IMU and the camera
            static int k = 0;
            ImgRGB curImg;
            vector<IMUData> imumeas;
            videoreader.getImage(curImg);
            imureader.getCachedMeas(imumeas);

            const char* dirPath = "/home/shengzhe/Record";
            std::stringstream ss;
            ss << dirPath << "/snapshot_" << k << ".png";

            imwrite(curImg, ss.str().c_str());

            ss.str("");
            ss.clear();
            ss << dirPath << "/imu_" << k++ << ".txt";
            writeIMUMeas(imumeas, ss.str().c_str());;
            cout << "snap shot!" << endl;
            showSnapshot(curImg);
        }

        if (!videoreader.curImg.empty()) {
            cloneImg(videoreader.curImg, img);
            bgr2rgb(img);
            if (videoreader._recording) {
                drawPoint(img, 15, 15, 6, 255, 0, 0, 2);
            }
        }
        imshow("img", img);
        ch = cv::waitKey(ts);

        if (imureader._recording) {
            double ts = tm.toc() / 1000;
            int k = ((int) ts) / 5;
            if (((int) ts) % 5 == 0 && k != k_old) {
                cout << "recording time: " << ts << " s" << endl;
                cout << "K:" << k << " k_old:" << k_old << endl;
            }
            k_old = k;
        }
    }
    return 0;
}

static int CompareContour(const void* a, const void* b, void* )
{
    float           dx,dy;
    float           h,w,ht,wt;
    CvPoint2D32f    pa,pb;
    CvRect          ra,rb;
    CvSeq*          pCA = *(CvSeq**)a;
    CvSeq*          pCB = *(CvSeq**)b;
    ra = ((CvContour*)pCA)->rect;
    rb = ((CvContour*)pCB)->rect;
    pa.x = ra.x + ra.width*0.5f;
    pa.y = ra.y + ra.height*0.5f;
    pb.x = rb.x + rb.width*0.5f;
    pb.y = rb.y + rb.height*0.5f;
    w = (ra.width+rb.width)*0.5f;
    h = (ra.height+rb.height)*0.5f;

    dx = (float)(fabs(pa.x - pb.x)-w);
    dy = (float)(fabs(pa.y - pb.y)-h);

    //wt = MAX(ra.width,rb.width)*0.1f;
    wt = 0;
    ht = MAX(ra.height,rb.height)*0.3f;
    return (dx < wt && dy < ht);
}

static void cvFindBlobsByCCClasters(IplImage* pFG, CvBlobSeq* pBlobs, CvMemStorage* storage)
{   /* Create contours: */
    IplImage*       pIB = NULL;
    CvSeq*          cnt = NULL;
    CvSeq*          cnt_list = cvCreateSeq(0,sizeof(CvSeq),sizeof(CvSeq*), storage );
    CvSeq*          clasters = NULL;
    int             claster_cur, claster_num;

    pIB = cvCloneImage(pFG);
    cvThreshold(pIB,pIB,128,255,CV_THRESH_BINARY);
    cvFindContours(pIB,storage, &cnt, sizeof(CvContour), CV_RETR_EXTERNAL);
    cvReleaseImage(&pIB);

    /* Create cnt_list.      */
    /* Process each contour: */
    for(; cnt; cnt=cnt->h_next)
    {
        cvSeqPush( cnt_list, &cnt);
    }

    claster_num = cvSeqPartition( cnt_list, storage, &clasters, CompareContour, NULL );

    for(claster_cur=0; claster_cur<claster_num; ++claster_cur)
    {
        int         cnt_cur;
        CvBlob      NewBlob;
        double      M00,X,Y,XX,YY; /* image moments */
        CvMoments   m;
        CvRect      rect_res = cvRect(-1,-1,-1,-1);
        CvMat       mat;

        for(cnt_cur=0; cnt_cur<clasters->total; ++cnt_cur)
        {
            CvRect  rect;
            CvSeq*  cont;
            int k = *(int*)cvGetSeqElem( clasters, cnt_cur );
            if(k!=claster_cur) continue;
            cont = *(CvSeq**)cvGetSeqElem( cnt_list, cnt_cur );
            rect = ((CvContour*)cont)->rect;

            if(rect_res.height<0)
            {
                rect_res = rect;
            }
            else
            {   /* Unite rects: */
                int x0,x1,y0,y1;
                x0 = MIN(rect_res.x,rect.x);
                y0 = MIN(rect_res.y,rect.y);
                x1 = MAX(rect_res.x+rect_res.width,rect.x+rect.width);
                y1 = MAX(rect_res.y+rect_res.height,rect.y+rect.height);
                rect_res.x = x0;
                rect_res.y = y0;
                rect_res.width = x1-x0;
                rect_res.height = y1-y0;
            }
        }

        if(rect_res.height < 1 || rect_res.width < 1)
        {
            X = 0;
            Y = 0;
            XX = 0;
            YY = 0;
        }
        else
        {
            cvMoments( cvGetSubRect(pFG,&mat,rect_res), &m, 0 );
            M00 = cvGetSpatialMoment( &m, 0, 0 );
            if(M00 <= 0 ) continue;
            X = cvGetSpatialMoment( &m, 1, 0 )/M00;
            Y = cvGetSpatialMoment( &m, 0, 1 )/M00;
            XX = (cvGetSpatialMoment( &m, 2, 0 )/M00) - X*X;
            YY = (cvGetSpatialMoment( &m, 0, 2 )/M00) - Y*Y;
        }
        NewBlob = cvBlob(rect_res.x+(float)X,rect_res.y+(float)Y,(float)(4*sqrt(XX)),(float)(4*sqrt(YY)));
        pBlobs->AddBlob(&NewBlob);

    }   /* Next cluster. */
}

//void * CLIP3(double n1, double &n, double n2){ if (n<n1) n=n1;  if (n>n2) n=n2;}

#include "ARDrone.h"
void* Control_loop(void* param) {
    IMURecorder imureader("/home/shengzhe/Record/imu.txt");
   VideoRecorder videoreader("/home/shengzhe/Record/video_ts.txt",
            "/home/shengzhe/Record/video.avi");
    ROSThread thread(imureader, videoreader);
    thread.showVideo = true;
    ImgRGB img(640, 360);
    IplImage *imgr,* imgg, *imgb;
    CvSize imgSize={640,360};
    imgr= cvCreateImage(imgSize, IPL_DEPTH_8U, 1 );
    imgg= cvCreateImage(imgSize, IPL_DEPTH_8U, 1 );
    imgb= cvCreateImage(imgSize, IPL_DEPTH_8U, 1 );
    //IplImage* red,green, blue;
        system("rosservice call /ardrone/setcamchannel 1");
////////////////////////////////
    ARDrone* drone = new ARDrone();
    //drone->open();
    drone->setup();
    ros::Rate loop_rate(50);
    int i , j;
    uchar * data, *datar, *datag, *datab;
    int blobDetected=0, pathDetected=0, blueDetected=0;
    int controlMode=0; //0: Manual; 1: Auto
    PIDController pidX, pidY, pidZ;
    int frame_count=0, lostframe=0;
    //CvBlobSeq* pOldBlobList=NULL;
    char c;

cout << "Start!" << endl;

    while (1) {
        //cv::waitKey(1);
        usleep(1000);
        lostframe ++;
        if (c=='x')  drone->land();
        if (lostframe>100) drone->hover();
    if (videoreader.newframe){
        frame_count++;
        lostframe=0;
        videoreader.newframe=false;

        //imshow("control", img);
        /////////////////////////////////////
        cout << "Battery:" << thread.navdata.batteryPercent << "% Rotate: " <<setw(8) << thread.navdata.rotX << " "<<setw(8) << thread.navdata.rotY <<" ";//<<setw(12) << thread.navdata.rotZ;
        cout << " Height:"<<setw(8)<< thread.navdata.altd << "  v: "<<setw(8) <<  thread.navdata.vx << " "<<setw(8)<< thread.navdata.vy ;//<<" "<< thread.navdata.vz;
        c = (char) cv::waitKey(1);
        // ESC key pressed
        if (c == 27 || c == 'q')
            break;
        if (c > -1)
            cout << " key press: " << (int) c ;//<< endl;

        //cout << endl;
        blobDetected=false;
        pathDetected=false;
        blueDetected=false;
///////////////////////////////////////////
#define THRESH 70
#define THRESHG 35
#define THRESHB 55

if (!videoreader.curImg.empty()) {
            cloneImg(videoreader.curImg, img);
            bgr2rgb(img);
            if (videoreader._recording) {
                drawPoint(img, 15, 15, 6, 255, 0, 0, 2);
            }
        }
        //imshow("Original", img);
        //img.split(imgr, imgg, imgb);
        data = img.data;
        datar= (uchar*)imgr->imageData;
        datag=(uchar*) imgg->imageData;
        datab=(uchar*) imgb->imageData;
        for (i=0; i<img.h; ++i)
            for (j=0; j<img.w; ++j){
                if ( (data[0]-data[1])> THRESH && (data[0]-data[2])> THRESH )
                    datar[0]=255;
                else datar[0]=0;
                if ( (data[1]-data[0])> THRESHG && (data[1]-data[2])> THRESHG )
                    datag[0]=255;
                else if ( (data[1]-data[0])> THRESHG/2 && (data[1]-data[2])> THRESHG/2 &&  (data[0]+data[1]+data[2])<100 )
                    datag[0]=255;
                else datag[0]=0;
                if ( (data[2]-data[0])> THRESHB && (data[2]-data[1])> THRESHB )
                   datab[0]=255;
                else datab[0]=0;
                data+=3;
                datar++; datag++; datab++;
            }
        //imshow("Red", imgr->imageData);
        cvShowImage("Red", imgr);
        cvShowImage("Green", imgg);
        cvShowImage("Blue", imgb);
        //imshow("Green", imgg);
        CvBlobSeq       BlobsGreen;
        CvBlobSeq       BlobsRed;
        CvBlobSeq       BlobsBlue;
        CvBlob newBlob;
        CvMemStorage *   storage = cvCreateMemStorage();

      //Green  /* Glue contours: */
        cvFindBlobsByCCClasters(imgg, &BlobsGreen, storage );
        {   /* Delete small and intersected blobs: */
            int i;
            for(i=BlobsGreen.GetBlobNum(); i>0; i--)
            {
                CvBlob* pB = BlobsGreen.GetBlob(i-1);

                if(pB->h < imgSize.height*0.02 || pB->w <imgSize.width*0.02)
                {
                    BlobsGreen.DelBlob(i-1);
                    continue;
                }
            }   /* Check next blob. */
        }   /*  Delete small and intersected blobs. */

        {   /* Bubble-sort blobs by size: */
            int N = BlobsGreen.GetBlobNum();
            int i,j;
            for(i=1; i<N; ++i)
            {
                for(j=i; j>0; --j)
                {
                    CvBlob  temp;
                    float   AreaP, AreaN;
                    CvBlob* pP = BlobsGreen.GetBlob(j-1);
                    CvBlob* pN = BlobsGreen.GetBlob(j);
                    AreaP = CV_BLOB_WX(pP)*CV_BLOB_WY(pP);
                    AreaN = CV_BLOB_WX(pN)*CV_BLOB_WY(pN);
                    if(AreaN < AreaP)break;
                    temp = pN[0];
                    pN[0] = pP[0];
                    pP[0] = temp;
                }
            }
            if (N>0) blobDetected=true;
        }   /* Sort blobs by size. */
        //////////////////////////////////////////////////////////////////////////
        cvReleaseMemStorage(&storage);
        storage = cvCreateMemStorage();

        //Blue  /* Glue contours: */
        cout << "beforeblue";
          cvFindBlobsByCCClasters(imgb, &BlobsBlue, storage );
          cout <<" blue ";
          {   /* Delete small and intersected blobs: */
              int i;
              for(i=BlobsBlue.GetBlobNum(); i>0; i--)
              {
                  CvBlob* pB = BlobsBlue.GetBlob(i-1);

                 // if(pB->h < imgSize.height*0.02 || pB->w <imgSize.width*0.02)
                  if ((pB->h * pB->w) <64)
                  {
                      BlobsBlue.DelBlob(i-1);
                      continue;
                  }
              }   /* Check next blob. */
          }   /*  Delete small and intersected blobs. */

          {   /* Bubble-sort blobs by size: */
              int N = BlobsBlue.GetBlobNum();
              int i,j;
              for(i=1; i<N; ++i)
              {
                  for(j=i; j>0; --j)
                  {
                      CvBlob  temp;
                      float   AreaP, AreaN;
                      CvBlob* pP = BlobsBlue.GetBlob(j-1);
                      CvBlob* pN = BlobsBlue.GetBlob(j);
                      AreaP = CV_BLOB_WX(pP)*CV_BLOB_WY(pP);
                      AreaN = CV_BLOB_WX(pN)*CV_BLOB_WY(pN);
                      if(AreaN < AreaP)break;
                      temp = pN[0];
                      pN[0] = pP[0];
                      pP[0] = temp;
                  }
              }
              if (N>0) blueDetected=true;
          }   /* Sort blobs by size. */
        if (blueDetected) drawPoint(img,  BlobsBlue.GetBlob(0)->x,  BlobsRed.GetBlob(0)->y, 6, 255, 127, 0, 5);
        /////////////////////////////////////////////////////////////////////////
        cvReleaseMemStorage(&storage);
        storage = cvCreateMemStorage();
        //Red  /* Glue contours: */
          cvFindBlobsByCCClasters(imgr, &BlobsRed, storage );
          {   /* Delete small and intersected blobs: */
              int i;
              for(i=BlobsRed.GetBlobNum(); i>0; i--)
              {
                  CvBlob* pB = BlobsRed.GetBlob(i-1);

                  //if(pB->h < imgSize.height*0.02 || pB->w <imgSize.width*0.02)
                  if( (pB->h * pB->w) <64)
                  {
                      BlobsRed.DelBlob(i-1);
                      continue;
                  }
                 /* if(pOldBlobList)
                  {
                      int j;
                      for(j=pOldBlobList->GetBlobNum(); j>0; j--)
                      {
                          CvBlob* pBOld = pOldBlobList->GetBlob(j-1);
                          if((fabs(pBOld->x-pB->x) < (CV_BLOB_RX(pBOld)+CV_BLOB_RX(pB)))
                            &&  (fabs(pBOld->y-pB->y) < (CV_BLOB_RY(pBOld)+CV_BLOB_RY(pB))))
                          {   // Intersection is present, so delete blob from list:
                              BlobsRed.DelBlob(i-1);
                              break;
                          }
                      }  //  Check next old blob.
                  }*/ ////////////  //  if pOldBlobList
              }   /* Check next blob. */
          }   /*  Delete small and intersected blobs. */

          int N = BlobsRed.GetBlobNum();
          {   /* Bubble-sort blobs by size: */

              int i,j;
              for(i=0; i<N; ++i)
              {
                  for(j=i+1; j<N; ++j)
                  {
                      CvBlob  temp;
                     // float   AreaP, AreaN;
                      CvBlob* pP = BlobsRed.GetBlob(i);
                      CvBlob* pN = BlobsRed.GetBlob(j);
                      //AreaP = CV_BLOB_WX(pP)*CV_BLOB_WY(pP);
                     // AreaN = CV_BLOB_WX(pN)*CV_BLOB_WY(pN);
                      //if(AreaN < AreaP)break;
                      if( (pP->x) < (pN->x) ) {
                      temp = pN[0];
                      pN[0] = pP[0];
                      pP[0] = temp;
                      }
                  }
              }
              if (N>0) pathDetected=true;
              if (pathDetected) drawPoint(img,  BlobsRed.GetBlob(0)->x,  BlobsRed.GetBlob(0)->y, 6, 255, 255, 0, 5);
              for (i=1; i<N; ++i) drawPoint(img,  BlobsRed.GetBlob(i)->x,  BlobsRed.GetBlob(i)->y, 5, 0, 255, 255, 4);
              /* Copy only first 10 blobs: */
              //for(i=0; i<MIN(N,10); ++i)
              //{
              //    m_pBlobLists[EBD_FRAME_NUM-1]->AddBlob(Blobs.GetBlob(i));
              //}

          }   /* Sort blobs by size. */
          /////////////////////////////////
        cvReleaseMemStorage(&storage);

        double targetx, targety;
        double centerx, centery;
        double errorx, errory;
        double lasterrorx, lasterrory;
        double targetvx, targetvy;
        double leftr, forwardb, upd, turnleft;
        double k=2.0;//0.001;
        static double kp=3.0;//0.0001;l
        static double kd=150.0;
         static double ki=0.0;
        double scale=0.0007;
        double x1,y1, x2,y2, slope;
       // PIDController pidX, pidY, pidZ;

        pidX.setParam(kp, ki, kd, 2);
        pidY.setParam(kp, ki, kd, 2);
        pidZ.setParam(kp, ki, kd, 3);

        if (blobDetected){
            targetx=(BlobsGreen.GetBlob(0))->x;
            targety=(BlobsGreen.GetBlob(0))->y;
             drawPoint(img, targetx, targety, 5, 255, 255, 0, 2);
        }
        //cout << "ok" << endl;

#define CLIP3(_n1, _n,  _n2) {if (_n<_n1) _n=_n1;  if (_n>_n2) _n=_n2;}
        centery=180- tan(thread.navdata.rotY * PI/180) * 800;//0.5104*thread.navdata.altd;
        centerx=300+ tan(thread.navdata.rotX * PI/180) * 800;//0.5104*thread.navdata.altd;
        CLIP3(10.0, centerx, 590.0);
        CLIP3(10.0, centery, 350.0);
        drawPoint(img, centerx, centery, 3, 0, 0, 255, 2);


        errorx=targetx - centerx;
        errory=targety - centery;
        if (controlMode==1) {
            targetvx = - k * errory- k*(errory-lasterrory);
            targetvy = - k * errorx- k*(errorx-lasterrorx);
        } else if (controlMode ==2) {
            targetvx = 0;
            targetvy = -400;
        }
        if (pathDetected)  {
            //turn = (180 - BlobsRed.GetBlob(0)->y) / 256;
            x1=300; y1=180; x2=300; y2=180;
            if (N==1){
                x1 = BlobsRed.GetBlob(0)->x;
                y1 = BlobsRed.GetBlob(0)->y;
            }
            else if (N==2) {
                x1 = (BlobsRed.GetBlob(0)->x +BlobsRed.GetBlob(1)->x)/2 ;
                y1 = (BlobsRed.GetBlob(0)->y + BlobsRed.GetBlob(1)->y)/2;
            }
            else if (N==3){
                x1 = BlobsRed.GetBlob(0)->x;
                y1 = BlobsRed.GetBlob(0)->y;
                x2 = BlobsRed.GetBlob(2)->x;
                y2 = BlobsRed.GetBlob(2)->y;
            }
            else {
                x1 = (BlobsRed.GetBlob(0)->x +BlobsRed.GetBlob(1)->x)/2 ;
                y1 = (BlobsRed.GetBlob(0)->y + BlobsRed.GetBlob(1)->y)/2;
                x2 = BlobsRed.GetBlob(3)->x;
                y2 = BlobsRed.GetBlob(3)->y;
            }
            if (x1==x2) slope =0;
            else slope = - (y1-y2)/(x1-x2);
            CLIP3(-10, slope, 10);
            turnleft = (180 - y1) / 300;
            if (N>=3) turnleft += slope/4;
            //cout << "ok";
        }
        CLIP3(-1.0, turnleft, 1.0);
        CLIP3(-200.0, targetvx, 200.0);
        CLIP3(-400.0, targetvy, 400.0);

        //leftr = - kp* errorx - kd*(errorx-lasterrorx);//*thread.navdata.altd;
        //forwardb = - kp * errory- kd*(errory-lasterrory);;//*thread.navdata.altd;
        leftr= - pidX.getOutput(errorx,1);
        forwardb= -pidY.getOutput(errory,1);
        leftr/=10000;        forwardb/=10000;
        forwardb = scale*(targetvx - thread.navdata.vx);
         leftr= scale*(targetvy - thread.navdata.vy);
        //forwardb = scale*targetvx;
        //leftr= scale*targetvy;
        CLIP3(-0.1, leftr, 0.1);
        CLIP3(-0.1, forwardb, 0.1);
        if (thread.navdata.altd>1200) upd=-0.001*(thread.navdata.altd-1200);
        else if (thread.navdata.altd<1150) upd=0.0011*(1150-thread.navdata.altd);
        else upd=0.0;
        CLIP3(-0.1, upd, 0.1);
        if (blobDetected){
            drawLine(img, 300, 180, 300-leftr*1000, 180, 255,255,255 );
            drawLine(img, 300, 180, 300, 180-forwardb*1000, 255,255,255 );
        }
        imshow("Original", img);
        cout << "  Control: " << setw(8) << leftr <<", " << setw(8)<<forwardb<<", "<< setw(4)<<upd <<" kp=" <<kp<<" kd="<<kd  << endl;
        lasterrorx=errorx;
        lasterrory=errory;
///////////////////////////////////////////////

        switch (c) {
        case 82: kp+=0.1;   break;
        case 84: kp-=0.1;    break;
        case 83: kd+=1;   break;
        case 81: kd-=1;    break;
        case 'z':
            drone->takeOff();
            break;

        case 'x':
            drone->land();
            break;

        case 'c':
            drone->resumeNormal();
            break;

        case 'v':
            drone->emergency();
            break;

        case 'h':
            drone->hover();
            break;

        case 'i':
            drone->moveUp((float) 0.2);
            break;

        case 'k':
            drone->moveDown((float) 0.2);
            break;

        case 'j':
            drone->turnLeft((float) 0.4);
            break;

        case 'l':
            drone->turnRight((float) 0.4);
            break;

        case 'a':
            drone->moveLeft((float) 0.2);
            break;

        case 'd':
            drone->moveRight((float) 0.2);
            break;

        case 'w':
            drone->moveForward((float) 0.2);
            break;

        case 's':
            drone->moveBackward((float) 0.2);
            break;

        case 'y':
            controlMode=1;
            pidX.reset();
            pidY.reset();
            pidZ.reset();
            break;
        case 't':
            controlMode=2;
            break;
        case 'u':
            controlMode=0;
            break;

        default:
            //drone->hover();
            switch (controlMode) {
            case 0:
                drone->hover();
                break;
            case 1:
                if (blobDetected)  drone->move(leftr, forwardb, upd, 0.0);
                else drone->moveUp(upd);
                break;
            case 2:
                if (pathDetected) drone->move(leftr, forwardb, upd, turnleft);
                else drone->moveUp(upd);
                break;
            default:
                 drone->hover();
            }
        }

         //c = (char) cv::waitKey(10);
        //loop_rate.sleep();
    }
    }

    cout << endl << "No. of frames: "<< frame_count <<endl;

    delete drone;
    cvReleaseImage(&imgr);
    cvReleaseImage(&imgg);
    cvReleaseImage(&imgb);
    return 0;
}

void ROSControl_main(int argc, char** argv) {
    //ros::init(argc, argv, "listener");
    pthread_t ROS_thread, control_thread;
    int rc = pthread_create(&control_thread, NULL, Control_loop, 0);
    if (rc) {
        printf("ERROR; return code from pthread_create() is %d\n", rc);
        exit(-1);
    }

//	rc = pthread_create(&ROS_thread, NULL, ROS_loop, 0);
//	if (rc) {
//		printf("ERROR; return code from pthread_create() is %d\n", rc);
//		exit(-1);
//	}
    pthread_join(control_thread, NULL);
//	pthread_join(ROS_thread,NULL);
}


#include "IMUVideoSync.h"
int main(int argc, char** argv) {
    ros::init(argc, argv,"ARDrone_test");
 //ROS_main(argc, argv);
    ROSControl_main(argc, argv);

    //std_msgs::Empty emp_msg;
    //ros::init(argc, argv,"ARDrone_test");
      //  ros::NodeHandle node;
       // ros::Rate loop_rate(50);
       // ros::Publisher pub_empty;
    //system( "rosservice  call /ardrone/setledanimation 5 2 5" );
    //system("rosservice call /ardrone/togglecam");


  //Control_loop(argc, argv);
    ////////////////////////////////////////////SZ
   #if 0
   pub_empty = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */

        while (ros::ok())
                    {
                    double time_start=(double)ros::Time::now().toSec();
                    while ((double)ros::Time::now().toSec()< time_start+5.0) /* Send command for five seconds*/
                        {
                        printf("hhhhhhhhhhh");
                        pub_empty.publish(emp_msg); /* launches the drone */
                        ros::spinOnce();
                        loop_rate.sleep();
                        }//time loop
                    ROS_INFO("ARdrone launched");
                    //exit(0);
                    }//ros::ok loop
        pub_empty = node.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */


            while (ros::ok()) {
                     double time_start=(double)ros::Time::now().toSec();
                    while ((double)ros::Time::now().toSec()< time_start+5.0){
                pub_empty.publish(emp_msg); //launches the droe
                    ros::spinOnce();
                    loop_rate.sleep();
        }
                    ROS_INFO("ARdrone landed");
                    //exit(0);
             }//ros::ok
    #endif

     #if 0
            ROS_INFO("ARdrone Test Back and Forth Starting");
                ros::init(argc, argv,"ARDrone_test");
                ros::NodeHandle node;
                ros::Rate loop_rate(50);

                ros::Publisher pub_empty_land;
                ros::Publisher pub_twist;
                ros::Publisher pub_empty_takeoff;
                ros::Publisher pub_empty_reset;
                double start_time;

            //hover message
                        twist_msg_hover.linear.x=0.0;
                        twist_msg_hover.linear.y=0.0;
                        twist_msg_hover.linear.z=0.0;
                        twist_msg_hover.angular.x=0.0;
                        twist_msg_hover.angular.y=0.0;
                        twist_msg_hover.angular.z=0.0;
            //up message
                        twist_msg_up.linear.x=0.0;
                        twist_msg_up.linear.y=0.0;
                        twist_msg_up.linear.z=0.5;
                        twist_msg_up.angular.x=0.0;
                        twist_msg_up.angular.y=0.0;
                        twist_msg_up.angular.z=0.0;
            //command message
                        float takeoff_time=0;//5.0;
                        float fly_time=0;//4.0;
                        float land_time=3.0;
                        float kill_time =2.0;


                        twist_msg.linear.x=0.0;
                        twist_msg.linear.y=0.25;
                        twist_msg.linear.z=0.0;
                        twist_msg.angular.x=0.0;
                        twist_msg.angular.y=0.0;
                        twist_msg.angular.z=0.0;

                        twist_msg_neg.linear.x=-twist_msg.linear.x;
                        twist_msg_neg.linear.y=-twist_msg.linear.y;
                        twist_msg_neg.linear.z=-twist_msg.linear.z;
                        twist_msg_neg.angular.x=-twist_msg.angular.x;
                        twist_msg_neg.angular.y=-twist_msg.angular.y;
                        twist_msg_neg.angular.z=-twist_msg.angular.z;



                pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1); /* Message queue length is just 1 */
                pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
                pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */
            pub_empty_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1); /* Message queue length is just 1 */


                start_time =(double)ros::Time::now().toSec();
                ROS_INFO("Starting ARdrone_test loop");


            while (ros::ok()) {
                    while ((double)ros::Time::now().toSec()< start_time+takeoff_time){ //takeoff

                        pub_empty_takeoff.publish(emp_msg); //launches the drone
                            pub_twist.publish(twist_msg_hover); //drone is flat
                        ROS_INFO("Taking off");
                        ros::spinOnce();
                        loop_rate.sleep();
                        }//while takeoff

                    while  ((double)ros::Time::now().toSec()> start_time+takeoff_time+fly_time){

                        pub_twist.publish(twist_msg_hover); //drone is flat
                        pub_empty_land.publish(emp_msg); //lands the drone
                        ROS_INFO("Landing");


                        if ((double)ros::Time::now().toSec()> takeoff_time+start_time+fly_time+land_time+kill_time){

                            ROS_INFO("Closing Node");
                            //pub_empty_reset.publish(emp_msg); //kills the drone
                            exit(0); 	}//kill node
                        ros::spinOnce();
                        loop_rate.sleep();
            }//while land

                    while ( (double)ros::Time::now().toSec()> start_time+takeoff_time && 						(double)ros::Time::now().toSec()< start_time+takeoff_time+fly_time){


                        if((double)ros::Time::now().toSec()< start_time+takeoff_time+fly_time/2){
                        pub_twist.publish(twist_msg);
                        ROS_INFO("Flying +ve");

                        }//fly according to desired twist

                        if((double)ros::Time::now().toSec()> start_time+takeoff_time+fly_time/2){
                        pub_twist.publish(twist_msg_neg);
                        ROS_INFO("Flying -ve");

                        }//fly according to desired twist

                        ros::spinOnce();
                    loop_rate.sleep();
                        }

                ros::spinOnce();
                loop_rate.sleep();

            }//ros::ok
    #endif
    /////////////////////////////////////////////////
    //ROSControl_main(argc, argv);
    //return IMUVideoSync_main(argc, argv);
    //return Drone_main(argc, argv);
    return 0;
}
