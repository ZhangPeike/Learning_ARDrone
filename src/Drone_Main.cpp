#include "DroneIMUThread.h"
#include "DroneVideoThread.h"

//read the drone sensor data without using the 'ardrone_autonomy' ROS package

int Drone_main(int argc, char** argv) {
    IMURecorder imu("/home/tsou/Record/imu.txt");
    VideoRecorder vid("/home/tsou/Record/ts.txt",
            "/home/tsou/Record/video.avi");

    DroneIMUThread thread(imu, vid);

    ImgRGB img(400, 400);
    int waiting_tm = 30;
    char ch = cv::waitKey(waiting_tm);

    int k_old = 0;
    TimeMeasurer tm;

    while (ch != 'q') {
        if (ch == 'r') {
            tm.tic();
            imu._recording = true;
            img.fill(255, 0, 0);
        }
        imshow("img", img);
        ch = cv::waitKey(waiting_tm);
        if (imu._recording) {
            drawPoint(img, 15, 15, 6, 255, 0, 0, 2);
            double ts = tm.toc() / 1000;
            if (ts > 300)
                break;
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
