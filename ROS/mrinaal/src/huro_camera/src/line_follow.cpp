#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
// #include <std_msgs/Char.h>
// #include <arduino_pkg/command_msg.h>
// #include <fstream>

using namespace std;
using namespace cv;

int h_min =10;
int s_min = 15;
int v_min = 20;
int h_max = 60;
int s_max = 60;
int v_max = 40;
// ros::Publisher arduino_pub = nh.advertise<arduino_pkg::command_msg>(
//     "command", 10);

int bias = 0;

int c=1;
Mat cameraFeed, hsv, mask, temp;
vector< vector<Point> > contours;
vector<Vec4i> hierarchy;

Moments moment;
int x=0,y=0;
bool track = false;

double minMaxCx = (bias > 0 ?  1000000 : -1000000);

// arduino_pkg::command_msg ard_msg;

void Following(const sensor_msgs::ImageConstPtr& msg);

void createTrackbars() {
    namedWindow("Trackbars",0);
    createTrackbar("H_MIN", "Trackbars", &h_min, 255);
    createTrackbar("H_MAX", "Trackbars", &h_max, 255);
    createTrackbar("S_MIN", "Trackbars", &s_min, 255);
    createTrackbar("S_MAX", "Trackbars", &s_max, 255);
    createTrackbar("V_MIN", "Trackbars", &v_min, 255);
    createTrackbar("V_MAX", "Trackbars", &v_max, 255);

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Line_Follower");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber image_sub;
	// image_transport::Publisher image_pub_;
    
	const std::string SUBS_NAME = "cv_camera/image_raw";
    // Subscrive to input video feed and publish output video feed
    image_sub = it.subscribe(SUBS_NAME, 1, &Following);
    // image_pub_ = it_.advertise("image_converter/output_video", 1);

    ros::spin();

	return 0;
}

void Following(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cameraFeed=cv_ptr->image;
    // cvtColor(cameraFeed, hsv, COLOR_BGR2HSV);
    cameraFeed.copyTo(hsv);
    //Mat roi = hsv;
    Mat roi(hsv, Rect(10, 2*hsv.rows/3, hsv.cols-20, hsv.rows/12));
    //Mat roi = hsv(Rect(10, 2*hsv.rows/3, hsv.cols-20, hsv.rows/12));
   
    inRange(roi, Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max), mask);
    medianBlur(mask, mask, 11);
    dilate(mask, mask, getStructuringElement(MORPH_RECT, Size(5,5)));
    mask.copyTo(temp);
    findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);
    
    imshow("a",roi);
    if(track && contours.size() > 0)
    {
        int j, maxindex, maxarea=0, area;
        for(j=0; j< contours.size(); j++)
        {
            moment = moments((Mat)contours.at(j));
            area= moment.m00;
            if(area>maxarea)
            {
                maxarea = area;
                maxindex = j;
            }
        }
        moment = moments((Mat)contours.at(maxindex));

        if (moment.m00 > 0.0)
        {
            Rect r = boundingRect(contours.at(maxindex));
            double cx;
            // if (bias > 0)
            // {
                cx = r.x ;//+ r.width - 12;
                if (cx > minMaxCx)
                {
                    minMaxCx = cx;
                }
            // }
            // else
            // {
                // cx = r.x + 12;
                if (minMaxCx > cx)
                {
                    minMaxCx = cx;
                }
            // }
        }

        if (minMaxCx==1000000||minMaxCx==-1000000)
            minMaxCx = roi.cols/2;
        float t= (1.0f - 2.0f*(float)(minMaxCx/roi.cols))*1000;
        char ch;
        if(t>-0.20&&t<0.20)
            ch='f';
        else if(t<-0.20)
            ch='r';
        else if(t>0.20)
            ch='l';
        int count5=0;
        if(count5%100==0)
        {
            cerr<<ch;
            // f.open("/dev/ttyACM0");
            // f<<ch;
            // f.close();
            // ard_msg.dir = ch;
            // ard_msg.ang = 5;
            // arduino_pub.publish(ard_msg);
        }
        count5++;
    }

    rectangle(cameraFeed, Rect(10, 2*hsv.rows/3, hsv.cols-20, hsv.rows/12), Scalar(172,255,135), 1, 8, 0 );
    
    imshow("Camera", mask);
    imshow("CameraFeed", cameraFeed);
    
    char chr;
    char ch = waitKey(1);
    if (ch == 'q')
    {
        // chr='s';
        // f.open("/dev/ttyACM0");
        // f<<chr;
        // f.close();
        exit(0);

    }
    if (ch == 't')
        track = !track;

    return;
}