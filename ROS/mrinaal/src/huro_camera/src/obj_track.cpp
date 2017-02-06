#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// #include <arduino_pkg/command_msg.h>
// #include <fstream>

using namespace std;
using namespace cv;

char t1, t2='\0';

char window_name[30] = "HSV Segemtation";
Mat src;
Mat Frame;
Mat hsv2;
Mat imgthreshold;
char c='\0';
int count5=0; 
int arr[10]={0,0,0,0,0,0,0,0,0,0};
// ofstream fout;			//	/dev/ttyACM2

int flag=0;
Point2f centre(-1,-1);

int minH = 0;
int maxH = 25;
int minS = 72;
int maxS = 250;
int minV = 180;
int maxV = 255;

Mat imgTmp, imgCircles;

// ros::Publisher arduino_pub = nh.advertise<arduino_pkg::command_msg>(
//     "command", 10);
// arduino_pkg::command_msg ard_msg;

void movement(int nx, int ny);
void Tracking(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Ball_Tracker");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber image_sub;
	// image_transport::Publisher image_pub_;
    
	const std::string SUBS_NAME = "cv_camera/image_raw";
    // Subscrive to input video feed and publish output video feed
    image_sub = it.subscribe(SUBS_NAME, 1, &Tracking);
    // image_pub_ = it_.advertise("image_converter/output_video", 1);

    ros::spin();

	return 0;
}

void Tracking(const sensor_msgs::ImageConstPtr& msg)
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
	
	imgTmp = cv_ptr->image;
	imgCircles = Mat::zeros( imgTmp.size(), CV_8UC3 );

	// char key = waitKey(33); 
	//if(key==27) return 0; 

	src = cv_ptr->image; 
	cvtColor(src,hsv2,CV_BGR2HSV);

	inRange(hsv2, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), imgthreshold); 
	medianBlur(imgthreshold, imgthreshold, 11);

	vector<vector<Point> > contour;
	vector<Vec4i> hierarchy;
	vector<Point> approx;
	findContours(imgthreshold, contour, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, Point(0,0));
	if(contour.size())
	{
		double maxarea = contourArea(contour[0]);
		double area;
		int maxindex=0;
		for(int i=1; i<contour.size(); i++)
		{
			area = contourArea(contour[i]);
			if(area>maxarea)
			{
				maxarea = area;
				maxindex = i;
			}
		}
		approxPolyDP(Mat(contour[maxindex]), approx, arcLength(Mat(contour[maxindex]), true)*0.02, true );
		if( fabs(contourArea(contour[maxindex])) <0.1 || !isContourConvex(approx))
		{
			flag = 0;
		}
		else	if( approx.size() >7)
					{
						float radius;
						
						minEnclosingCircle(contour[maxindex], centre, radius);
						circle(src,centre,radius,Scalar(0,255,0),2);

						flag = 1;
					}
	
	}
	else
	{
		flag=0;
	}

	movement(centre.x, centre.y);

	if(t1=='4')
		arr[4]++;
	else if(t1=='5')
				arr[5]++;
			 else if(t1=='6')
							arr[6]++;
						else if(t1=='7')
									arr[7]++;
								else if(t1=='8')
											arr[8]++;	

	if((count5 % 5)==0)
	{
		int most=0;
		for(int j=5; j<9; j++)
		{
			if(t1=='0')
			break;
			else
			{
				most=0;
				if(arr[j]>arr[most])
				{
					most=j;
				}
			}
		}

		switch(most)
		{
			case 4: t1='4';break;
			case 5: t1='5';break;
			case 6: t1='6';break;
			case 7: t1='7';break;
			case 8: t1='8';break;

		}

		for(int k=4; k<9; k++)		 
		{
			arr[k]=0;
		}
	
		for( int i=0; i<10; i++)
		{
			// fout.open("/dev/ttyACM2");
			// cerr << t1 ;
			// fout<<t1;
			// fout.close();
			// ard_msg.dir = 'o';
            // ard_msg.ang = t1;
            // arduino_pub.publish(ard_msg);
		}
	}
	count5++;

	imshow("Original",src ); //show the original image

	int c = waitKey(1);
	if( (char)c == 'q' ) { exit(0); } // escape
 
	return; 
}

void movement(int nx, int ny)
{
	if(flag)
	{
		if( nx < src.cols/3)
		{
			t1 = '4';
		}
		else if( nx > (2*src.cols)/3)
			{
				t1 = '6';
			}
			else
			{
				if( ny > (3*src.rows)/4)
				{
					t1 = '0';
				}
				else
				{
					t1 = '5';
				}
			}
	}
	else
	{
		if( nx == -1 )
		{
			t1 = '8';
		}
		else if( nx < (src.cols)/5 || nx > (4*src.cols)/5)
			{
				t1 = '8';
			}
			else if( ny > (6*src.rows)/7 )
				{
					t1 = '0';
				}
				 else
				 {
						t1 = '7';
				 }
	}
}