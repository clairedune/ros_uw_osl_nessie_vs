#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include <vector>
#include <ctime>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
/*#include <opencv2/core/core.hpp>
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
*/
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/robot/vpSimulatorCamera.h>

static const std::string TOPIC_NESSIE_IMG = "/camera_down_right/image_raw";
static const std::string TOPIC_COMMAND = "/cmd/twist";
ros::Publisher pub;
int h_min, h_max, lowThreshold;
int counter = 0;
int const max_lowThreshold = 100;

void on_trackbar_min( int, void* )
{
}
void on_trackbar_max( int, void* )
{
}
void findLargestContour(std::vector<std::vector<cv::Point> > cntr, cv::Rect& bRect)
{
	double largest_area=0.0; double a;
	for( int i = 0; i< cntr.size(); i++ )
	{
		a=contourArea( cntr[i],false);  //  Find the area of contour
		if(a>largest_area){
		largest_area=a;
		bRect=boundingRect(cntr[i]); // Find the bounding rectangle for biggest contour
		}
	}
}
void cannyDetector(cv::Mat src, cv::Rect& bRect)
{
	int ratio = 3;
	int kernel_size = 3;
	cv::Mat srcGray, srcHsv, cannyOutput;
	std::vector<std::vector<cv::Point> > contours;

	// from color to gray
	cv::cvtColor(src, srcGray, cv::COLOR_BGR2GRAY);
	// from color to hsv
	cv::cvtColor(src, srcHsv, cv::COLOR_BGR2HSV); 

	/// Reduce noise with a kernel 3x3
	cv::blur( srcGray, srcGray, cv::Size(3,3) );

	/// Canny detector
	cv::Canny( srcGray, cannyOutput, lowThreshold, lowThreshold*ratio, kernel_size );

	/// Find contours
	cv::findContours(cannyOutput, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	// Select orange contours
	cv::Mat imgMap = cv::Mat::zeros( srcGray.size(), srcGray.type() );
	bRect = cv::Rect(0,0,0,0);
	for( int i = 0; i< contours.size(); i++ )
	{
		if (cv::contourArea(contours[i]) > 0.00001*src.rows*src.cols)
		{
			bRect = cv::boundingRect(contours[i]);
			cv::inRange(srcHsv(bRect), cv::Scalar(h_min,50,50), cv::Scalar(h_max,255,255), imgMap(bRect));
		}
	}
	cv::imshow("imgDilate", imgMap);
	cv::erode(imgMap, imgMap, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
	cv::dilate(imgMap, imgMap, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	cv::findContours(imgMap.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	findLargestContour(contours, bRect);
	cv::imshow("imgCanny", imgMap);
	
}
cv::Rect coloredContour(cv::Mat src, cv::Rect& bRect)
{
	cv::Mat srcHsv, imgMap;
	std::vector<std::vector<cv::Point> > contours;

	// from color to hsv
	cv::cvtColor(src, srcHsv, cv::COLOR_BGR2HSV); 

	// Select wished color
	imgMap = cv::Mat::zeros( src.size(), src.type() );
	cv::inRange(srcHsv, cv::Scalar(h_min,150,150), cv::Scalar(h_max,255,255), imgMap);
	cv::erode(imgMap, imgMap, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
	cv::dilate(imgMap, imgMap, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15,15)));
	cv::imshow("imgSeg", imgMap);
	/// Find contours
	cv::findContours(imgMap.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	// Find largest contour
	bRect = cv::Rect(0,0,0,0);
	findLargestContour(contours, bRect);
}
void motionControl(float xi_d, float yi_d, cv::Rect bRect)
{
	int ball_size = 20000; // in pixels
	float dx, dy, dz;
	float kp, kpz; kp = 0.0005; kpz = 0.000005;
	dx = xi_d - (bRect.x + 0.5*bRect.width);
	dy = yi_d - (bRect.y + 0.5*bRect.height);
	dz = ball_size - bRect.width*bRect.height; // desired size in pixels - rectangle size

	geometry_msgs::TwistStamped ts;
	ts.twist.linear.x = kp*dy;
	ts.twist.linear.y = -kp*dx;
	ts.twist.linear.z = kpz*dz;
	pub.publish(ts);

	// to write in log file
	std::cerr<<counter<<"\t"<<clock()<<"\t"<<dx<<"\t"<<dy<<"\t"<<dz<<"\t"; //errors
	std::cerr<<ts.twist.linear.x<<"\t"<<ts.twist.linear.y<<"\t"<<ts.twist.linear.z; //command
	std::cerr << std::endl;
	// to output in commanda line only
	//std::cerr<<"vx vy: "<<ts.twist.linear.x<<" "<<ts.twist.linear.y<<std::endl;
	//std::cerr << "Rect air: "<< bRect.width*bRect.height << std::endl;
	//std::cerr<<"xd yd rx ry: "<<xi_d<<" "<<yi_d<<" "<<bRect.x + 0.5*bRect.width<<" "<<bRect.y + 0.5*bRect.height<<std::endl;
	counter++;

}
void imageCallBack(const sensor_msgs::ImageConstPtr& msg) {
	try {
	//convert ROS image to CV image
	cv::Mat img;
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	img = cv_ptr->image;
	// detect red contours
	cv::Rect bRect; // bounding rectangle of largest contour
	coloredContour(img, bRect);
	// robot motion control
	//motionControl(0.5*(img.cols),0.5*(img.rows),bRect);
	//visualization
	circle(img, cv::Point(0.5*(img.cols),0.5*(img.rows)), 10, cv::Scalar(0,255,0), 1, 8);
	cv::rectangle(img, bRect, cv::Scalar(0,255,0), 1, 4); // draw bRect
	cv::waitKey(10);
	cv::imshow("imgMap", img);
	
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                msg->encoding.c_str());
    }
}
int main(int argc, char **argv) {

	//init node and NH
    ros::init(argc, argv, "robot_cntrl");
    ros::NodeHandle nh;
	// create opencv window
	cv::namedWindow("imgMap", CV_WINDOW_NORMAL);
	cv::namedWindow("imgSeg", CV_WINDOW_NORMAL);
	h_min = 5;
	h_max = 30;
	lowThreshold = 0;
	cv::createTrackbar( "min Hue", "imgSeg", &h_min, 180, on_trackbar_min );
	cv::createTrackbar( "max Hue", "imgSeg", &h_max, 180, on_trackbar_max );
    cv::startWindowThread();
	// create image transport instance
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(TOPIC_NESSIE_IMG, 1, imageCallBack);
	// create twist publisher
	pub = nh.advertise<geometry_msgs::TwistStamped>(TOPIC_COMMAND, 1000);

	// initialize Visp
	vpHomogeneousMatrix cdMo(0, 0, 0.75, 0, 0, 0);
	vpHomogeneousMatrix cMo(0.15, -0.1, 1.,vpMath::rad(10), vpMath::rad(-10), vpMath::rad(50));

	// ros run
	ros::Rate loop_rate(10);
    ros::spin();

    ros::shutdown();
    return 0;
}

