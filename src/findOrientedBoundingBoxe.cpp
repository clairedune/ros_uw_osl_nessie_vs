#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <visp/vpFeatureBuilder.h>
#include <visp/vpServo.h>
#include <visp/vpSimulatorCamera.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpColVector.h>

#include <imageProcessing/colorDetection.hpp>


static const std::string TOPIC_IMG = "/camera_down_right/image_raw";
int h_min = 8 ;
int h_max = 31;
int m_e = 10 ;
int m_d = 10;

void on_trackbar( int, void* )
{
}

void imageCallBack(const sensor_msgs::ImageConstPtr& msg) 
{
	try 
	{
		// use cv_bridge to convert ros image to opencv image
		cv_bridge::CvImagePtr cv_ptr ;
		cv_ptr   = cv_bridge::toCvCopy(msg, "bgr8");
		
		// build the opencv image
		cv::Mat img;
		img   = cv_ptr->image;
 	
		// create images for temp displays
		cv::Mat imgMap = img.clone();
		cv::Mat imgMorpho = img.clone();
	    
		// detect contours of orange areas
		std::vector<std::vector<cv::Point> > contours;
		contours =  colorDetection::findColorContour( img, imgMap, imgMorpho, h_min, h_max, m_d, m_e);
   
		// detect straight bounding boxe 	
		cv::Rect bRect; // bounding rectangle of largest contour
		bRect    = colorDetection::findBoundingBoxe(contours);
		cv::RotatedRect rRect;
		rRect    = colorDetection::findOrientedBoxe(contours);    

		//visualization
		circle(img, cv::Point(0.5*(img.cols),0.5*(img.rows)), 3, cv::Scalar(255,255,355), 1, 8);
		cv::rectangle(img, bRect, cv::Scalar(0,255,0), 1, 4); // draw bRect
		for( int i = 0; i < contours.size(); i++ )
		{
			cv::drawContours(img, contours, i, cv::Scalar(50*i,50*i,0));
		}
		cv::Point2f rect_points[4]; 
		rRect.points( rect_points );
		for( int j = 0; j < 4; j++ )
			cv::line( img, rect_points[j], rect_points[(j+1)%4], cv::Scalar(255,0,0), 1, 8 );
		
		
		cv::imshow("imgMap", imgMap);
		cv::imshow("imgDilate", imgMorpho);
		cv::imshow("view", img);
		//cv::waitKey(10);

	} 
    catch (cv_bridge::Exception& e) 
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
        msg->encoding.c_str());
    }
}


int main(int argc, char **argv) {

	//init node and NH
    ros::init(argc, argv, "findOrienteoundingdBoxe");
    ros::NodeHandle nh;
	// create opencv window
	cv::namedWindow("imgMap", CV_WINDOW_NORMAL);
	cv::namedWindow("imgDilate", CV_WINDOW_NORMAL);
    cv::createTrackbar( "erode size", "imgDilate", &m_e, 180, on_trackbar);
	cv::createTrackbar( "dilate size", "imgDilate", &m_d, 180, on_trackbar);
	cv::namedWindow("view", CV_WINDOW_NORMAL);
	cv::createTrackbar( "min Hue", "imgMap", &h_min, 180, on_trackbar);
	cv::createTrackbar( "max Hue", "imgMap", &h_max, 180, on_trackbar);
    cv::startWindowThread();
    
	// create image transport instance
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(TOPIC_IMG, 1, imageCallBack);


	// ros run
	ros::Rate loop_rate(10);
    ros::spin();

    ros::shutdown();
    return 0;
}

