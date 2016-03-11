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


static const std::string TOPIC_IMG = "/uwsim/camera1";
//static const std::string TOPIC_COMMAND = "/cmd/twist";
static const std::string TOPIC_COMMAND = "/nessie/twist";

ros::Publisher pub;
int h_min, h_max;

// for the visual servoing
vpServo task;
vpFeaturePoint pc[4], pd[4]; 
bool initTask = false;
vpCameraParameters cam;
int counter = 0;


void on_trackbar_min( int, void* )
{
}
void on_trackbar_max( int, void* )
{
}
void findLargestContour(const std::vector<std::vector<cv::Point> > &cntr, cv::Rect& bRect)
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


/*
* author : Claire Dune
* date   : 11/03/2016
*
*/
cv::RotatedRect findLargestOrientedBoxe(const std::vector<std::vector<cv::Point> > &contours)
{
  /// Find the rotated rectangles and ellipses for each contour
  cv::RotatedRect maxRect;
  double largest_area=0.0; 

  for( int i = 0; i < contours.size(); i++ )
     { 
       int area = cv::contourArea(contours[i],false);
       if( area > largest_area )
	{
		largest_area=area;
		maxRect = cv::minAreaRect( cv::Mat(contours[i]) );
     	}
     }
 return maxRect;
}


cv::Rect coloredContour(cv::Mat src, cv::Rect& bRect)
{
	cv::Mat srcHsv, imgMap;
	std::vector<std::vector<cv::Point> > contours;

	// from color to hsv
	cv::cvtColor(src, srcHsv, cv::COLOR_BGR2HSV); 

	// Select wished color
	imgMap = cv::Mat::zeros( src.size(), src.type() );
	cv::inRange(srcHsv, cv::Scalar(h_min,50,50), cv::Scalar(h_max,255,255), imgMap);
	cv::imshow("imgMap", imgMap);
	cv::erode(imgMap, imgMap, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
	cv::dilate(imgMap, imgMap, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15,15)));
	cv::imshow("imgDilate", imgMap);

	/// Find contours
	cv::findContours(imgMap.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	// Find largest contour
	bRect = cv::Rect(0,0,0,0);
	findLargestContour(contours, bRect);
}



void motionControl(float xi_d, float yi_d, cv::Rect bRect)
{
	float dx, dy;
	float kp; kp = 0.005;
	dx = xi_d - (bRect.x + 0.5*bRect.width);
	dy = yi_d - (bRect.y + 0.5*bRect.height);

	geometry_msgs::TwistStamped ts;
	ts.twist.linear.x = kp*dy;
	ts.twist.linear.y = -kp*dx;
	pub.publish(ts);

	std::cout<<"vx vy: "<<ts.twist.linear.x<<" "<<ts.twist.linear.y<<std::endl;
	std::cout<<"xd yd rx ry: "<<xi_d<<" "<<yi_d<<" "<<bRect.x + 0.5*bRect.width<<" "<<bRect.y + 0.5*bRect.height<<std::endl;

}
/*

4 points visual servoing

*/
vpColVector visp4PointsServo(   cv::Rect bRect, // opencv detected red rectangle
                                vpPoseVector posed, // desired object pose
                                double lambda,  // control gain
                                bool& init // do we have to init the task ?
							)
{
   
    // desired pose as an homogeneous matrix
    vpHomogeneousMatrix cdMo(posed);
    vpPoseVector pose (posed);
    
    if(!init) // do it one for all
    {

 	// camera parameters
    	double px = 257.34; // pixel width
    	double py = 257.34; // pixel high
    	double u0 = 160; // optical center shift in u
    	double v0 = 120; // optical center shift in v
    	// Camera initialization with a perspective projection without distortion model
    	cam.initPersProjWithoutDistortion(px,py,u0,v0);

        // construct the 3D object
        vpPoint point[4] ;
        float width(0.20), height(0.12);
        point[0].setWorldCoordinates( 0 , 0 , 0);
        point[1].setWorldCoordinates( width ,0, 0);
        point[2].setWorldCoordinates( width, height, 0);
        point[3].setWorldCoordinates( 0 , height, 0);
 
       //vpServo task ;
        task.setServo(vpServo::EYEINHAND_CAMERA);
        task.setInteractionMatrixType(vpServo::CURRENT);
        task.setLambda(lambda);
	 
        //  init the desired features
        for (unsigned int i = 0 ; i < 4 ; i++) {
		  pd[i].set_x(0); pd[i].set_y(0);
          pc[i].set_x(0); pc[i].set_y(0);
          
        }

        // project the 3D points to the desired position 
        for (unsigned int i = 0 ; i < 4 ; i++) {
            point[i].track(cdMo);
            vpFeatureBuilder::create(pd[i], point[i]); // copy xd,yd and Zd
			
        }
	
        // read the current features and convert them to meters
        double xtemp, ytemp, utemp, vtemp;

	utemp = bRect.x; 				
	vtemp = bRect.y;
	vpPixelMeterConversion::convertPoint (cam, utemp, vtemp, xtemp, ytemp);
        pc[0].buildFrom(xtemp,ytemp,1);

	utemp = bRect.x + bRect.width; 	
	vtemp = bRect.y;
        vpPixelMeterConversion::convertPoint (cam, utemp, vtemp, xtemp, ytemp);
        pc[1].buildFrom(xtemp,ytemp,1);
        
	utemp = bRect.x + bRect.width; 	
	vtemp = bRect.y + bRect.height;
        vpPixelMeterConversion::convertPoint (cam, utemp, vtemp, xtemp, ytemp);
        pc[2].buildFrom(xtemp,ytemp,1);
	    
	utemp = bRect.x; 				
	vtemp = bRect.y + bRect.height;
        vpPixelMeterConversion::convertPoint (cam, utemp, vtemp, xtemp, ytemp);
        pc[3].buildFrom(xtemp,ytemp,1);	

	// add features (points projection) to the task
        for (unsigned int i = 0 ; i < 4 ; i++) {
            task.addFeature(pc[i], pd[i]);
        }

	// turn the init flag to true
	init=true; 
  }
    
  else // if we do not want to init just updtade current p
  {
	std::cerr << "\t x*1 " << pd[0].get_x()<< "\t y*1 "<< pd[0].get_y()<< std::endl;
	std::cerr << "\t x*2 " << pd[1].get_x()<< "\t y*2 "<< pd[1].get_y()<< std::endl;
	std::cerr << "\t x*3 " << pd[2].get_x()<< "\t y*3 "<< pd[2].get_y()<< std::endl;
	std::cerr << "\t x*4 " << pd[3].get_x()<< "\t y*4 "<< pd[3].get_y()<< std::endl;
        std::cerr << "\t x1 " << pc[0].get_x()<< "\t y1 "<< pc[0].get_y()<< std::endl;
	std::cerr << "\t x2 " << pc[1].get_x()<< "\t y2 "<< pc[1].get_y()<< std::endl;
	std::cerr << "\t x3 " << pc[2].get_x()<< "\t y3 "<< pc[2].get_y()<< std::endl;
	std::cerr << "\t x4 " << pc[3].get_x()<< "\t y4 "<< pc[3].get_y()<< std::endl;
    
    
    double xtemp, ytemp, utemp, vtemp;
	
	// P1
	utemp = bRect.x; 				
	vtemp = bRect.y;
	vpPixelMeterConversion::convertPoint (cam, utemp, vtemp, xtemp, ytemp);
        pc[0].buildFrom(xtemp,ytemp,1);

	
	//P2
	utemp = bRect.x + bRect.width; 	
	vtemp = bRect.y;
    	vpPixelMeterConversion::convertPoint (cam, utemp, vtemp, xtemp, ytemp);
    	pc[1].buildFrom(xtemp,ytemp,1);
        
	//P3
	utemp = bRect.x + bRect.width; 	
	vtemp = bRect.y + bRect.height;
    	vpPixelMeterConversion::convertPoint (cam, utemp, vtemp, xtemp, ytemp);
    	pc[2].buildFrom(xtemp,ytemp,1);


	
	//P4    
	utemp = bRect.x; 				
	vtemp = bRect.y + bRect.height;
    	vpPixelMeterConversion::convertPoint (cam, utemp, vtemp, xtemp, ytemp);
    	pc[3].buildFrom(xtemp,ytemp,1);	
  }
    
  vpColVector v = task.computeControlLaw();
  return v;
}

void motionControlVisp(double xd, double yd, double zd, double rolld, double pitchd, double yawd, double lambda,cv::Rect bRect)
{
  vpPoseVector posed(xd, yd, zd, rolld, pitchd, yawd);
  vpColVector vc;	
  vc = visp4PointsServo(bRect, posed, lambda, initTask);
  // From camera to robot frame
  vpPoseVector rpc(0,0,0,0,0,M_PI/2);
  vpHomogeneousMatrix rMc(rpc);
  vpVelocityTwistMatrix rVc(rMc);
	
  vpColVector vr;	
  
  //vc[0] = 0;
  //vc[1] = 0;
  //vc[2] = 1;
  //vc[3] = 0;
  //vc[4] = 0;
  //vc[5] = 0;
       
  vr = rVc*vc;     
       
  std::cerr << counter << std::endl ;
  std::cerr <<" v-->> " << vc.transpose() << std::endl;
  std::cerr << " vr-->> " << vr.transpose()<< std::endl;  
  std::cerr << " error-->> "<<task.getError().transpose() <<std::endl;   
  counter++;
  geometry_msgs::TwistStamped ts;
  ts.twist.linear.x  = vr[0];
  ts.twist.linear.y  = vr[1];
  ts.twist.linear.z  = vr[2];
  ts.twist.angular.x = vr[3];
  ts.twist.angular.y = vr[4];
  ts.twist.angular.z = vr[5];
  pub.publish(ts);
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

	motionControlVisp(0, 0.2, 1, 0, 0, 0, 0.8, bRect);


    double xtemp, ytemp, utemp, vtemp;
    for (int i =0 ; i<4 ; i++){
		xtemp = pd[i].get_x(); ytemp = pd[i].get_y();		
    	vpMeterPixelConversion::convertPoint(cam, xtemp, ytemp, utemp, vtemp);  
		//std::cerr << "\t" << xtemp<< "\t"<<ytemp;  
		circle(img, cv::Point(floor(utemp),floor(vtemp)), 7, cv::Scalar(0,0,255), 1, 8);
		xtemp = pc[i].get_x(); ytemp = pc[i].get_y();		
    	vpMeterPixelConversion::convertPoint(cam, xtemp, ytemp, utemp, vtemp);  
		std::cerr << "\t"<< pd[i].get_x()-pc[i].get_x()<< "\t"<<pd[i].get_y()-pc[i].get_y();  
		circle(img, cv::Point(floor(utemp),floor(vtemp)), 5, cv::Scalar(0,255,255), 1, 8);
		}
	 
    std::cerr << std::endl;

    //circle(img, cv::Point(floor(utemp),floor(vtemp)), 10, cv::Scalar(0,0,255), 1, 8);
	//visualization
	circle(img, cv::Point(0.5*(img.cols),0.5*(img.rows)), 10, cv::Scalar(0,255,0), 1, 8);
	cv::rectangle(img, bRect, cv::Scalar(0,255,0), 1, 4); // draw bRect
	cv::imshow("view", img);
	cv::waitKey(10);

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
	cv::namedWindow("imgDilate", CV_WINDOW_NORMAL);
	cv::namedWindow("view", CV_WINDOW_NORMAL);
	h_min = 170;
	h_max = 180;
	cv::createTrackbar( "min Hue", "imgMap", &h_min, 180, on_trackbar_min );
	cv::createTrackbar( "max Hue", "imgMap", &h_max, 180, on_trackbar_max );
    cv::startWindowThread();
	// create image transport instance
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(TOPIC_IMG, 1, imageCallBack);
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
/*Message position_req
---
header: 
  seq: 1
  stamp: 
    secs: 0
    nsecs: 0
  frame_id: ''
priority: 0
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
limit_velocity: [0, 0, 0, 0, 0, 0]
disable_axis: [0, 0, 0, 0, 0, 0]
---
*/

