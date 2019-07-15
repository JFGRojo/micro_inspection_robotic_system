#include <ros/ros.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/utility.hpp"
#include <vector>
#include <string>
//#include <iostream>

static const std::string OPENCV_WINDOW = "Input";
static const std::string OPENCV_WINDOW1 = "Output";
static const std::string OPENCV_WINDOW2 = "ROI area";
static const std::string OPENCV_WINDOW3 = "Binary Image";

static const std::string IMAGE_DIR = "/home/jf/Desktop/CalibrationAcetato/CalibrationLeft2/CalLeft";

cv::Mat cv_img; // RGB Image

/*class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;


	public:
	ImageConverter()
	: it_(nh_)
	{
		// Subscribe to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/micro_s1_image", 1,& ImageConverter::imageCb, this);
		image_pub_ = it_.advertise("/micro_s1_result", 1);

		cv::namedWindow(OPENCV_WINDOW, 0);
		cv::namedWindow(OPENCV_WINDOW1, 0);
		cv::namedWindow(OPENCV_WINDOW2, 0);
		cv::namedWindow(OPENCV_WINDOW3, 0);
	}

	~ImageConverter()
	{
		cv::destroyWindow(OPENCV_WINDOW);
		cv::destroyWindow(OPENCV_WINDOW1);
		cv::destroyWindow(OPENCV_WINDOW2);
		cv::destroyWindow(OPENCV_WINDOW3);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
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

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
	}
}; //ImageConverter */

int _brightness = 100;
int _contrast = 100;

static void updateBrightnessContrast( int /*arg*/, void* )
{
    int histSize = 64;
    int brightness = _brightness - 100;
    int contrast = _contrast - 100;

    /*
     * The algorithm is by Werner D. Streidt
     * (http://visca.com/ffactory/archives/5-99/msg00021.html)
     */
    double a, b;
    if( contrast > 0 )
    {
        double delta = 127.*contrast/100;
        a = 255./(255. - delta*2);
        b = a*(brightness - delta);
    }
    else
    {
        double delta = -128.*contrast/100;
        a = (256.-delta*2)/255.;
        b = a*brightness + delta;
    }

    cv::Mat dst, hist;
    cv_img.convertTo(dst, CV_8U, a, b);
    cv::imshow("image", dst);

    cv::calcHist(&dst, 1, 0, cv::Mat(), hist, 1, &histSize, 0);
    cv::Mat histImage = cv::Mat::ones(200, 320, CV_8U)*255;

    cv::normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, CV_32F);

    histImage = cv::Scalar::all(255);
    int binW = cvRound((double)histImage.cols/histSize);

    for( int i = 0; i < histSize; i++ )
        cv::rectangle( histImage, cv::Point(i*binW, histImage.rows),
                   cv::Point((i+1)*binW, histImage.rows - cvRound(hist.at<float>(i))),
                   cv::Scalar::all(0), -1, 8, 0 );
    cv::imshow("histogram", histImage);
}

/// Global variables

int threshold_value = 0;
int threshold_type = 3;;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;

cv::Mat src, th_dst;
static const std::string window_name = "Threshold Demo";

static const std::string trackbar_type = "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
static const std::string trackbar_value = "Value";

/**
 * @function Threshold_Demo
 */
void Threshold_Demo( int, void* )
{
  /* 0: Binary
     1: Binary Inverted
     2: Threshold Truncated
     3: Threshold to Zero
     4: Threshold to Zero Inverted
   */

  threshold(src, th_dst, threshold_value, max_BINARY_value,threshold_type );

  cv::imshow( window_name, th_dst );
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "micro_s1_processing_node");
	//ros::NodeHandle n; //needed to use ros::Rate, ros::spin and similars.. else use ros::Time::init()
	ros::Time::init();
	/*ImageConverter ic;
	ros::Subscriber sub = n.subscribe("chatter", 1,chatterCallback);
	ros::spin();*/
	ros::Rate loop_rate(10);
	//cv::Mat cv_img; // RGB Image
	cv::Mat cv_gray; // Gray Image
	cv::Mat cv_bn; // binary Image
	cv::Mat cv_out; // Output Image

	//cv_calib.setTo(cv::Scalar(255,255,255));
	std::vector<std::vector<cv::Point> > c; // Contours
	std::vector<cv::Vec4i> h; // Hierarchy
	cv::Moments m;
	cv::Point cm;

	int idx=0; // Index to run contours' vector

	// Filter parameters
	int w_threshold_max = 350; //50 celeg
	int h_threshold_max = 350;
	int w_threshold_min = 200; //15 celeg
	int h_threshold_min = 200;
	const float area_max = 999999.0; //170.0 celeg
	const float area_min = 30000.0; //100	celeg

	for(int i=0; i<5;i++)
	{
		std::string img_in_dir = IMAGE_DIR;
		img_in_dir += std::to_string(i);
		img_in_dir +=".bmp";
		std::string img_out_dir = "/home/jf/Desktop/Outputs/micro_s1_out";
		img_out_dir += std::to_string(i);
		img_out_dir +=".bmp";
		std::cout << "Entrada: " << img_in_dir << std::endl;
		std::cout << "Salida: " << img_out_dir << std::endl;
		//cv_img = cv::imread(img_in_dir,CV_LOAD_IMAGE_COLOR);
		cv_img = cv::imread(img_in_dir,0);
		if(cv_img.empty())
    {
        std::cerr << "Cannot read image file: " << img_in_dir << std::endl;
        return -1;
    }
		src = cv_img.clone();

		cv::namedWindow("image", 0);
		cv::namedWindow("histogram", 0);
		cv::namedWindow( window_name, 0 );//CV_WINDOW_AUTOSIZE

		cv::createTrackbar("brightness", "image", &_brightness, 200, updateBrightnessContrast);
		cv::createTrackbar("contrast", "image", &_contrast, 200, updateBrightnessContrast);

		cv::createTrackbar( trackbar_type, window_name, &threshold_type, max_type, Threshold_Demo );

		cv::createTrackbar( trackbar_value, window_name, &threshold_value, max_value, Threshold_Demo );

		updateBrightnessContrast(0, 0);
		Threshold_Demo( 0, 0 );

		cv::waitKey();

		/*cv::cvtColor(cv_img, cv_gray, cv::COLOR_BGR2GRAY);	// RGB to GRAY image copy
		cv::medianBlur(cv_gray,cv_gray,11);
		cv::adaptiveThreshold(cv_gray,cv_bn,255,CV_ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY,851,0); //55,2//11,2//11,0
		cv_out=cv_img.clone();
		cv_out.setTo(cv::Scalar(255,255,255));
		cv::bitwise_not ( cv_bn, cv_bn );
		cv::findContours(cv_bn,c,h,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
		int numContours = c.size() - 1;
		std::cout << "Number of objects: " << numContours << std::endl;
		std::vector<cv::Vec4i>::iterator ith = h.begin();

		for(std::vector<std::vector<cv::Point> >::iterator it = c.begin(); it != c.end(); )
		{
			cv::Rect R = boundingRect(*it);
			int area = contourArea(*it,false);

			//if ((area > area_max) || (area < area_min))
			//{
			if ((R.width > w_threshold_max || R.height > h_threshold_max) || (R.width < w_threshold_min || R.height < h_threshold_min) || ((area > area_max) || (area < area_min)))
			{
				it = c.erase(it);
				ith = h.erase(ith);
			}
			//}
			else
			{
				it ++;
				ith ++;
			}
		}
		//int numTargets = c.size();
		int numTargets = c.size();
		std::cout << "Number of targets: " << numTargets << std::endl;
		int numH = h.size();
		std::cout << "Number of hierarchies: " << numH << std::endl;
		//for(idx=0;idx >=0; idx = h[idx][0]) {
		if (c.size() > 0)
		{
			for(idx=0;idx <numTargets; idx ++)
			{
				cv::Rect R_contours = cv::boundingRect( c[idx]);
				cv::rectangle(cv_img, R_contours.tl(), R_contours.br(),CV_RGB(255, 0, 0),3,8,0);
				m = cv::moments(c[idx], false);
				cm = cv::Point((int)(m.m10/m.m00),(int)(m.m01/m.m00));
				// std::string x = std::toString(cm.x);
				// std::string y = std::toString(cm.y);
				// std::string centroid = "(" + x + "," + y + ")";
				// stringstream converter;
				// converter.str("");
				// converter << contourArea(c[idx],false)

				std::string c_area = std::to_string(idx);// + " " + to_string(contourArea(c[idx],false));
				cv::putText(cv_img,c_area,R_contours.br(),cv::FONT_HERSHEY_PLAIN,5,CV_RGB(255, 0, 0),3,8);
				cv::drawContours(cv_img,c,idx,cv::Scalar (rand()%255, rand()%255, rand()%255),CV_FILLED,cv::LINE_8,h,1);
				std::cout << idx << " " << contourArea(c[idx],false) << " " << R_contours.br().x-R_contours.tl().x << "," << R_contours.br().y-R_contours.tl().y << std::endl;
				cv::drawContours(cv_out,c,idx,cv::Scalar (0, 0, 0),CV_FILLED,cv::LINE_8,h,1);
			}
		}

		cv::namedWindow(OPENCV_WINDOW,cv::WINDOW_NORMAL);
		cv::resizeWindow(OPENCV_WINDOW,1000,1000);
		cv::imshow(OPENCV_WINDOW, cv_img);
		cv::moveWindow(OPENCV_WINDOW, 1920/2-1000, 1080/2-400);
		cv::namedWindow(OPENCV_WINDOW1,cv::WINDOW_NORMAL);
		cv::resizeWindow(OPENCV_WINDOW1,1000,1000);
		cv::imshow(OPENCV_WINDOW1, cv_out);
		cv::moveWindow(OPENCV_WINDOW1, 1920/2-100, 1080/2-400);
		cv::waitKey(3);
		cv::imwrite(img_out_dir,cv_out);*/
		loop_rate.sleep();
	}
	return 0;
}
