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
static const std::string OPENCV_WINDOW1 = "Binary";
static const std::string OPENCV_WINDOW2 = "Binary2";
static const std::string OPENCV_WINDOW3 = "Gray Image";

static const std::string IMAGE_DIR = "/home/jf/Desktop/CalibrationAcetato/CalibrationLeft2/CalLeft";

class ImageConverter
{
	ros::NodeHandle nh_;
	ros::NodeHandle nh2_;
	ros::NodeHandle nh3_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	//ros::Subscriber image_sub_;
	ros::Publisher command_pub_;
	ros::Publisher command3_pub_;
	private:

		bool preproc;
		bool prepare;

	public:
	ImageConverter()
	: it_(nh_)
	{
		// Subscribe to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/micro_s1_image_unc", 1,& ImageConverter::imageCb, this);
		//image_sub_ = nh2_.subscribe("/micro_s1_image/compressed", 1,& ImageConverter::imageCb, this);
		command_pub_ = nh2_.advertise<std_msgs::String>("/micro_s1_result", 1);
		command3_pub_ = nh3_.advertise<std_msgs::String>("/teensy_lighting", 1);

		preproc = true;
		prepare = true;

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
		if (prepare)
		{
			std_msgs::String marlin;
			marlin.data = "G28\n";
			command_pub_.publish(marlin);
			cv::waitKey(0);
			std_msgs::String test;
			test.data = "100";
			command3_pub_.publish(test);
			cv::waitKey(0);
			test.data = "k";
			command3_pub_.publish(test);
			cv::waitKey(0);
			test.data = "w";
			command3_pub_.publish(test);
			cv::waitKey(0);
			marlin.data = "G0 X21 Z21\n";
			command_pub_.publish(marlin);
			cv::waitKey(0);
			prepare=false;
			return;
		}
		cv_bridge::CvImagePtr cv_ptr; //sensor_msgs Image
		cv::Mat cv_img; // RGB Image
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			cv_img = cv_ptr->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

		//Variable Declaration
		cv::Mat cv_gray; // Gray Image
		cv::Mat cv_bn; // binary Image
		cv::Mat cv_bn2; // binary Image
		cv::Mat cv_out; // Output Image
		cv::Mat cv_roi; // ROI Image

		cv::Moments m;
		cv::Point cm;

		cv::Point distance;
		cv::Point distance_dt;//(9999,9999);
			distance_dt.x = 9000; //dist max inicial
			distance_dt.y = 9000;

		cv::Point motion_distance;
		cv::Point target_cm;
		cv::Point target_cm_dt;
		cv::Point ROI_Origin;

		cv::Rect target_rect;
		cv::Rect ROI;

		int thresh = 67;

		int idx=0; // Index to run contours vector
		int target_idx=0;

		int ROI_W;
		int ROI_H;

		// Filter parameters
		int w_threshold_max = 350; //50 celeg
		int h_threshold_max = 350;
		int w_threshold_min = 160; //15 celeg
		int h_threshold_min = 160;
		int max_targets = 15;
		const float area_max = 999999.0; //170.0 celeg
		const float area_min = 23000.0; //100	celeg

		float motion_x = 0.0;
		float motion_y = 0.0;
		float motion_max = 200.0;

		std::string motion;

		if (preproc)
		{
			ROS_INFO("Preprocessing image");

			//Image binarization
			cv::cvtColor(cv_img, cv_gray, cv::COLOR_BGR2GRAY);	// RGB to GRAY image copy
			cv::medianBlur(cv_gray,cv_gray,11);
			cv::adaptiveThreshold(cv_gray,cv_bn,255,CV_ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY,851,0); //55,2//11,2//11,0
			cv::bitwise_not ( cv_bn, cv_bn );
			cv::imshow(OPENCV_WINDOW1, cv_bn);
			cv::imshow(OPENCV_WINDOW3, cv_gray);
			cv::moveWindow(OPENCV_WINDOW1, 1920/2-480, 1080/2-540);
			cv::threshold (cv_gray, cv_bn2, thresh, 255,cv::THRESH_BINARY_INV);
			cv::imshow(OPENCV_WINDOW2, cv_bn2);
			cv::moveWindow(OPENCV_WINDOW2, 1920/2, 1080/2-540);
			cv::moveWindow(OPENCV_WINDOW, 1920/2-960, 1080/2-540);
			//cv::waitKey(0);

			//Image Segmentation
			std::vector<std::vector<cv::Point> > c; // Contours
			std::vector<cv::Vec4i> h; // Hierarchy
			cv::findContours(cv_bn2,c,h,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
			int numContours = c.size() - 1;
			std::cout << "Number of objects: " << numContours << std::endl;

			//Image Labelling
			std::vector<cv::Vec4i>::iterator ith = h.begin();
			for(std::vector<std::vector<cv::Point> >::iterator it = c.begin(); it != c.end(); )
			{
				cv::Rect R = boundingRect(*it);
				int area = contourArea(*it,false);

				if ((R.width > w_threshold_max || R.height > h_threshold_max) || (R.width < w_threshold_min || R.height < h_threshold_min) || ((area > area_max) || (area < area_min)))
				{
					it = c.erase(it);
					ith = h.erase(ith);
				}
				else
				{
					it ++;
					ith ++;
				}
			}

			if (c.size() > 0)
			{
				std::cout << "Number of targets: " << c.size() << std::endl;
				std::cout << "Number of hierarchies: " << h.size() << std::endl;
			}
			else
			{
				std::cout << "No Targets Found" << std::endl;
			}

			if (c.size() <= max_targets)
			{
				for(idx=0; idx <c.size(); idx ++)
				{
					cv::Rect R_contours = cv::boundingRect( c[idx]);
					cv::rectangle(cv_img, R_contours.tl(), R_contours.br(),CV_RGB(255, 0, 0),3,8,0);
					m = cv::moments(c[idx], false);
					cm = cv::Point((int)(m.m10/m.m00),(int)(m.m01/m.m00));
					std::cout << "cm " << idx << " -> [" << cm.x << "," << cm.y << "] A: " << contourArea(c[idx],false) << " W: " << R_contours.br().x-R_contours.tl().x << ", H: " << R_contours.br().y-R_contours.tl().y << std::endl;
					std::string c_area = std::to_string(idx);// + " " + to_string(contourArea(c[idx],false));
					cv::putText(cv_img,c_area,R_contours.br(),cv::FONT_HERSHEY_PLAIN,5,CV_RGB(255, 0, 0),3,8);
					cv::drawMarker(cv_img,cm,CV_RGB(0,0,255),cv::MARKER_CROSS,50,5,8);
				}
			}
			else
			{
				std::cout << "Too many Targets" << std::endl;
			}

			//Fixing main Target
			for(idx=0; idx <c.size(); idx ++)
			{
				m = cv::moments(c[idx], false);
				cm = cv::Point((int)(m.m10/m.m00),(int)(m.m01/m.m00));
				distance.x = abs(cm.x - cv_img.cols/2);
				distance.y = abs(cm.y - cv_img.rows/2);
				std::cout << "cm " << idx << " -> [" << cm.x << "," << cm.y << "] A: " << contourArea(c[idx],false) << " Dist. X: " << distance.x << ", Dist. Y: " << distance.y << std::endl;

				if (sqrt((distance.x * distance.x) + (distance.y * distance.y)) < sqrt((distance_dt.x * distance_dt.x) + (distance_dt.y * distance_dt.y)))
				{
					target_cm.x = cm.x;
					target_cm.y = cm.y;
					target_idx = idx; //index of closest object found to acess to it's contour info.
					distance_dt.x = distance.x; //Distance actualitation to compare the next object found
					distance_dt.y = distance.y;
					target_rect = cv::boundingRect( c[idx]);
					//std::cout << "cm " << target_idx << " -> [" << target_cm.x << "," << target_cm.y << "] A: " << contourArea(c[target_idx],false) << " Dist. X: " << distance_dt.x << ", Dist. Y: " << distance_dt.y << std::endl;
				}
			}
			distance_dt.x = 9000;
			distance_dt.y = 9000;
			cv::drawContours(cv_img,c,target_idx,cv::Scalar(rand()%255, rand()%255, rand()%255),CV_FILLED,cv::LINE_8,h,1);
			cv::drawMarker(cv_img,cv::Point((cv_img.cols/2),(cv_img.rows/2)),CV_RGB(10,200,30),cv::MARKER_CROSS,100,5,8);
			cv::drawMarker(cv_img,target_cm,cv::Scalar(255,255,255),cv::MARKER_CROSS,50,5,8);
			cv::line(cv_img,cv::Point((cv_img.cols/2),(cv_img.rows/2)),target_cm,cv::Scalar(rand()%255, rand()%255, rand()%255),5,cv::LINE_8,0);

			// Saving last target position
			target_cm_dt = target_cm;

			//Target ROI
			ROI_W = (target_cm.x-cv_img.cols/2);
			if (ROI_W > 0)
			{
				ROI_W += 2*abs(target_rect.tl().x-target_rect.br().x);
				ROI_Origin.x = (cv_img.cols/2) - abs(target_rect.tl().x-target_rect.br().x);
			}
			else if (ROI_W < 0)
			{
				ROI_W = abs(ROI_W + 2*(target_rect.tl().x-target_rect.br().x));
				ROI_Origin.x = target_cm.x - abs(target_rect.tl().x-target_rect.br().x);
			}
			else
			{
				std::cout << "dimensional Width error" << std::endl;
				//return 1;
			}

			ROI_H = (target_cm.y-cv_img.rows/2);
			if (ROI_H > 0)
			{
				ROI_H += 2*abs(target_rect.tl().y-target_rect.br().y);
				ROI_Origin.y = (cv_img.rows/2) - abs(target_rect.tl().y-target_rect.br().y);
			}
			else if (ROI_H < 0)
			{
				ROI_H = abs(ROI_H + 2*(target_rect.tl().y-target_rect.br().y));
				ROI_Origin.y = target_cm.y - abs(target_rect.tl().y-target_rect.br().y);
			}
			else
			{
				std::cout << "dimensional Heigth error" << std::endl;
				//return 1;
			}
			ROI.x = ROI_Origin.x;
			ROI.y = ROI_Origin.y;
			ROI.width = ROI_W;
			ROI.height = ROI_H;
			cv::rectangle(cv_img, ROI.tl(), ROI.br(),CV_RGB(0, 0, 0),3,8,0);
			cv::imshow(OPENCV_WINDOW, cv_img);
			cv::waitKey(3);

			preproc = false;
			ROS_INFO("Target fixed");
		} //END OF PREPROC

		//Setting ROI
		cv_roi = cv_img(ROI);
		//Image binarization
		cv::cvtColor(cv_roi, cv_gray, cv::COLOR_BGR2GRAY);	// RGB to GRAY image copy
		cv::medianBlur(cv_gray,cv_gray,11);
		cv::adaptiveThreshold(cv_gray,cv_bn,255,CV_ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY,851,0); //55,2//11,2//11,0
		cv::bitwise_not ( cv_bn, cv_bn );
		cv::imshow(OPENCV_WINDOW1, cv_bn);
		cv::moveWindow(OPENCV_WINDOW1, 1920/2-480, 1080/2-540);
		cv::threshold (cv_gray, cv_bn2, thresh, 255,cv::THRESH_BINARY_INV);
		cv::imshow(OPENCV_WINDOW2, cv_bn2);
		cv::moveWindow(OPENCV_WINDOW2, 1920/2, 1080/2-540);
		cv::rectangle(cv_img, ROI.tl(), ROI.br(),CV_RGB(0, 0, 0),3,8,0);
		cv::imshow(OPENCV_WINDOW, cv_img);
		cv::moveWindow(OPENCV_WINDOW, 1920/2-960, 1080/2-540);
		cv::waitKey(3);

		//Image Segmentation
		std::vector<std::vector<cv::Point> > c; // Contours
		std::vector<cv::Vec4i> h; // Hierarchy
		cv::findContours(cv_bn2,c,h,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
		int numContours = c.size() - 1;
		std::cout << "Number of objects: " << numContours << std::endl;

		//Image Labelling
		std::vector<cv::Vec4i>::iterator ith = h.begin();
		for(std::vector<std::vector<cv::Point> >::iterator it = c.begin(); it != c.end(); )
		{
			cv::Rect R = boundingRect(*it);
			int area = contourArea(*it,false);

			if ((R.width > w_threshold_max || R.height > h_threshold_max) || (R.width < w_threshold_min || R.height < h_threshold_min) || ((area > area_max) || (area < area_min)))
			{
				it = c.erase(it);
				ith = h.erase(ith);
			}
			else
			{
				it ++;
				ith ++;
			}
		}

		if (c.size() > 0)
		{
			std::cout << "Number of targets: " << c.size() << std::endl;
			std::cout << "Number of hierarchies: " << h.size() << std::endl;
		}
		else
		{
			std::cout << "No Targets Found" << std::endl;
			preproc = true;
			motion_x = 0.0;
			motion_y = 0.0;
		}

		if (c.size() <= max_targets)
		{
			for(idx=0; idx < c.size(); idx ++)
			{
				cv::Rect R_contours = cv::boundingRect( c[idx]);
				cv::rectangle(cv_img, R_contours.tl()+ROI_Origin, R_contours.br()+ROI_Origin,CV_RGB(255, 0, 0),3,8,0);
				m = cv::moments(c[idx], false);
				cm = cv::Point((int)(m.m10/m.m00),(int)(m.m01/m.m00));
				std::cout << "cm " << idx << " -> [" << cm.x << "," << cm.y << "] A: " << contourArea(c[idx],false) << " W: " << R_contours.br().x-R_contours.tl().x << ", H: " << R_contours.br().y-R_contours.tl().y << std::endl;
				std::string c_area = std::to_string(idx);// + " " + to_string(contourArea(c[idx],false));
				cv::putText(cv_img,c_area,R_contours.br()+ROI_Origin,cv::FONT_HERSHEY_PLAIN,5,CV_RGB(255, 0, 0),3,8);
				cv::drawMarker(cv_img,cm+ROI_Origin,CV_RGB(0,0,255),cv::MARKER_CROSS,50,5,8);
			}
		}
		else
		{
			std::cout << "Too many Targets" << std::endl;
			motion_x = 0.0;
			motion_y = 0.0;
		}

		//Looking for the same target
		target_cm_dt = target_cm_dt - ROI_Origin; //correction in ROI target dist
		for(idx=0; idx < c.size(); idx ++)
		{
			m = cv::moments(c[idx], false);
			cm = cv::Point((int)(m.m10/m.m00),(int)(m.m01/m.m00));
			distance.x = abs(cm.x - target_cm_dt.x);
			distance.y = abs(cm.y - target_cm_dt.y);
			if (sqrt((distance.x * distance.x) + (distance.y * distance.y)) < sqrt((target_cm_dt.x * target_cm_dt.x) + (target_cm_dt.y * target_cm_dt.y)))
			{
				target_cm.x = cm.x;
				target_cm.y = cm.y;
				target_idx = idx; //index of closest object found to acess to it's contour info.
				distance_dt.x = distance.x; //Distance actualitation to compare the next object found
				distance_dt.y = distance.y;
				target_rect = cv::boundingRect( c[idx]);
				std::cout << "Target cm " << target_idx << " -> [" << target_cm.x << "," << target_cm.y << "] A: " << contourArea(c[target_idx],false) << " Dist. X: " << distance_dt.x << ", Dist. Y: " << distance_dt.y << std::endl;
			}
		}
		cv::drawMarker(cv_img,target_cm_dt+ROI_Origin,CV_RGB(10,10,200),cv::MARKER_CROSS,100,5,8);
		cv::line(cv_img,target_cm_dt+ROI_Origin,target_cm+ROI_Origin,cv::Scalar(rand()%255, rand()%255, rand()%255),5,cv::LINE_8,0);
		target_cm_dt = target_cm + ROI_Origin;

		//cv::drawContours(cv_img,c,target_idx,cv::Scalar(rand()%255, rand()%255, rand()%255),CV_FILLED,cv::LINE_8,h,1,ROI_Origin);
		cv::drawMarker(cv_img,cv::Point((cv_img.cols/2),(cv_img.rows/2)),CV_RGB(10,200,30),cv::MARKER_CROSS,100,5,8);
		cv::drawMarker(cv_img,target_cm+ROI_Origin,cv::Scalar(255,255,255),cv::MARKER_CROSS,50,5,8);
		cv::line(cv_img,cv::Point((cv_img.cols/2),(cv_img.rows/2)),target_cm+ROI_Origin,cv::Scalar(rand()%255, rand()%255, rand()%255),5,cv::LINE_8,0);
		std::cout << "cool" << std::endl;
		cv::imshow(OPENCV_WINDOW, cv_img);
		cv::waitKey(3);

		motion_x = 0.002057613*(-(cv_img.cols/2) + (target_cm.x+ROI_Origin.x)); //4 mm / 2592 px = 0.00154321
		motion_y = 0.002057613*(-(cv_img.cols/2) + (target_cm.y+ROI_Origin.y)); //3 mm / 1944 px = 0.002057613

		if (motion_x > motion_max)
		{
			motion_x = 0.0;
		}
		if (motion_y > motion_max)
		{
			motion_y = 0.0;
		}

		motion = "G1 X" + std::to_string(motion_x) + " Z" + std::to_string(motion_y) + " \n";

		std_msgs::String command;
		command.data = motion;
    command_pub_.publish(command);
		cv_ptr.reset();
	}
}; //ImageConverter

int main(int argc, char** argv)
{
	ros::init(argc, argv, "micro_s1_processing_node");
	//ros::NodeHandle n; //needed to use ros::Rate, ros::spin and similars.. else use ros::Time::init()
	//ros::Time::init();
	ImageConverter ic;
	//ros::Publisher command_pub_;
	//command_pub_ = n.advertise<std_msgs::String>("/micro_s1_result", 1);
	//ros::Subscriber sub = n.subscribe("chatter", 1,chatterCallback);
	ros::spin();
	/*//ros::Rate loop_rate(10);
	cv::Mat cv_img; // RGB Image
	cv::Mat cv_gray; // Gray Image
	cv::Mat cv_bn; // binary Image
	cv::Mat cv_bn2; // binary Image
	cv::Mat cv_out; // Output Image
	cv::Mat cv_roi; // ROI Image

	cv::namedWindow(OPENCV_WINDOW, 0);
	cv::namedWindow(OPENCV_WINDOW1, 0);
	cv::namedWindow(OPENCV_WINDOW2, 0);

	//cv_calib.setTo(cv::Scalar(255,255,255));
	cv::Moments m;
	cv::Point cm;

	cv::Point distance;
	cv::Point distance_dt;//(9999,9999);
		distance_dt.x = 9000; //dist max inicial
		distance_dt.y = 9000;

	cv::Point motion_distance;
	cv::Point target_cm;
	cv::Point target_cm_dt;
	cv::Point ROI_Origin;

	cv::Rect target_rect;
	cv::Rect ROI;

	int idx=0; // Index to run contours' vector
	int target_idx=0;

	int ROI_W;
	int ROI_H;

	// Filter parameters
	int w_threshold_max = 350; //50 celeg
	int h_threshold_max = 350;
	int w_threshold_min = 160; //15 celeg
	int h_threshold_min = 160;
	int max_targets = 15;
	const float area_max = 999999.0; //170.0 celeg
	const float area_min = 23000.0; //100	celeg
	//int i=1;

	float motion_x = 0.0;
	float motion_y = 0.0;
	float motion_max = 200.0;

	std::string motion;

	bool preproc = true;
	//while (ros::ok())
	for(int i=0; i<50;i++)
	{
		std::string img_in_dir = IMAGE_DIR;
		img_in_dir += std::to_string(i);
		img_in_dir +=".bmp";
		std::string img_out_dir = "/home/jf/Desktop/Outputs/micro_s1_out";
		img_out_dir += std::to_string(i);
		img_out_dir +=".bmp";
		// std::cout << "Entrada: " << img_in_dir << std::endl;
		// std::cout << "Salida: " << img_out_dir << std::endl;

		if (preproc)
		{
			ROS_INFO("Preprocessing image");
			cv_img = cv::imread(img_in_dir,CV_LOAD_IMAGE_COLOR);
			// std::cout << "rows" << cv_img.rows << std::endl;
			// std::cout << "cols" << cv_img.cols << std::endl;

			//Image binarization
			cv::cvtColor(cv_img, cv_gray, cv::COLOR_BGR2GRAY);	// RGB to GRAY image copy
			cv::medianBlur(cv_gray,cv_gray,11);
			cv::adaptiveThreshold(cv_gray,cv_bn,255,CV_ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY,851,0); //55,2//11,2//11,0
			cv::bitwise_not ( cv_bn, cv_bn );
			cv::imshow(OPENCV_WINDOW1, cv_bn);
			cv::moveWindow(OPENCV_WINDOW1, 1920/2-480, 1080/2-540);
			cv::threshold (cv_gray, cv_bn2, 100, 255,cv::THRESH_BINARY_INV);
			cv::imshow(OPENCV_WINDOW2, cv_bn2);
			cv::moveWindow(OPENCV_WINDOW2, 1920/2, 1080/2-540);
			cv::waitKey(3);

			//Image Segmentation
			std::vector<std::vector<cv::Point> > c; // Contours
			std::vector<cv::Vec4i> h; // Hierarchy
			cv::findContours(cv_bn2,c,h,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
			int numContours = c.size() - 1;
			std::cout << "Number of objects: " << numContours << std::endl;
			std::vector<cv::Vec4i>::iterator ith = h.begin();
			for(std::vector<std::vector<cv::Point> >::iterator it = c.begin(); it != c.end(); )
			{
				cv::Rect R = boundingRect(*it);
				int area = contourArea(*it,false);

				if ((R.width > w_threshold_max || R.height > h_threshold_max) || (R.width < w_threshold_min || R.height < h_threshold_min) || ((area > area_max) || (area < area_min)))
				{
					it = c.erase(it);
					ith = h.erase(ith);
				}
				else
				{
					it ++;
					ith ++;
				}
			}

			if (c.size() > 0)
			{
				std::cout << "Number of targets: " << c.size() << std::endl;
				std::cout << "Number of hierarchies: " << h.size() << std::endl;
			}
			else
			{
				std::cout << "No Targets Found" << std::endl;
			}

			if (c.size() <= max_targets)
			{
				for(idx=0; idx <c.size(); idx ++)
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
					std::cout << "cm " << idx << " -> [" << cm.x << "," << cm.y << "] A: " << contourArea(c[idx],false) << " W: " << R_contours.br().x-R_contours.tl().x << ", H: " << R_contours.br().y-R_contours.tl().y << std::endl;
					std::string c_area = std::to_string(idx);// + " " + to_string(contourArea(c[idx],false));
					cv::putText(cv_img,c_area,R_contours.br(),cv::FONT_HERSHEY_PLAIN,5,CV_RGB(255, 0, 0),3,8);
					//cv::drawContours(cv_img,c,idx,cv::Scalar (rand()%255, rand()%255, rand()%255),CV_FILLED,cv::LINE_8,h,1);
					cv::drawMarker(cv_img,cm,CV_RGB(0,0,255),cv::MARKER_CROSS,50,5,8);
					//cv::drawContours(cv_out,c,idx,cv::Scalar (0, 0, 0),CV_FILLED,cv::LINE_8,h,1);
				}
			}
			else
			{
				std::cout << "Too many Targets" << std::endl;
			}

			//Fixing main Targets

			for(idx=0; idx <c.size(); idx ++)
			{
				m = cv::moments(c[idx], false);
				cm = cv::Point((int)(m.m10/m.m00),(int)(m.m01/m.m00));
				distance.x = abs(cm.x - cv_img.cols/2);
				distance.y = abs(cm.y - cv_img.rows/2);
				std::cout << "cm " << idx << " -> [" << cm.x << "," << cm.y << "] A: " << contourArea(c[idx],false) << " Dist. X: " << distance.x << ", Dist. Y: " << distance.y << std::endl;
				//std::cout << "Act. Dist " << sqrt((distance.x * distance.x) + (distance.y * distance.y)) << std::endl;
				//std::cout << "Prev. Dist " << sqrt((distance_dt.x * distance_dt.x) + (distance_dt.y * distance_dt.y)) << std::endl;
				if (sqrt((distance.x * distance.x) + (distance.y * distance.y)) < sqrt((distance_dt.x * distance_dt.x) + (distance_dt.y * distance_dt.y)))
				{
					target_cm.x = cm.x;
					target_cm.y = cm.y;
					target_idx = idx; //index of closest object found to acess to it's contour info.
					distance_dt.x = distance.x; //Distance actualitation to compare the next object found
					distance_dt.y = distance.y;
					target_rect = cv::boundingRect( c[idx]);
					//std::cout << "cm " << target_idx << " -> [" << target_cm.x << "," << target_cm.y << "] A: " << contourArea(c[target_idx],false) << " Dist. X: " << distance_dt.x << ", Dist. Y: " << distance_dt.y << std::endl;
				}
			}
			distance_dt.x = 9000;
			distance_dt.y = 9000;
			cv::drawContours(cv_img,c,target_idx,cv::Scalar(rand()%255, rand()%255, rand()%255),CV_FILLED,cv::LINE_8,h,1);
			cv::drawMarker(cv_img,cv::Point((cv_img.cols/2),(cv_img.rows/2)),CV_RGB(10,200,30),cv::MARKER_CROSS,100,5,8);
			cv::drawMarker(cv_img,target_cm,cv::Scalar(255,255,255),cv::MARKER_CROSS,50,5,8);
			cv::line(cv_img,cv::Point((cv_img.cols/2),(cv_img.rows/2)),target_cm,cv::Scalar(rand()%255, rand()%255, rand()%255),5,cv::LINE_8,0);
			// cv::imshow(OPENCV_WINDOW, cv_img);
			// cv::waitKey(3);

			// Saving last target position
			target_cm_dt = target_cm;

			//Target ROI
			ROI_W = (target_cm.x-cv_img.cols/2);
			if (ROI_W > 0)
			{
				ROI_W += 2*abs(target_rect.tl().x-target_rect.br().x);
				ROI_Origin.x = (cv_img.cols/2) - abs(target_rect.tl().x-target_rect.br().x);
			}
			else if (ROI_W < 0)
			{
				ROI_W = abs(ROI_W + 2*(target_rect.tl().x-target_rect.br().x));
				ROI_Origin.x = target_cm.x - abs(target_rect.tl().x-target_rect.br().x);
			}
			else
			{
				std::cout << "dimensional Width error" << std::endl;
				return 1;
			}

			ROI_H = (target_cm.y-cv_img.rows/2);
			if (ROI_H > 0)
			{
				ROI_H += 2*abs(target_rect.tl().y-target_rect.br().y);
				ROI_Origin.y = (cv_img.rows/2) - abs(target_rect.tl().y-target_rect.br().y);
			}
			else if (ROI_H < 0)
			{
				ROI_H = abs(ROI_H + 2*(target_rect.tl().y-target_rect.br().y));
				ROI_Origin.y = target_cm.y - abs(target_rect.tl().y-target_rect.br().y);
			}
			else
			{
				std::cout << "dimensional Heigth error" << std::endl;
				return 1;
			}
			ROI.x = ROI_Origin.x;
			ROI.y = ROI_Origin.y;
			ROI.width = ROI_W;
			ROI.height = ROI_H;
			cv::rectangle(cv_img, ROI.tl(), ROI.br(),CV_RGB(0, 0, 0),3,8,0);
			cv::imshow(OPENCV_WINDOW, cv_img);
			cv::waitKey(3);

			preproc = false;
			ROS_INFO("Target fixed");
		} //END OF PREPROC

		cv_img = cv::imread(img_in_dir,CV_LOAD_IMAGE_COLOR);
		// std::cout << "rows" << cv_img.rows << std::endl;
		// std::cout << "cols" << cv_img.cols << std::endl;
		cv_roi = cv_img(ROI);
		//Image binarization
		cv::cvtColor(cv_roi, cv_gray, cv::COLOR_BGR2GRAY);	// RGB to GRAY image copy
		cv::medianBlur(cv_gray,cv_gray,11);
		cv::adaptiveThreshold(cv_gray,cv_bn,255,CV_ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY,851,0); //55,2//11,2//11,0
		cv::bitwise_not ( cv_bn, cv_bn );
		cv::imshow(OPENCV_WINDOW1, cv_bn);
		cv::moveWindow(OPENCV_WINDOW1, 1920/2-480, 1080/2-540);
		cv::threshold (cv_gray, cv_bn2, 100, 255,cv::THRESH_BINARY_INV);
		cv::imshow(OPENCV_WINDOW2, cv_bn2);
		cv::moveWindow(OPENCV_WINDOW2, 1920/2, 1080/2-540);
		cv::rectangle(cv_img, ROI.tl(), ROI.br(),CV_RGB(0, 0, 0),3,8,0);
		cv::imshow(OPENCV_WINDOW, cv_img);
		cv::moveWindow(OPENCV_WINDOW, 1920/2-960, 1080/2-540);
		cv::waitKey(3);

		std::vector<std::vector<cv::Point> > c; // Contours
		std::vector<cv::Vec4i> h; // Hierarchy
		cv::findContours(cv_bn2,c,h,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
		int numContours = c.size() - 1;
		std::cout << "Number of objects: " << numContours << std::endl;
		std::vector<cv::Vec4i>::iterator ith = h.begin();
		for(std::vector<std::vector<cv::Point> >::iterator it = c.begin(); it != c.end(); )
		{
			cv::Rect R = boundingRect(*it);
			int area = contourArea(*it,false);

			if ((R.width > w_threshold_max || R.height > h_threshold_max) || (R.width < w_threshold_min || R.height < h_threshold_min) || ((area > area_max) || (area < area_min)))
			{
				it = c.erase(it);
				ith = h.erase(ith);
			}
			else
			{
				it ++;
				ith ++;
			}
		}

		if (c.size() > 0)
		{
			std::cout << "Number of targets: " << c.size() << std::endl;
			std::cout << "Number of hierarchies: " << h.size() << std::endl;
		}
		else
		{
			std::cout << "No Targets Found" << std::endl;
		}

		if (c.size() <= max_targets)
		{
			for(idx=0; idx < c.size(); idx ++)
			{
				cv::Rect R_contours = cv::boundingRect( c[idx]);
				cv::rectangle(cv_img, R_contours.tl()+ROI_Origin, R_contours.br()+ROI_Origin,CV_RGB(255, 0, 0),3,8,0);
				m = cv::moments(c[idx], false);
				cm = cv::Point((int)(m.m10/m.m00),(int)(m.m01/m.m00));
				// std::string x = std::toString(cm.x);
				// std::string y = std::toString(cm.y);
				// std::string centroid = "(" + x + "," + y + ")";
				// stringstream converter;
				// converter.str("");
				// converter << contourArea(c[idx],false)
				std::cout << "cm " << idx << " -> [" << cm.x << "," << cm.y << "] A: " << contourArea(c[idx],false) << " W: " << R_contours.br().x-R_contours.tl().x << ", H: " << R_contours.br().y-R_contours.tl().y << std::endl;
				std::string c_area = std::to_string(idx);// + " " + to_string(contourArea(c[idx],false));
				cv::putText(cv_img,c_area,R_contours.br()+ROI_Origin,cv::FONT_HERSHEY_PLAIN,5,CV_RGB(255, 0, 0),3,8);
				//cv::drawContours(cv_img,c,idx,cv::Scalar (rand()%255, rand()%255, rand()%255),CV_FILLED,cv::LINE_8,h,1);
				cv::drawMarker(cv_img,cm+ROI_Origin,CV_RGB(0,0,255),cv::MARKER_CROSS,50,5,8);
				//cv::drawContours(cv_out,c,idx,cv::Scalar (0, 0, 0),CV_FILLED,cv::LINE_8,h,1);
			}
		}
		else
		{
			std::cout << "Too many Targets" << std::endl;
		}

		//Looking for the same target
		target_cm_dt = target_cm_dt - ROI_Origin; //correction in ROI target dist
		for(idx=0; idx < c.size(); idx ++)
		{
			m = cv::moments(c[idx], false);
			cm = cv::Point((int)(m.m10/m.m00),(int)(m.m01/m.m00));
			distance.x = abs(cm.x - target_cm_dt.x);
			distance.y = abs(cm.y - target_cm_dt.y);
			//std::cout << "cm " << idx << " -> [" << cm.x << "," << cm.y << "] A: " << contourArea(c[idx],false) << " Dist. X: " << distance.x << ", Dist. Y: " << distance.y << std::endl;
			//std::cout << "Act. Dist " << sqrt((distance.x * distance.x) + (distance.y * distance.y)) << std::endl;
			//std::cout << "Prev. Dist " << sqrt((distance_dt.x * distance_dt.x) + (distance_dt.y * distance_dt.y)) << std::endl;
			if (sqrt((distance.x * distance.x) + (distance.y * distance.y)) < sqrt((target_cm_dt.x * target_cm_dt.x) + (target_cm_dt.y * target_cm_dt.y)))
			{
				target_cm.x = cm.x;
				target_cm.y = cm.y;
				target_idx = idx; //index of closest object found to acess to it's contour info.
				distance_dt.x = distance.x; //Distance actualitation to compare the next object found
				distance_dt.y = distance.y;
				target_rect = cv::boundingRect( c[idx]);
				std::cout << "Target cm " << target_idx << " -> [" << target_cm.x << "," << target_cm.y << "] A: " << contourArea(c[target_idx],false) << " Dist. X: " << distance_dt.x << ", Dist. Y: " << distance_dt.y << std::endl;
			}
		}
		cv::drawMarker(cv_img,target_cm_dt+ROI_Origin,CV_RGB(10,10,200),cv::MARKER_CROSS,100,5,8);
		cv::line(cv_img,target_cm_dt+ROI_Origin,target_cm+ROI_Origin,cv::Scalar(rand()%255, rand()%255, rand()%255),5,cv::LINE_8,0);
		target_cm_dt = target_cm + ROI_Origin;


		//cv::drawContours(cv_img,c,target_idx,cv::Scalar(rand()%255, rand()%255, rand()%255),CV_FILLED,cv::LINE_8,h,1,ROI_Origin);
		cv::drawMarker(cv_img,cv::Point((cv_img.cols/2),(cv_img.rows/2)),CV_RGB(10,200,30),cv::MARKER_CROSS,100,5,8);
		cv::drawMarker(cv_img,target_cm+ROI_Origin,cv::Scalar(255,255,255),cv::MARKER_CROSS,50,5,8);
		cv::line(cv_img,cv::Point((cv_img.cols/2),(cv_img.rows/2)),target_cm+ROI_Origin,cv::Scalar(rand()%255, rand()%255, rand()%255),5,cv::LINE_8,0);
		std::cout << "cool" << std::endl;
		cv::imshow(OPENCV_WINDOW, cv_img);
		cv::waitKey(0);

		motion_x = 0.002057613*(-(cv_img.cols/2) + (target_cm.x+ROI_Origin.x)); //4 mm / 2592 px = 0.00154321
		motion_y = 0.002057613*(-(cv_img.cols/2) + (target_cm.y+ROI_Origin.y)); //3 mm / 1944 px = 0.002057613

		if (motion_x > motion_max)
		{
			motion_x = 0.0;
		}
		if (motion_y > motion_max)
		{
			motion_y = 0.0;
		}

		motion = "G1 X" + std::to_string(motion_x) + " Z" + std::to_string(motion_y) + " \n";

		std_msgs::String command;
		command.data = motion;
    command_pub_.publish(command);

		loop_rate.sleep();
	}*/
	return 0;
}
