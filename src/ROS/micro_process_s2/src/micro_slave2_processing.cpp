#include <ros/ros.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <string>
//#include <iostream>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Contornos";
static const std::string OPENCV_WINDOW1 = "Mask";
static const std::string OPENCV_WINDOW2 = "ROI area";
static const std::string OPENCV_WINDOW3 = "Binary Image";


void chatterCallback(const std_msgs::String::ConstPtr& msg)
				{

					cv::Mat cv_img; // RGB Image
					cv::Mat cv_gray; // Gray Image
					cv::Mat cv_out; // Segmented Image
					cv::Mat cv_roi; // ROI Image


					std::vector<std::vector<cv::Point> > c; // Contours
					std::vector<cv::Vec4i> h; // Hierarchy
					cv::Moments m;
					cv::Point cm;
					/*cv::Point center; // Image center
						center.x=1296;
						center.y=972;*/

					/*int width = 500; // Rectangle ROI parameters
					int height = 500;

					cv::Point offset; // Offset if
						offset.x=center.x+width;
						offset.y=center.y-height;*/

					int idx=0; // Index to run contours' vector

					// Filter parameters
					int w_threshold_max = 50; //50 celeg
					int h_threshold_max = 50;
					int w_threshold_min = 15; //15 celeg
					int h_threshold_min = 15;
					const float area_max = 170.0; //170.0 celeg
					const float area_min = 100.0; //100	celeg

							cv_img = cv::imread("/home/pi/Desktop/CalLeft4.bmp",CV_LOAD_IMAGE_COLOR); // Color Image reading  "/home/pi/Desktop/MacroV1ss100br35.bmp"

						    cv::Mat mask = cv::Mat::zeros(cv_img.size(), CV_8UC1); // Mask Image to ROI definition
							cv::Mat cv_imagePart = cv::Mat::zeros(cv_img.size(), cv_img.type()); // ROI Image definition

							cv::Point circleCenter(mask.cols / 2 , mask.rows / 2 );
							int radius = std::min(mask.cols/2-50, mask.rows/2-50);

							int width = radius; // Rectangle ROI parameters
							int height = radius;

							cv::Point offset; // Offset if
							offset.x=circleCenter.x-radius;
							offset.y=circleCenter.y-radius;

							cv::circle(mask, circleCenter, radius, 255,-1);

							cv::imshow(OPENCV_WINDOW1, mask);
							cv::moveWindow(OPENCV_WINDOW1, 1920/2-500, 1080/2-500);
							cv::waitKey(3);

							cv::cvtColor(cv_img, cv_gray, COLOR_BGR2GRAY);	// RGB to GRAY image copy
							cv_gray.copyTo(cv_imagePart, mask); // Copy GRAY Image to ROI Image using Mask
							cv::Rect roi(circleCenter.x - radius, circleCenter.y - radius, 2*radius, 2*radius);
							cv::Rect true_roi(0, 0, cv_img.cols, cv_img.rows);
							roi = roi & cv::Rect(0, 0, cv_img.cols, cv_img.rows);

							//cv::rectangle(cv_gray,Rect(center.x-500,center.y-500,1000,1000),cv::Scalar (rand()%255, rand()%255, rand()%255),2,8,0);

							cv::circle(cv_img, circleCenter, radius, CV_RGB(255, 0, 0),10);
							cv::rectangle(cv_img,true_roi.tl(),true_roi.br(),CV_RGB(255, 0, 0),10,8,0);
							cv::imshow(OPENCV_WINDOW2, cv_img);
							cv::moveWindow(OPENCV_WINDOW2, 1920/2, 1080/2);
					 		cv::waitKey(3);

							//roi = cv_gray(Rect(center.x-500,center.y-500,1000,1000));

							cv_roi = cv_imagePart(roi);
							//cv::imshow(OPENCV_WINDOW3, cv_roi);
							//cv::moveWindow(OPENCV_WINDOW3, 1920/2-500, 1080/2+500);
					 		//cv::waitKey(3);

							cv::medianBlur(cv_roi,cv_roi,7);
							cv::adaptiveThreshold(cv_roi,cv_out,255,CV_ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY,11,-2); //55,2//11,2//11,0

							cv::imshow(OPENCV_WINDOW3, cv_out);
							cv::moveWindow(OPENCV_WINDOW3, 1920/2-500, 1080/2+500);
					 		cv::waitKey(3);

							cv::findContours(cv_out,c,h,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
							int numContours = c.size() - 1;
							cout << "Number of objects: " << numContours << endl;
							std::vector<cv::Vec4i>::iterator ith = h.begin();

							for(vector<vector<cv::Point> >::iterator it = c.begin(); it != c.end(); )
							{
								Rect R = boundingRect(*it);
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
							cout << "Number of targets: " << numTargets << endl;
							int numH = h.size();
							cout << "Number of hierarchies: " << numH << endl;
							//for(idx=0;idx >=0; idx = h[idx][0]) {
							if (c.size() > 0)
							{
								for(idx=0;idx <numTargets; idx ++) {
									Rect R_contours = cv::boundingRect( c[idx]);
									cv::rectangle(cv_img, R_contours.tl()+offset, R_contours.br()+offset,CV_RGB(255, 0, 0),3,8,0);
									m = cv::moments(c[idx], false);
									cm = Point((int)(m.m10/m.m00),(int)(m.m01/m.m00));
									/*std::string x = std::toString(cm.x);
									std::string y = std::toString(cm.y);
									std::string centroid = "(" + x + "," + y + ")";*/
									/*stringstream converter;
									converter.str("");
									converter << contourArea(c[idx],false)*/

									string c_area = to_string(idx);// + " " + to_string(contourArea(c[idx],false));
									cv::putText(cv_img,c_area,cm+offset,FONT_HERSHEY_PLAIN,5,CV_RGB(255, 0, 0),3,8);
									cv::drawContours(cv_img,c,idx,cv::Scalar (rand()%255, rand()%255, rand()%255),CV_FILLED,LINE_8,h,1,offset);
									cout << idx << " " << contourArea(c[idx],false) << " " << R_contours.br().x-R_contours.tl().x << "," << R_contours.br().y-R_contours.tl().y << endl;
								}
							}
							// Update GUI Window
						    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
							cv::imshow(OPENCV_WINDOW, cv_img);
							cv::moveWindow(OPENCV_WINDOW, 1920/2-600, 1080/2+10);
					 		cv::waitKey(3);

			   }

class ImageConverter
{
   ros::NodeHandle nh_;
   image_transport::ImageTransport it_;
   image_transport::Subscriber image_sub_;
   image_transport::Publisher image_pub_;



	 public:
		   ImageConverter()
			 : it_(nh_)
			   {
			     // Subscrive to input video feed and publish output video feed
				     image_sub_ = it_.subscribe("/micro_s2_image", 1,
					 & ImageConverter::imageCb, this);
				     image_pub_ = it_.advertise("/micro_s2_result", 1);

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
					cv::Mat cv_img; // RGB Image
					cv::Mat cv_gray; // Gray Image
					cv::Mat cv_out; // Segmented Image
					cv::Mat cv_roi; // ROI Image


					std::vector<std::vector<cv::Point> > c; // Contours
					std::vector<cv::Vec4i> h; // Hierarchy
					cv::Moments m;
					cv::Point cm;
					/*cv::Point center; // Image center
						center.x=1296;
						center.y=972;*/

					/*int width = 500; // Rectangle ROI parameters
					int height = 500;

					cv::Point offset; // Offset if
						offset.x=center.x+width;
						offset.y=center.y-height;*/

					int idx=0; // Index to run contours' vector

					// Filter parameters
					int w_threshold_max = 50; //50 celeg
					int h_threshold_max = 50;
					int w_threshold_min = 15; //15 celeg
					int h_threshold_min = 15;
					const float area_max = 170.0; //170.0 celeg
					const float area_min = 100.0; //100	celeg

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

							cv_img = cv::imread("/home/pi/Desktop/CalLeft4.bmp",CV_LOAD_IMAGE_COLOR); // Color Image reading  "/home/pi/Desktop/MacroV1ss100br35.bmp"

						    cv::Mat mask = cv::Mat::zeros(cv_img.size(), CV_8UC1); // Mask Image to ROI definition
							cv::Mat cv_imagePart = cv::Mat::zeros(cv_img.size(), cv_img.type()); // ROI Image definition

							cv::Point circleCenter(mask.cols / 2 , mask.rows / 2 );
							int radius = std::min(mask.cols/2-50, mask.rows/2-50);

							int width = radius; // Rectangle ROI parameters
							int height = radius;

							cv::Point offset; // Offset if
							offset.x=circleCenter.x-radius;
							offset.y=circleCenter.y-radius;

							cv::circle(mask, circleCenter, radius, 255,-1);

							cv::imshow(OPENCV_WINDOW1, mask);
							cv::moveWindow(OPENCV_WINDOW1, 1920/2-500, 1080/2-500);
							cv::waitKey(3);

							cv::cvtColor(cv_img, cv_gray, COLOR_BGR2GRAY);	// RGB to GRAY image copy
							cv_gray.copyTo(cv_imagePart, mask); // Copy GRAY Image to ROI Image using Mask
							cv::Rect roi(circleCenter.x - radius, circleCenter.y - radius, 2*radius, 2*radius);
							cv::Rect true_roi(0, 0, cv_img.cols, cv_img.rows);
							roi = roi & cv::Rect(0, 0, cv_img.cols, cv_img.rows);

							//cv::rectangle(cv_gray,Rect(center.x-500,center.y-500,1000,1000),cv::Scalar (rand()%255, rand()%255, rand()%255),2,8,0);

							cv::circle(cv_img, circleCenter, radius, CV_RGB(255, 0, 0),10);
							cv::rectangle(cv_img,true_roi.tl(),true_roi.br(),CV_RGB(255, 0, 0),10,8,0);
							cv::imshow(OPENCV_WINDOW2, cv_img);
							cv::moveWindow(OPENCV_WINDOW2, 1920/2, 1080/2);
					 		cv::waitKey(3);

							//roi = cv_gray(Rect(center.x-500,center.y-500,1000,1000));

							cv_roi = cv_imagePart(roi);
							//cv::imshow(OPENCV_WINDOW3, cv_roi);
							//cv::moveWindow(OPENCV_WINDOW3, 1920/2-500, 1080/2+500);
					 		//cv::waitKey(3);

							cv::medianBlur(cv_roi,cv_roi,7);
							cv::adaptiveThreshold(cv_roi,cv_out,255,CV_ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY,11,-2); //55,2//11,2//11,0

							cv::imshow(OPENCV_WINDOW3, cv_out);
							cv::moveWindow(OPENCV_WINDOW3, 1920/2-500, 1080/2+500);
					 		cv::waitKey(3);

							cv::findContours(cv_out,c,h,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
							int numContours = c.size() - 1;
							cout << "Number of objects: " << numContours << endl;
							std::vector<cv::Vec4i>::iterator ith = h.begin();

							for(vector<vector<cv::Point> >::iterator it = c.begin(); it != c.end(); )
							{
								Rect R = boundingRect(*it);
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
							cout << "Number of targets: " << numTargets << endl;
							int numH = h.size();
							cout << "Number of hierarchies: " << numH << endl;
							//for(idx=0;idx >=0; idx = h[idx][0]) {
							if (c.size() > 0)
							{
								for(idx=0;idx <numTargets; idx ++) {
									Rect R_contours = cv::boundingRect( c[idx]);
									cv::rectangle(cv_img, R_contours.tl()+offset, R_contours.br()+offset,CV_RGB(255, 0, 0),3,8,0);
									m = cv::moments(c[idx], false);
									cm = Point((int)(m.m10/m.m00),(int)(m.m01/m.m00));
									/*std::string x = std::toString(cm.x);
									std::string y = std::toString(cm.y);
									std::string centroid = "(" + x + "," + y + ")";*/
									/*stringstream converter;
									converter.str("");
									converter << contourArea(c[idx],false)*/

									string c_area = to_string(idx);// + " " + to_string(contourArea(c[idx],false));
									cv::putText(cv_img,c_area,cm+offset,FONT_HERSHEY_PLAIN,5,CV_RGB(255, 0, 0),3,8);
									cv::drawContours(cv_img,c,idx,cv::Scalar (rand()%255, rand()%255, rand()%255),CV_FILLED,LINE_8,h,1,offset);
									cout << idx << " " << contourArea(c[idx],false) << " " << R_contours.br().x-R_contours.tl().x << "," << R_contours.br().y-R_contours.tl().y << endl;
								}
							}
							// Update GUI Window
						    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
							cv::imshow(OPENCV_WINDOW, cv_img);
							cv::moveWindow(OPENCV_WINDOW, 1920/2-600, 1080/2+10);
					 		cv::waitKey(3);

							 // Output modified video stream
							 image_pub_.publish(cv_ptr->toImageMsg());
			   }
		 };

int main(int argc, char** argv)
{
   ros::init(argc, argv, "micro_s2_processing_node");
   ImageConverter ic;
   ros::NodeHandle n;
   ros::Subscriber sub = n.subscribe("chatter", 1,chatterCallback);
   ros::spin();
   return 0;
}
