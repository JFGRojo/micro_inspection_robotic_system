/***
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <string.h>


class SubscribeAndPublish
{
	public:
		SubscribeAndPublish(std::string sub_topic, std::string pub_topic)
		{
			//Topic we are subscribed to
			sub_ = nh.subscribe(sub_topic, 1, &SubscribeAndPublish::callback, this);
			//Topic we are publishing to
			pub_ = nh.advertise<std_msgs::String>(pub_topic, 1);
		}

		void callback(const std_msgs::String::ConstPtr& msg)
		{
			//Read
			ROS_INFO("Callback. Heard: [%s]",msg->data.c_str());
			// std::string res = "";
			// res = (msg->data.c_str());
			// res = res1 + "\n";
			// ser.write(res);
			// //Publish
			// std_msgs::String out;
			// out.data = ser.read(ser.available());
			// ROS_INFO_STREAM("Read: " << result.data);
			pub_.publish(msg);
		}

	private:
		ros::NodeHandle nh;
		ros::Subscriber sub_;
		ros::Publisher pub_;

};//SubscribeAndPublish

int main (int argc, char** argv)
{
    ros::init(argc, argv, "robotic_sys_control");

		SubscribeAndPublish SAP_teensy("/teensy_lighting","/action");
		SubscribeAndPublish SAP_marlin("/micro_s1_result","/motion");

		ros::spin();

		return 0;
}
