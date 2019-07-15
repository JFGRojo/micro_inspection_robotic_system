/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <string.h>

serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("motion", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("marlin", 1000);

    try
    {
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(5);

	if(ros::ok()){std::cout << "ok" << std::endl;}
	else{std::cout << "not ok" << std::endl;}
	while(ros::ok())
	{
    //if(ros::ok()){
        ros::spinOnce();
		//std::cout << "hola3" << std::endl;
 
			std::string res = "";
			std::string res1 = "";
			std::string res2 = "";
			std::cout << "1st command" << std::endl;
			std::cin >> res1;
			//std::cout << "2nd command" << std::endl;
			//std::cin >> res2;
			res = res1;// + " " + res2 + "\n";
			ser.write(res);
			std::cout << res << std::endl; 
		
		if(ser.available())
		{	
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
            read_pub.publish(result);
        }
		else
		{
			std::cout << "not ser aval" << std::endl;
		}
        loop_rate.sleep();
    }
}

