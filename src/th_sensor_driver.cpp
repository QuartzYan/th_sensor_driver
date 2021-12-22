#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <boost/format.hpp>

#include "modbus_rtu_master.h"
#include "th_sensor_driver/th_sensor.h"

class THSensorDriver
{
public:
  THSensorDriver(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~THSensorDriver();
  void loop();

private:
  ros::NodeHandle nh_, private_nh_;
  th_sensor_driver::th_sensor msg_;
  ros::Publisher sensor_msg_pub_;
  int32_t baudrate_;
  std::string sensor_com_;
  ModbusRTUMaster *sensor_;
};

THSensorDriver::THSensorDriver(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh), private_nh_(private_nh)
{
  private_nh_.param<int>("baudrate", baudrate_, 9600);
  private_nh_.param<std::string>("sensor_com", sensor_com_, "/dev/ttyUSB1");
  
  sensor_ = new ModbusRTUMaster(sensor_com_, baudrate_);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  sensor_msg_pub_ = nh_.advertise<th_sensor_driver::th_sensor>("th_sensor_data", 10);
}

THSensorDriver::~THSensorDriver()
{
  free(sensor_);
  sensor_ = nullptr;
}

void THSensorDriver::loop()
{
  uint16_t data[256] = {0};
  uint8_t ret = 0;

  msg_.header.stamp = ros::Time::now();

  // get Atmospheric humidity and temperature
  ret = sensor_->GetMultipleRegisters(0x01, 0x0000, 0x0002, data);
  if (!ret) ROS_WARN("get Atmospheric humidity&temperature faild!!");
  else {
    msg_.humid = float(data[0] / 10.0);
    msg_.temp = float(data[1] / 10.0);
    sensor_msg_pub_.publish(msg_);
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "th_sensor_driver");
  ros::NodeHandle nh, private_nh("~");

  THSensorDriver jx(nh, private_nh);

  ros::Rate r(1);

  while (ros::ok())
  {
    ros::spinOnce();
    jx.loop();
    r.sleep();
  }

  ROS_INFO("All finish");

  return 0;
}