#include <ros/ros.h>
#include <ros/console.h>
#include <librealsense/rs.hpp>

int main(int argc, char * argv[]) {
    rs::log_to_console(rs::log_severity::warn);
    ros::init(argc, argv, "acrv_realsense_ros_node");
    ros::NodeHandle n("~");

   rs::context ctx;


   for (int i=0; i<ctx.get_device_count(); ++i)
   {
     rs::device * dev;
     dev = ctx.get_device(i);
     std::cout << dev->get_serial() << std::endl;
   }
 }
