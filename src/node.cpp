#include "encode_ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "encode");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    encode::EncodeRos encode;

    ros::spin();

    return 0;
}
