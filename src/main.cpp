#include "LIDAR.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "LIDAR");
    LIDAR LID;

    ros::spin();

    return 0;
}
