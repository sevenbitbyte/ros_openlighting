#include <ros/ros.h>
#include <ros/console.h>

#include "olamanager.h"


using namespace std;

ros::NodeHandlePtr _nhPtr;

int main(int argc, char** argv){

    ros::init (argc, argv, "pixel_map_node");
    _nhPtr = ros::NodeHandlePtr(new ros::NodeHandle());

    OlaManager ola;

    ros::spin();

    return 0;
}
