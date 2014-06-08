#include <ros/ros.h>
#include <ros/console.h>

#include "olamanager.h"
#include "ola_bridge.h"
#include "lighting_msgs/run_command.h"


using namespace std;

ros::NodeHandlePtr _nhPtr;


int main(int argc, char** argv){

    ros::init(argc, argv, "ola_bridge_node");
    _nhPtr = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    OlaBridge olaBridge(_nhPtr);

    ros::spin();

    return 0;
}

