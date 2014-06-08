#include <ros/ros.h>
#include <ros/console.h>

#include "lighting_msgs/run_command.h"


using namespace std;

ros::NodeHandlePtr _nhPtr;


int main(int argc, char** argv){

    ros::init(argc, argv, "dmx_test");
    _nhPtr = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    u_int16_t universe = 1;
    //_nhPtr->getParam("universe", universe);

    lighting_msgs::DmxCommand dispCmd;

    dispCmd.action = lighting_msgs::DmxCommand::DISPLAY;
    dispCmd.loop = true;

    lighting_msgs::DmxFrame frame0;

    frame0.delay.fromSec(0.0f);
    frame0.duration.fromSec(5.0f);

    lighting_msgs::DmxEasing e1;
    e1.curve = lighting_msgs::DmxEasing::Linear;
    e1.delay.fromSec(0.0f);
    e1.duration.fromSec(5.0f);

    lighting_msgs::DmxValue valueTemp;
    valueTemp.universe = universe;
    valueTemp.offset = 0;
    valueTemp.data.assign(3, 0);

    e1.start.push_back(valueTemp);

    valueTemp.data.assign(3, 255);
    e1.end.push_back(valueTemp);

    frame0.easings.push_back(e1);

    valueTemp.offset += 3;

    frame0.values.push_back(valueTemp);
    dispCmd.layers.push_back(frame0);

    ros::ServiceClient client = _nhPtr->serviceClient<lighting_msgs::run_command::Request>("/ola_bridge/run_cmd");

    lighting_msgs::run_command srv;
    srv.request.command = dispCmd;

    if(client.call(srv)){
        std::cout << "Successful call" << std::endl;
    }
    else{
        std::cout << "FAILED call" << std::endl;
    }

    ros::spin();

    return 0;
}

