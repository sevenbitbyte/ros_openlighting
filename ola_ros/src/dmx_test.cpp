#include <ros/ros.h>
#include <ros/console.h>

#include "lighting_msgs/dmx_command.h"

#define TRANSITION_MS (2000.0f)
#define FADE_TIME_MS (5000.0f)

using namespace std;

ros::NodeHandlePtr _nhPtr;

lighting_msgs::DmxCommand renderSnake(int universe) {
    lighting_msgs::DmxCommand dispCmd;
    for(int i=0; i<512; i++){
        lighting_msgs::DmxFrame f;
        f.delayMs = ( i * TRANSITION_MS );
        f.durationMs = ( FADE_TIME_MS );

        if(i > 0){
            //Setup static values
            lighting_msgs::DmxValue staticValue;
            staticValue.universe = universe;
            staticValue.offset = 0;
            staticValue.data.assign(i, 255);
            f.values.push_back(staticValue);
        }

        lighting_msgs::DmxEasing fadeIn;
        fadeIn.delayMs = 0;
        fadeIn.durationMs = (FADE_TIME_MS);
        fadeIn.curve = lighting_msgs::DmxEasing::Linear;

        lighting_msgs::DmxValue fadeValue;
        fadeValue.universe = universe;
        fadeValue.offset = i;
        fadeValue.data.push_back(0);
        fadeIn.start.push_back(fadeValue);      //Start fade value

        fadeValue.data[0] = 255;
        fadeIn.end.push_back(fadeValue);        //End fade value

        f.easings.push_back(fadeIn);
        dispCmd.layers.push_back(f);

        for(int j=0; j<10; j++){
            dispCmd.layers.push_back(f);
        }
    }

    return dispCmd;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "dmx_test");
    _nhPtr = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    u_int16_t universe = 1;
    //_nhPtr->getParam("universe", universe);

    /*lighting_msgs::DmxCommand dispCmd;

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
    dispCmd.layers.push_back(frame0);*/

    ros::ServiceClient client = _nhPtr->serviceClient<lighting_msgs::dmx_command::Request>("/ola_bridge/dmx_cmd");

    lighting_msgs::dmx_command srv;
    srv.request.command = renderSnake(1);

    if(client.call(srv)){
        std::cout << "Successful call" << std::endl;
    }
    else{
        std::cout << "FAILED call" << std::endl;
    }

    ros::spin();

    return 0;
}

