#include <stdio.h>

#include "ola_bridge_nodelet.h"


using namespace ola_ros;

OlaBridge::OlaBridge() : value_(0){
    //
}

void OlaBridge::onInit() {
    ros::NodeHandle& node = getPrivateNodeHandle();


    _client.Setup();
    //_client.GetClient()->FetchUniverseList()
    /*
    node.getParam("value", value_);
    pub = node.advertise<std_msgs::Float64>("out", 10);
    sub = node.subscribe("in", 10, &OlaBridge::callback, this);
    */

    _frameSub = node.subscribe<lighting_msgs::DmxFrame>("frame", 10, &OlaBridge::frameCallback, this);
    _valueSub = node.subscribe<lighting_msgs::DmxValue>("dmx_out", 10, &OlaBridge::dmxCallback, this);

    _valuePub = node.advertise<lighting_msgs::DmxValue>("dmx_in", 10);
}

void OlaBridge::dmxCallback(const lighting_msgs::DmxValue::ConstPtr& input) {
    //
}

void OlaBridge::frameCallback(const lighting_msgs::DmxFrame::ConstPtr& input) {
    //input->values;
    //input->easings;
}

void OlaBridge::callback(const std_msgs::Float64::ConstPtr& input) {
    std_msgs::Float64Ptr output(new std_msgs::Float64());
    output->data = input->data + value_;
    NODELET_DEBUG("Adding %f to get %f", value_, output->data);
    pub.publish(output);
}
