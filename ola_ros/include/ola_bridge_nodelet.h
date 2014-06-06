#ifndef OLA_BRIDGE_H
#define OLA_BRIDGE_H

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <ola/Callback.h>
#include <ola/Clock.h>
#include <ola/DmxBuffer.h>
#include <ola/Logging.h>
#include <ola/OlaClientWrapper.h>
#include <ola/base/Flags.h>
#include <ola/base/Init.h>
#include <ola/thread/SignalThread.h>

#include <std_msgs/Float64.h>
#include <lighting_msgs/DmxValue.h>
#include <lighting_msgs/DmxFrame.h>
#include <lighting_msgs/DmxEasing.h>
#include <lighting_msgs/DmxCommand.h>

#include "olamanager.h"

namespace ola_ros {
    class OlaBridge : public nodelet::Nodelet {
        public:
            OlaBridge();

        private:
            virtual void onInit();

            void callback(const std_msgs::Float64::ConstPtr& input);

            void dmxCallback(const lighting_msgs::DmxValue::ConstPtr& input);
            void frameCallback(const lighting_msgs::DmxFrame::ConstPtr& input);

            ros::Subscriber _valueSub;
            ros::Subscriber _frameSub;
            ros::Publisher _frameStatusPub;
            ros::Publisher _valuePub;

            ros::Publisher pub;
            ros::Subscriber sub;
            double value_;

            ola::OlaCallbackClientWrapper _client;
    };

    //PLUGINLIB_EXPORT_CLASS(ola_ros::OlaBridge, nodelet::Nodelet);
    PLUGINLIB_DECLARE_CLASS(ola_ros, OlaBridge, ola_ros::OlaBridge, nodelet::Nodelet);
}


#endif  //OLA_BRIDGE_H
