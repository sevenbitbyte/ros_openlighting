#ifndef OLA_BRIDGE_H
#define OLA_BRIDGE_H

#include <ros/ros.h>
#include <ros/console.h>

#include "olamanager.h"
#include "lighting_msgs/run_command.h"

class OlaBridge {
    private:
        ros::NodeHandlePtr _nhPtr;
        ros::ServiceServer _cmdSrv;
        OlaManager _ola;

        ros::Timer _loopTimer;
        QMap<std::string, lighting_msgs::DmxCommand> _commandBuffer;
        lighting_msgs::DmxCommand _currentCommand;
        QDateTime _commandStartTime;

        /*bool _displayEnable;

        lighting_msgs::DmxCommand _displayCmd;
        QDateTime _displayStartTime;*/

    public:
        OlaBridge(ros::NodeHandlePtr ptr);
        ~OlaBridge();

        void render();

    protected:
        bool processCommand(lighting_msgs::run_command::Request& req, lighting_msgs::run_command::Response& res);
        void renderCallback(const ros::TimerEvent& event);

        int renderCmd(lighting_msgs::DmxCommand cmd, QDateTime start, QDateTime now);
        void renderDmxValue(lighting_msgs::DmxValue value);
        void renderDmxEasing(lighting_msgs::DmxEasing easing, qreal progress);
};

#endif  //OLA_BRIDGE_H
