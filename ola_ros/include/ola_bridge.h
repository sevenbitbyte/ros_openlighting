#ifndef OLA_BRIDGE_H
#define OLA_BRIDGE_H

#include <ros/ros.h>
#include <ros/console.h>

#include "olamanager.h"
#include "pixel_mapper.h"
#include "lighting_msgs/dmx_command.h"
#include "lighting_msgs/create_dmx_device.h"
#include "lighting_msgs/set_pixelmap.h"
#include "lighting_msgs/device_list.h"

class OlaBridge {
    private:
        ros::NodeHandlePtr _nhPtr;
        ros::ServiceServer _cmdSrv;
        ros::ServiceServer _devCreateSrv;
        ros::ServiceServer _setPixelMapSrv;
        ros::ServiceServer _devListSrv;

        OlaManager _ola;
        QMap<QString, lighting_msgs::DmxDevice> _devices;
        QMap<QString, PixelMapper*> _pixelMapper;

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
        OlaManager* getOla();

    protected:
        bool processCommand(lighting_msgs::dmx_command::Request& req, lighting_msgs::dmx_command::Response& res);
        bool createDmxDevice(lighting_msgs::create_dmx_device::Request& req, lighting_msgs::create_dmx_device::Response& res);
        bool updatePixelMap(lighting_msgs::set_pixelmap::Request& req, lighting_msgs::set_pixelmap::Response& res);
        bool deviceList(lighting_msgs::device_list::Request& req, lighting_msgs::device_list::Response& res);

        void renderCallback(const ros::TimerEvent& event);

        int renderCmd(lighting_msgs::DmxCommand cmd, QDateTime start, QDateTime now);
        void renderDmxValue(lighting_msgs::DmxValue value);
        void renderDmxEasing(lighting_msgs::DmxEasing easing, qreal progress);
};

#endif  //OLA_BRIDGE_H
