#include "ola_bridge.h"

OlaBridge::OlaBridge(ros::NodeHandlePtr ptr){
    _nhPtr = ptr;
    _cmdSrv = _nhPtr->advertiseService("dmx_cmd", &OlaBridge::processCommand, this);
    _devCreateSrv = _nhPtr->advertiseService("create_dmx_device", &OlaBridge::createDmxDevice, this);
    _setPixelMapSrv = _nhPtr->advertiseService("set_pixelmap", &OlaBridge::updatePixelMap, this);
    _devListSrv = _nhPtr->advertiseService("device_list", &OlaBridge::deviceList, this);

    _loopTimer = _nhPtr->createTimer(ros::Duration(0.03f), &OlaBridge::renderCallback, this);
}

OlaBridge::~OlaBridge() {
    _loopTimer.stop();
    _cmdSrv.shutdown();
    _devCreateSrv.shutdown();
    _setPixelMapSrv.shutdown();

    while(!_pixelMapper.empty()){
        PixelMapper* mapper = _pixelMapper.last();
        _pixelMapper.remove(_pixelMapper.lastKey());
        delete mapper;
    }
}

OlaManager* OlaBridge::getOla(){
    return &_ola;
}

void OlaBridge::renderCallback(const ros::TimerEvent& event) {
    render();
    _ola.sendBuffers();
}

void OlaBridge::renderDmxValue(lighting_msgs::DmxValue value) {
    uint8_t* d = new uint8_t[value.data.size()];

    for(int i=0; i<value.data.size(); i++){
        d[i] = value.data[i];
    }

    ola::DmxBuffer* buf = _ola.getBuffer(value.universe);
    buf->SetRange(value.offset, d, value.data.size());

    delete d;
}

void OlaBridge::renderDmxEasing(lighting_msgs::DmxEasing easing, qreal progress) {
    QEasingCurve curve( (QEasingCurve::Type) easing.curve);

    qreal value = curve.valueForProgress(progress);

    assert(easing.start.size() == easing.end.size());

    for(int i=0; i<easing.start.size(); i++) {
        lighting_msgs::DmxValue start = easing.start[i];
        lighting_msgs::DmxValue end = easing.end[i];

        assert(start.universe == end.universe);
        assert(start.offset == end.offset);
        assert(start.data.size() == end.data.size());

        ola::DmxBuffer* buf = _ola.getBuffer(start.universe);

        for(int j=0; j<start.data.size(); j++){
            qreal d = (((qreal)end.data[j] - (qreal)start.data[j]) * value) + start.data[j];

            buf->SetChannel(j + start.offset, (uint8_t)d);
        }
    }
}

int OlaBridge::renderCmd(lighting_msgs::DmxCommand cmd, QDateTime start, QDateTime now){
    int activeFrames = 0;
    foreach(lighting_msgs::DmxFrame frame, cmd.layers){
        qreal deltaMs = start.msecsTo(now);
        qreal delayMs = frame.delayMs;
        qreal durationMs = frame.durationMs;

        qreal progress = (deltaMs - delayMs) / durationMs;

        if(progress >= 0.0f && progress <= 1.0f){
            //Render this frame

            foreach(lighting_msgs::DmxValue value, frame.values) {
                renderDmxValue(value);
            }

            foreach(lighting_msgs::DmxEasing easing, frame.easings) {
                qreal easingDelayMs = delayMs + easing.delayMs;
                qreal easingDurationMs = easing.durationMs;

                qreal easingProgress = (deltaMs - easingDelayMs) / easingDurationMs;
                if(easingProgress >= 0.0f && easingProgress <= 1.0f){
                    renderDmxEasing(easing, easingProgress);
                }
            }
            activeFrames++;
        }
        else if(progress < 0.0f){
            //Has yet to run
            activeFrames++;
        }
    }

    return activeFrames;
}

void OlaBridge::render() {
    QDateTime now = QDateTime::currentDateTimeUtc();

    if(_currentCommand.action == lighting_msgs::DmxCommand::PLAY){
        // Play

        if(_commandBuffer.contains(_currentCommand.name)){
            int count = renderCmd(_commandBuffer.value(_currentCommand.name), _commandStartTime, now);
        }
    }
    else if(_currentCommand.action == lighting_msgs::DmxCommand::DISPLAY){
        // Display
        lighting_msgs::DmxCommand cmd;

        if( !_currentCommand.layers.empty() ){
            //Render layers
            cmd = _currentCommand;
        }
        else if( !_currentCommand.name.empty() && _commandBuffer.contains(_currentCommand.name)){
            //Render named command
            cmd = _commandBuffer.value(_currentCommand.name);
        }

        if(!cmd.layers.empty()){
            int count = renderCmd(cmd, _commandStartTime, now);

            if(count < 1){
                //End of frame
                if(cmd.loop){
                    _commandStartTime = now;
                }
                else{

                    if( !cmd.next.empty() ){
                        _commandStartTime = now;
                        _currentCommand.action = lighting_msgs::DmxCommand::PLAY;
                        _currentCommand.name = cmd.next;
                        _currentCommand.layers.clear();
                    }
                    else{
                        _currentCommand.action = lighting_msgs::DmxCommand::STOP;
                    }
                }
            }
        }
    }
}

bool OlaBridge::createDmxDevice(lighting_msgs::create_dmx_device::Request& req, lighting_msgs::create_dmx_device::Response& res){
    QString devName(req.device.name.c_str());

    if(_devices.contains(devName)){
        res.error = "Device name in use";
        return true;
    }
    else if(devName.isEmpty()){
        res.error = "Empty device names not allowed";
        return true;
    }

    _devices.insert(devName, req.device);

    return true;
}

bool OlaBridge::updatePixelMap(lighting_msgs::set_pixelmap::Request& req, lighting_msgs::set_pixelmap::Response& res){
    QString devName = req.device_name.c_str();

    if(!_devices.contains(devName)){
        res.error = "Device does not exist";
        return true;
    }

    if(!_pixelMapper.contains(devName)){
        //Register new pixel map
        PixelMapper* p = new PixelMapper(_nhPtr, devName, getOla());
        _pixelMapper.insert(devName, p);
    }

    PixelMapper* mapper = _pixelMapper[devName];

    if(mapper==NULL){
        res.error = "Failed to locate pixelmaper";
        return true;
    }


    //Parse mapping json blob and update pixelmap
    QByteArray jsonText(req.mapping.c_str());
    QJsonParseError error;
    QJsonDocument json = QJsonDocument::fromJson(jsonText, &error);

    if(error.error != QJsonParseError::NoError){
        QByteArray temp = error.errorString().toLocal8Bit();
        res.error = temp.data();
    }
    else if(!mapper->fromJson(json)){
        res.error = "Malformed pixel mapping";
    }


    QByteArray topicTemp = mapper->topicPath().toLocal8Bit();
    res.topic = topicTemp.data();

    return true;
}

bool OlaBridge::deviceList(lighting_msgs::device_list::Request& req, lighting_msgs::device_list::Response& res){

    foreach(lighting_msgs::DmxDevice device, _devices){
        res.devices.push_back(device);
    }

    return true;
}

bool OlaBridge::processCommand(lighting_msgs::dmx_command::Request& req, lighting_msgs::dmx_command::Response& res) {
    QDateTime now = QDateTime::currentDateTimeUtc();

    std::cout<< "Got a command " << (int)req.command.action
             << " with " << req.command.layers.size() << std::endl;

    if(req.command.action == lighting_msgs::DmxCommand::DISPLAY){
        if( !req.command.layers.empty() ){
            //TODO: Validate command
            _currentCommand = req.command;
            _commandStartTime = now;

            std::cout<< "First layer has "
                     << req.command.layers[0].values.size() << "values "
                     << req.command.layers[0].easings.size() << "easings "
                     << (int)req.command.layers[0].durationMs << "duration"
                     << std::endl;

            return true;
        }
        else if( !req.command.name.empty() ){
            _currentCommand = req.command;
            _commandStartTime = now;

            return true;
        }
    }
    else if(req.command.action == lighting_msgs::DmxCommand::PLAY){
        if( !req.command.name.empty() && _commandBuffer.contains(req.command.name) ){
            _currentCommand = req.command;
            _commandStartTime = now;
            _loopTimer.setPeriod(ros::Duration(0.1f));
            _loopTimer.start();

            return true;
        }
    }
    else if(req.command.action == lighting_msgs::DmxCommand::STOP){
        _currentCommand.action = lighting_msgs::DmxCommand::STOP;
        _loopTimer.setPeriod(ros::Duration(0.5f));
        _loopTimer.start();
        return true;
    }
    else if(req.command.action == lighting_msgs::DmxCommand::REMOVE){
        if(_commandBuffer.contains(req.command.name)){
            _commandBuffer.remove(req.command.name);
        }
        return true;
    }
    else if(req.command.action == lighting_msgs::DmxCommand::STORE){
        //TODO: Validate command
        _commandBuffer.insert( req.command.name, req.command );
        return true;
    }
    else if(req.command.action == lighting_msgs::DmxCommand::LOAD){
        return false;
    }

    return false;
}
