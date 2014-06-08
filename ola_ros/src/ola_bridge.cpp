#include "ola_bridge.h"

OlaBridge::OlaBridge(ros::NodeHandlePtr ptr){
    _nhPtr = ptr;
    _cmdSrv = _nhPtr->advertiseService("run_cmd", &OlaBridge::processCommand, this);

    _loopTimer = _nhPtr->createTimer(ros::Duration(0.33f), &OlaBridge::renderCallback, this);
}

OlaBridge::~OlaBridge() {
    //
}

void OlaBridge::renderCallback(const ros::TimerEvent& event) {
    render();
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
            qreal d = ((qreal)end.data[j] - (qreal)start.data[j]) * value;
            buf->SetChannel(j + start.offset, (uint8_t)d);
        }
    }
}

int OlaBridge::renderCmd(lighting_msgs::DmxCommand cmd, QDateTime start, QDateTime now){
    int activeFrames = 0;
    foreach(lighting_msgs::DmxFrame frame, cmd.layers){
        qreal deltaMs = start.msecsTo(now);
        qreal delayMs = (frame.delay.toSec() *  1000.0f);
        qreal durationMs = (frame.duration.toSec() *  1000.0f);

        qreal progress = (deltaMs - delayMs) / durationMs;

        if(progress >= 0.0f && progress <= 1.0f){
            //Render this frame

            foreach(lighting_msgs::DmxValue value, frame.values) {
                renderDmxValue(value);
            }

            foreach(lighting_msgs::DmxEasing easing, frame.easings) {
                qreal easingDelayMs = delayMs + (easing.delay.toSec() * 1000.0f);
                qreal easingDurationMs = easing.duration.toSec() * 1000.0f;

                qreal easingProgress = (deltaMs - easingDelayMs) / easingDurationMs;
                renderDmxEasing(easing, easingProgress);
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


bool OlaBridge::processCommand(lighting_msgs::run_command::Request& req, lighting_msgs::run_command::Response& res) {
    QDateTime now = QDateTime::currentDateTimeUtc();


    if(req.command.action == lighting_msgs::DmxCommand::DISPLAY){
        if( !req.command.layers.empty() ){
            //TODO: Validate command
            _currentCommand = req.command;
            _commandStartTime = now;

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

            return true;
        }
    }
    else if(req.command.action == lighting_msgs::DmxCommand::STOP){
        _currentCommand.action = lighting_msgs::DmxCommand::STOP;
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
