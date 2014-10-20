var Dmx={
  emitter: new EventEmitter2({wildcard: true})/*,
  ros: new ROSLIB.Ros({
    url : 'ws://' + window.location.hostname + ':4000'
  })*/
};

/*
$( document ).ready(
function() {
  if(Dmx.ros !== undefined){
    Dmx.ros.on('connection',
      function(){
        console.log('Dmx.ros: connected');
        Dmx.factory = new ROSUtils.MessageFactory(Dmx.ros);
        Dmx.factory.getMessageDetails('lighting_msgs/DmxCommand',
          function(type){
            console.log("type: " + type + " ready");
            Dmx.emitter.emit('ready');
          }
        );
      }
    );

    Dmx.ros.on('error',
      function(evt){
        console.log("ERROR - Dmx.ros: ");
        console.log(evt);
      }
    );
  }
});
*/

Dmx.Address = function(options){
  this.universe = 0;
  this.offset = 0;

  if(options !== undefined && options != null){
    this.set(options);
  }
}

Dmx.Address.prototype.toString = function(){
  return this.universe + '.' + this.offset;
}

Dmx.Address.prototype.set = function(options){
  if(typeof options == 'string'){
    var tokens = options.split('.');
    if(tokens.length == 2){
      this.universe = parseInt(tokens[0]);
      this.offset = parseInt(tokens[1]);
    }
  }
  else{
    if(options.universe !== undefined){
      this.universe = options.universe;
    }

    if(options.offset !== undefined){
      this.offset = options.offset;
    }
  }
}

Dmx.Field = function(options){
  this.offset = 0;
  this.bytes = 1;
  this.chunkSize = 1;
  this.name = '';
  /*this.min = 0;
  this.default = 0;
  this.max = Math.pow(2, 8*this.bytes)-1;*/
}

Dmx.Field.prototype.computeValue = function(value){
  if(value.length !== undefined && value.length > 0){
    return value;
  }

  if(value > 0.0 && value <= 1.0) {
    value = Math.round(value * f.max);
  }
  else{
    value = Math.round(value);
  }

  if(value < f.min){
    throw "value too low";
  }

  if(value > f.max){
    throw "value too low";
  }

  console.log(value);

  var arr=new Array(this.bytes);
  for(var i=0; i<this.bytes; i++){
    arr[this.bytes - (1+i)] = (value >> i*8) & 255;
    //console.log(byte);
    //arr.push(byte);
  }

  return arr;
}

Dmx.DeviceTemplate = function(options){
  this.fields = {};
  this.name = '';
}

Dmx.DeviceTemplate.prototype.getField = function(name){
  if(this.fields[name] === undefined){
    //throw "No such field [" + name + "]";
    return undefined;
  }

  return this.fields[name];
}

Dmx.DeviceTemplate.prototype.addField = function(field){
  this.fields[field.name] = field;
  return this.fields[field.name];
}

Dmx.Device = function(options){
  this.name = options.name;
  this.address = new Dmx.Address(options.address);
  this.template = new Dmx.DeviceTemplate(options.type);
  this.values = {};  //Mapped by template.fields.name
}

Dmx.Device.prototype.getField = function(name){
  return this.template.getField(name);
}

Dmx.Device.prototype.getFieldNames = function(){
  var names=[];
  for(var idx in this.template.fields){
    names.push(idx);
  }
  return names;
}

Dmx.Device.prototype.update = function(){
  Dmx.emitter.emit('update.Device.'+this.name, this);
}

Dmx.Device.prototype.set = function(name, value){
  console.log("Dmx.Device.setField() - " + name + ',' + value)
  f = this.getField(name);
  this.values[f.name] = value;

  Dmx.emitter.emit('change.Device.'+this.name, {device:this, field:f});
}


Dmx.CommandClient = function(ros, servicePath){
  if(servicePath === undefined){
    servicePath = '/ola_bridge/dmx_cmd';
  }
  this.srvClient = new ROSLIB.Service({
    ros : ros,
    name : servicePath,
    serviceType : 'lighting_msgs/dmx_command'
  });

  this.command = {};

  this.mode = 'replace';
}

Dmx.CommandClient.prototype.add = function(dev){
  console.log("CommandClient - add device " + dev.name);
  /*Dmx.emitter.on('change.Device.'+dev.name,
    function(evt){
      this.set(evt.device, evt.field);
    }.bind(this)
  );*/

  Dmx.emitter.on('update.Device.'+dev.name,
    function(dev){
      this.setDevice(dev);
      this.send();
    }.bind(this)
  );
}

Dmx.CommandClient.prototype.setDevice = function(dev){
  console.log("Dmx.CommandClient.setDevice");
  this.command = Dmx.factory.createMessage('lighting_msgs/DmxCommand');

  var frame = Dmx.factory.createMessage('lighting_msgs/DmxFrame');

  var fieldNames = dev.values;
  for(idx in fieldNames){
    var dmxValue = Dmx.factory.createMessage('lighting_msgs/DmxValue');
    var field = dev.getField(idx);

    if(field === undefined){
      console.log("not found: " + idx)
      continue;
    }

    dmxValue.universe = dev.address.universe;
    dmxValue.offset = dev.address.offset + field.offset;

    var val = dev.values[field.name];

    if(val.length != field.bytes){
      console.log(dev)
      console.log(field)
      throw "DMX data format error (expected " + field.bytes + "bytes but recieved " + val.length + ")";
    }

    for(i=0; i<field.length; i++){
      for(j=0; j<val.length; j++){
        dmxValue.data.push(val[j]);
      }
    }

    frame.values.push(dmxValue);
  }

  frame.durationMs = 1000;

  this.command.layers = [frame];
}

Dmx.CommandClient.prototype.set = function(dev, field){
  console.log("Dmx.CommandClient.set");
  if(this.mode == 'replace'){
    this.command = Dmx.factory.createMessage('lighting_msgs/DmxCommand');

    var frame = Dmx.factory.createMessage('lighting_msgs/DmxFrame');
    var dmxValue = Dmx.factory.createMessage('lighting_msgs/DmxValue');

    console.log(dev);

    dmxValue.universe = dev.address.universe;
    dmxValue.offset = dev.address.offset + field.offset;

    var val = dev.values[field.name];

    if(val.length != field.bytes){
      console.log(dev)
      console.log(field)
      throw "DMX data format error (expected " + field.bytes + "bytes but recieved " + val.length + ")";
    }

    for(i=0; i<field.length; i++){
      dmxValue.data.push(val[i]);
    }

    frame.values.push(dmxValue);
    frame.durationMs = 1000;

    this.command.layers = [frame];
  }
}

Dmx.CommandClient.prototype.send = function(){
  var request = new ROSLIB.ServiceRequest({
    command : this.command
  });

  console.log("Dmx.CommandClient.send()");
  console.log(this.command);

  this.srvClient.callService(request,
    function(result) {
      console.log('Dmx.CommandClient: Result for service call on: ' + result.status);
      console.log(result);
    },
    function(err){
      console.log("ERROR - Dmx.CommandClient: " + err);
    }
  );
}
