var Dmx={
  emitter: new EventEmitter2({wildcard: true}),
  ros: new ROSLIB.Ros({
    url : 'ws://' + window.location.host + ':4000'
  })
};

$( document ).ready(
function() {
  if(Dmx.ros !== undefined){
    Dmx.ros.on('connection',
      function(){
        console.log('connected');
        Dmx.factory = new ROSUtils.MessageFactory(Dmx.ros);
        Dmx.factory.getMessageDetails('lighting_msgs/DmxCommand');
      }
    );
  }
});

Dmx.Address = function(options){
  this.universe = 0;
  this.offset = 0;

  if(options !== undefined && options != null){
    this.set(options);
  }
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
  this.length = 1;
  this.bytes = 1;
  this.name = '';
  this.min = 0;
  this.default = 0;
  this.max = Math.pow(2, 8*this.bytes)-1;
  this.value = this.default;
}

Dmx.DeviceTemplate = function(options){
  this.fields = {};
  this.name = '';
}

Dmx.DeviceTemplate.prototype.getField = function(name){
  if(this.fields[name] === undefined){
    throw "No such field [" + name + "]";
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
}

Dmx.Device.prototype.getField = function(name){
  return this.template.getField(name);
}

Dmx.Device.prototype.setField = function(name, value){
  console.log("Dmx.Device.setField() - " + name + ',' + value)
  f = this.getField(name);

  if(value > 0.0 && value <= 1.0) {
    f.value = (value * f.max);
  }
  else{
    f.value = value;
  }

  Dmx.emitter.emit('change.Device.'+this.name, f);
}
