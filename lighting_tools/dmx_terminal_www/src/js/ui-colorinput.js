//var Ui = {};

Ui.DmxMoverControl = function(stage, options){
  this.stage = stage;
  this.device = options.device;
  this.name = options.name;
  this.globalName = "DmxMoverControl." + this.name;
  this.margins = { inner: {x: 0, y: 10}, outer: {x: 10, y: 10} };
  if(options.margins !== undefined){
    if(options.margins.inner !== undefined){
      this.margins.inner = options.margins.inner;
    }

    if(options.margins.outer !== undefined){
      this.margins.outer = options.margins.outer;
    }
  }

  this.layer = new Kinetic.Layer(options);

  console.log("DmxMoverControl() - " + JSON.stringify(options));
  //console.log(this.layer);

  var config = {
    x: this.margins.outer.x,
    y: this.margins.outer.y,
    offsetX: options.offsetX
  }

  //p,t,z,i,w

  this.titleText = new Kinetic.Text({
    x: config.x,
    y: config.y,
    offsetX: options.offsetX,
    text: this.name,
    fontSize: 20,
    fontFamily: 'monospace',
    fill: '#B2E9E2'
  });
  this.layer.add(this.titleText);

  this.addrText = new Kinetic.Text({
    x: config.x,
    y: config.y+21,
    offsetX: options.offsetX,
    text: 'dmx: ' + this.device.address.toString(),
    fontSize: 8,
    fontFamily: 'monospace',
    fill: '#B2E9E2'
  });
  this.layer.add(this.addrText);

  this.fieldsText = new Kinetic.Text({
    x: config.x,
    y: config.y+30,
    offsetX: options.offsetX,
    text: 'fields: ' + this.device.getFieldNames().length,
    fontSize: 8,
    fontFamily: 'monospace',
    fill: '#B2E9E2'
  });
  this.layer.add(this.fieldsText);

  config.y += 30;

  var barSize = (this.layer.height() - (2 * this.margins.outer.y))/4;
  var polarSize = ((barSize * 3) - (2 * this.margins.outer.y));

  config.width = this.layer.width() - (2 * this.margins.outer.x);
  config.height = Math.min(polarSize, config.width);

  this.image = new K

  config.name = this.name + '.pan-tilt';
  this.polarInput = new Ui.PolarInput(this.stage, config);
  this.addGroup(this.polarInput.group);

  // Zoom
  config.y += config.height + this.margins.inner.y;
  config.height = Math.floor((barSize / 3) / 5.0)*5;
  config.name = this.name + '.zoom';
  this.zoomBar = new Ui.BarInput(this.stage, config);
  this.addGroup(this.zoomBar.group);

  // Intensity
  config.y += config.height + this.margins.inner.y;
  config.name = this.name + '.intensity';
  this.intensityBar = new Ui.BarInput(this.stage, config);
  this.addGroup(this.intensityBar.group);

  // White
  config.y += config.height + this.margins.inner.y;
  config.name = this.name + '.white';
  this.whiteBar = new Ui.BarInput(this.stage, config);

  //this.whiteBar.interiorRect.setAttrs( {'fill': 'red' } );
  //this.polarInput.hoverOutsideCircle.setAttrs( {'stroke': 'red' } );

  this.addGroup(this.whiteBar.group);

  this.setupEventHandlers();
  return this;
}


Ui.DmxMoverControl.prototype.update = function(){
  this.layer.draw();
  //console.log("DmxMoverControl - draw");
}


Ui.DmxMoverControl.prototype.addGroup = function(g){
  this.layer.add(g);
  g.on('draw',
    function(){
      this.update();
    }.bind(this)
  );
}

Ui.DmxMoverControl.prototype.modelChangeHandler = function(field){
  var scaledValue = (field.value - field.min) / (field.max - field.min);
  if(field.name == 'zoom'){
    this.zoomBar.setValue(scaledValue);
  }
  else if(field.name == 'intensity'){
    this.intensityBar.setValue(scaledValue);
  }
  else if(field.name == 'white'){
    this.whiteBar.setValue(scaledValue);
  }
}

Ui.DmxMoverControl.prototype.setupEventHandlers = function(){
  var barPath = 'change.BarInput.'+this.name + '.';
  var device = this.device;
  Ui.emitter.on( barPath + '*',
    function(value){
      //console.log(this.event);
      //console.log(value);
      var fieldName = this.event.replace(barPath, '');
      field = device.getField(fieldName);

      //console.log('setting field: ' + device.name + '.' + fieldName);

      device.set(field.name, value);
      device.update();
    }
  );

  var polarPath = 'change.PolarInput.'+this.name+'.pan-tilt';
  Ui.emitter.on( polarPath,
    function(value){
      console.log('polar input changed');
      console.log(value);

      device.set('pan', value.theta);
      device.set('tilt', (1-value.r) / 2);
      device.update();
    }
  );

  polarPath = 'change.PolarInput.'+this.name+'.pan-tilt.hover';
  Ui.emitter.on( polarPath,
    function(value){
      console.log('polar input changed');
      console.log(value);

      device.set('pan', value.theta);
      device.set('tilt', (1-value.r) / 2);
      device.update();
    }
  );

  //Handle model changes
  var that = this;
  var devPath = 'change.Device.'+device.name;
  Dmx.emitter.on(devPath,
    function(field){

      that.modelChangeHandler.bind(that)(field);
      //console.log(field);
    }
  );
}
