//var Ui = {};

Ui.DmxMoverControl = function(stage, options){
  this.stage = stage;
  this.margins = { inner: {x: 0, y: 10}, outer: {x: 10, y: 10} };



  if(options.margins !== undefined){ margins=options.margins }
  if(options.value !== undefined){ this.value=options.value }

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
    text: 'mover-00',
    fontSize: 20,
    fontFamily: 'monospace',
    fill: '#B2E9E2'
  });
  this.layer.add(this.titleText);

  config.y += 20 + this.margins.inner.y*0.75;

  var barSize = (this.layer.height() - (2 * this.margins.outer.y))/4;
  var polarSize = ((barSize * 3) - (2 * this.margins.outer.y));

  config.width = this.layer.width() - (2 * this.margins.outer.x);
  config.height = Math.min(polarSize, config.width);
  this.polarInput = new Ui.PolarInput(this.stage, config);
  this.addGroup(this.polarInput.group);

  // Zoom
  config.y += config.height + this.margins.inner.y;
  config.height = Math.floor((barSize / 3) / 5.0)*5;
  this.zoomBar = new Ui.InputBar(this.stage, config);
  this.addGroup(this.zoomBar.group);

  // Intensity
  config.y += config.height + this.margins.inner.y;
  this.intensityBar = new Ui.InputBar(this.stage, config);
  this.addGroup(this.intensityBar.group);

  // Intensity
  config.y += config.height + this.margins.inner.y;
  this.whiteBar = new Ui.InputBar(this.stage, config);

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

Ui.DmxMoverControl.prototype.setupEventHandlers = function(){
  //
}
