//var Ui = {};

Ui.PolarInput = function(stage, options){
  this.stage = stage;
  this.name = options.name;
  this.globalName = "PolarInput." + this.name;
  this.margins = { inner: {x: 2, y: 2}, outer: {x: 0, y: 0} };

  this._hover = {
    x: 0,
    y: 0,
    radius: 10,
    enable: false
  }

  if(options.margins !== undefined){ margins=options.margins }
  if(options.value !== undefined){ this.value=options.value }

  this.group = new Kinetic.Group(options);

  console.log("PolarInput() - " + JSON.stringify(options));
  //console.log(this.group);

  this.radius = Math.min(this.group.width(), this.group.height())/2;
  this.center = {
    x: this.group.width() / 2,
    y: this.group.height() / 2
  };


  this.outsideCircle = new Kinetic.Circle({
    x: this.center.x,
    y: this.center.y,
    radius: this.radius,
    stroke: '#B2E9E2',
    strokeWidth: 1,
    strokeOpacity: 1,
    shadowEnabled: false,
    fillEnabled: true
  });

  this.hoverOutsideCircle = new Kinetic.Circle({
    radius: this._hover.radius,
    stroke: 'grey',
    strokeWidth: 1,
    strokeOpacity: 1,
    shadowEnabled: false,
    fillEnabled: false,
    dash: [3, 3]
  });
  this.hoverOutsideCircle.hide();

  this.hoverAngleKnuckle = new Kinetic.Arc({
    x: this.center.x,
    y: this.center.y,
    innerRadius: this.radius-4,
    outerRadius: this.radius-2,
    fill: '#B2E9E2',
    stroke: '#B2E9E2',
    strokeWidth: 1,
    angle: 15,
    rotationDeg: 0,
  });

  this.group.add(this.outsideCircle);
  this.group.add(this.hoverOutsideCircle);
  this.group.add(this.hoverAngleKnuckle);

  this.layer = new Kinetic.Layer(options);
  this.layer.add(this.group);

  this.setupEventHandlers();
  return this;
}


Ui.PolarInput.prototype.update = function(){
  this.outsideCircle.setAttrs({
    x: this.center.x,
    y: this.center.y,
    radius: this.radius
  });

  if(this._hover.enable){

    this.hoverOutsideCircle.setAttrs({
      x: this._hover.x,
      y: this._hover.y
    });

  }

  this.group.draw();
  this.layer.draw();
  this.group.fire('draw');
}

Ui.PolarInput.prototype.getPosValue = function(x, y){
  var val = {
    r: Math.sqrt( Math.pow(this.center.x - x, 2) + Math.pow(this.center.y -y, 2) ) / this.radius,
    theta: -Math.atan2(y - this.center.y, x - this.center.x)
  };

  if(val.theta < 0){
    val.theta += 2*Math.PI;
  }

  val.theta = val.theta / (2 * Math.PI)

  return val;
}

Ui.PolarInput.prototype.setHoverXY = function(x, y){
  this._hover.x = x;
  this._hover.y = y;
  this._hover.enable = true;
  this.hoverOutsideCircle.show();
  this.update();
  Ui.emitter.emit('change.'+this.globalName+'.hover', this.getHoverValue());
}

Ui.PolarInput.prototype.getHoverValue = function(){
  return this.getPosValue( this._hover.x, this._hover.y );
}

Ui.PolarInput.prototype.inputEndCallback = function(evt){
  this._hover.enable = false;
  this.hoverOutsideCircle.hide();
  this.update();

}

Ui.PolarInput.prototype.hoverCallback = function(evt){
  var pos = this.stage.getPointerPosition();
  pos.x -= (this.group.getLayer().x() + this.group.x());
  pos.y -= (this.group.getLayer().y() + this.group.y());


  this.setHoverXY(pos.x, pos.y);
  var angle = 360 - (this.getHoverValue().theta * 360);
  this.hoverAngleKnuckle.setAttr("rotationDeg", angle - (this.hoverAngleKnuckle.angle()/2));

  evt.cancelBubble = false;
};

Ui.PolarInput.prototype.clickCallback = function(evt){
  var pos = this.stage.getPointerPosition();
  pos.x -= (this.group.getLayer().x() + this.group.x());
  pos.y -= (this.group.getLayer().y() + this.group.y());

  this.setHoverXY(pos.x, pos.y);
  var angle = 360 - (this.getHoverValue().theta * 360);
  this.hoverAngleKnuckle.setAttr("rotationDeg", angle - (this.hoverAngleKnuckle.angle()/2));

  Ui.emitter.emit('change.'+this.globalName, this.getPosValue(pos.x, pos.y));
  evt.cancelBubble = false;
};


Ui.PolarInput.prototype.setupEventHandlers = function(){
  this.group.on('mousemove',
    this.hoverCallback.bind(this)
  );

  /*this.group.on('mouseleave',
    this.inputEndCallback.bind(this)
  );*/

  this.group.on('click touchstart touchmove',
    this.clickCallback.bind(this)
  );
}
