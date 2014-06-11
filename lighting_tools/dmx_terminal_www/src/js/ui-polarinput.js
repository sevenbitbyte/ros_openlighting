//var Ui = {};

Ui.PolarInput = function(stage, options){
  this.stage = stage;
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
    stroke: '#B2E9E2',
    strokeWidth: 1,
    strokeOpacity: 1,
    shadowEnabled: false,
    fillEnabled: false
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
    rotationDeg: 0
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
  return {
    r: Math.sqrt( Math.pow(this.center.x, 2) + Math.pow(this.center.y, 2) ),
    theta: Math.atan2(pos.y - this.center.y, pos.x - this.center.x) * (180.0/Math.PI)
  };
}

Ui.PolarInput.prototype.setHoverXY = function(x, y){
  this._hover.x = x;
  this._hover.y = y;
  this._hover.enable = true;
  this.hoverOutsideCircle.show();
  this.update();
  this.group.fire('hoverchanged');
}

Ui.PolarInput.prototype.getHoverValue = function(){
  return this.getPosValue( this._hover.x, this._hover.y );
}

Ui.PolarInput.prototype.inputEndCallback = function(evt){
  this._hover.enable = false;
  this.hoverOutsideCircle.hide();
  this.render();

  console.log("Input end");
}

Ui.PolarInput.prototype.hoverCallback = function(evt){

  var pos = this.stage.getPointerPosition();
  pos.x -= (this.group.getLayer().x() + this.group.x());
  pos.y -= (this.group.getLayer().y() + this.group.y());

  var angle = Math.atan2(pos.y - this.center.y, pos.x - this.center.x) * (180.0/Math.PI);
  //console.log("Hover: " + pos.x + ", " + pos.y + " angle=" + angle);

  this.hoverAngleKnuckle.setAttr("rotationDeg", angle - (this.hoverAngleKnuckle.angle()/2));

  this.setHoverXY(pos.x, pos.y);
  evt.cancelBubble = false;
};


Ui.PolarInput.prototype.setupEventHandlers = function(){
  this.group.on('mousemove touchmove',
    this.hoverCallback.bind(this)
  );

  this.group.on('click tap',
    this.hoverCallback.bind(this)
  );
}
