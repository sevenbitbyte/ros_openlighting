//var Ui = {};

Ui.BarInput = function(stage, options){
  this.stage = stage;
  this.name = options.name;
  this.globalName = "BarInput." + this.name;
  this.margins = { inner: {x: 2, y: 2}, outer: {x: 0, y: 0} };
  this.value = 1.0;

  if(options.margins !== undefined){ margins=options.margins }
  if(options.value !== undefined){ this.value=options.value }

  this.group = new Kinetic.Group(options);

  console.log("BarInput() - " + JSON.stringify(options));
  //console.log(this.group);

  //Exterior
  this.exteriorRect = new Kinetic.Rect({
    x: this.margins.outer.x,
    y: this.margins.outer.y,
    width: this.group.width() - (2*this.margins.outer.x),
    height: this.group.height()  - (2*this.margins.outer.y),
    stroke: 'grey',
    strokeWidth: 1.0,
    shadowEnabled: false,
    cornerRadius: 0.0
  });

  //Interior
  this.interiorRect = new Kinetic.Rect({
    x: this.exteriorRect.x() + this.margins.inner.x,
    y: this.exteriorRect.y() + this.margins.inner.y,
    width: (this.exteriorRect.width() - (2*this.margins.inner.x)) * this.value,
    height: this.exteriorRect.height() - (2*this.margins.inner.y),
    fill: '#B2E9E2',
    shadowEnabled: false,
    cornerRadius: 0.0
  });

  this.group.add(this.exteriorRect);
  this.group.add(this.interiorRect);

  this._click_started = false;
  this.group.on('mousedown touchstart',
    function(evt){
      this._click_started = true;

      var pos = this.stage.getPointerPosition();
      pos.x -= (this.group.getLayer().x() + this.group.x());
      pos.y -= (this.group.getLayer().y() + this.group.y());

      this.setPos(pos);
      Ui.emitter.emit('change.'+this.globalName, this.value);
      evt.cancelBubble = true;
    }.bind(this)
  );

  this.group.on('tap',
    function(evt){
      var pos = this.stage.getPointerPosition();
      pos.x -= (this.group.getLayer().x() + this.group.x());
      pos.y -= (this.group.getLayer().y() + this.group.y());

      this.setPos(pos);
      Ui.emitter.emit('change.'+this.globalName, this.value);
      evt.cancelBubble = true;
    }.bind(this)
  );

  this.group.on('mouseup mouseleave touchend',
    function(evt){
      if(this._click_started){
        var pos = this.stage.getPointerPosition();
        pos.x -= (this.group.getLayer().x() + this.group.x());
        pos.y -= (this.group.getLayer().y() + this.group.y());

        this.setPos(pos);
        Ui.emitter.emit('change.'+this.globalName, this.value);
        evt.cancelBubble = true;
      }


      this._click_started = false;
    }.bind(this)
  );

  this.group.on('mousemove touchmove',
    function(evt){
      if(!this._click_started){return;}

      var pos = this.stage.getPointerPosition();
      pos.x -= (this.group.getLayer().x() + this.group.x());
      pos.y -= (this.group.getLayer().y() + this.group.y());

      this.setPos(pos);
      Ui.emitter.emit('change.'+this.globalName, this.value);
      evt.cancelBubble = true;
    }.bind(this)
  );


this._wireframe = new Kinetic.Rect({
  x: 0,
  y: 0,
  width: this.group.width(),
  height: this.group.height(),
  stroke: 'white',
  strokeWidth: 1.0,
  shadowEnabled: false,
  cornerRadius: 0.0
});

  this.layer = new Kinetic.Layer(options);
  this.layer.add(this.group);
  return this;
}

Ui.BarInput.prototype.update = function(){
  this.exteriorRect.x(this.margins.outer.x);
  this.exteriorRect.y(this.margins.outer.y);
  this.exteriorRect.width(this.group.width() - (2*this.margins.outer.x));
  this.exteriorRect.height(this.group.height()  - (2*this.margins.outer.y));

  this.interiorRect.x(this.exteriorRect.x() + this.margins.inner.x);
  this.interiorRect.y(this.exteriorRect.y() + this.margins.inner.y);
  this.interiorRect.width((this.exteriorRect.width() - (2*this.margins.inner.x)) * this.value);
  this.interiorRect.height(this.exteriorRect.height() - (2*this.margins.inner.y));

  this.group.draw();
  this.layer.draw();
  this.group.fire('draw');
}

Ui.BarInput.prototype.setPos = function(pos){
  this.setValue( pos.x / this.group.width() );
}

Ui.BarInput.prototype.setValue = function(value){
  this.value = Math.max(0.0, Math.min(1.0, value));
  this.update();
}
