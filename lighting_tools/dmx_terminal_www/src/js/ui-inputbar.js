//var Ui = {};

Ui.InputBar = function(stage, options){
  this.stage = stage;
  this.margins = { inner: {x: 2, y: 2}, outer: {x: 0, y: 0} };
  this.value = 1.0;

  if(options.margins !== undefined){ margins=options.margins }
  if(options.value !== undefined){ this.value=options.value }

  this.group = new Kinetic.Group(options);

  console.log("InputBar() - " + JSON.stringify(options));
  console.log(this.group);

  //Exterior
  this._rect1 = new Kinetic.Rect({
    x: this.margins.outer.x,
    y: this.margins.outer.y,
    width: this.group.width() - (2*this.margins.outer.x),
    height: this.group.height()  - (2*this.margins.outer.y),
    stroke: 'grey',
    strokeWidth: 1.0,
    shadowEnabled: false,
    cornerRadius: 0.0
  });

  console.log(this._rect1.y());

  //Interior
  this._rect2 = new Kinetic.Rect({
    x: this._rect1.x() + this.margins.inner.x,
    y: this._rect1.y() + this.margins.inner.y,
    width: (this._rect1.width() - (2*this.margins.inner.x)) * this.value,
    height: this._rect1.height() - (2*this.margins.inner.y),
    fill: '#B2E9E2',
    shadowEnabled: false,
    cornerRadius: 0.0
  });

  this.group.add(this._rect1);
  this.group.add(this._rect2);

  this._click_started = false;
  this.group.on('mousedown touchstart',
    function(evt){
      this._click_started = true;

      var pos = this.stage.getPointerPosition();
      pos.x -= (this.group.getLayer().x() + this.group.x());
      pos.y -= (this.group.getLayer().y() + this.group.y());

      this.setPos(pos);
      evt.cancelBubble = true;
    }.bind(this)
  );

  this.group.on('tap',
    function(evt){
      var pos = this.stage.getPointerPosition();
      pos.x -= (this.group.getLayer().x() + this.group.x());
      pos.y -= (this.group.getLayer().y() + this.group.y());

      this.setPos(pos);
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

Ui.InputBar.prototype.update = function(){
  this._rect1.x(this.margins.outer.x);
  this._rect1.y(this.margins.outer.y);
  this._rect1.width(this.group.width() - (2*this.margins.outer.x));
  this._rect1.height(this.group.height()  - (2*this.margins.outer.y));

  this._rect2.x(this._rect1.x() + this.margins.inner.x);
  this._rect2.y(this._rect1.y() + this.margins.inner.y);
  this._rect2.width((this._rect1.width() - (2*this.margins.inner.x)) * this.value);
  this._rect2.height(this._rect1.height() - (2*this.margins.inner.y));

  this.group.draw();
  this.layer.draw();
  this.group.fire('draw');
}

Ui.InputBar.prototype.setPos = function(pos){
  this.setValue( pos.x / this.group.width() );
}

Ui.InputBar.prototype.setValue = function(value){
  this.value = Math.max(0.0, Math.min(1.0, value));
  this.update();
  this.group.fire('valuechanged', null, false);
}
