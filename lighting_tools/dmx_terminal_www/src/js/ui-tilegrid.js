//

Ui.TileGrid = function(stage, opt){
  this.stage = stage;
  this.name = opt.name;
  this.globalName = "TileGrid." + this.name;

  this.margins = { inner: {x:1, y: 1}, outer: {x: 0, y: 0} };
  if(opt.margins !== undefined){
    if(opt.margins.inner !== undefined){
      this.margins.inner = opt.margins.inner;
    }

    if(opt.margins.outer !== undefined){
      this.margins.outer = opt.margins.outer;
    }
  }

  this.tiles = {};
  this.selections = {};       // x -> y -> id:Number
  this.selectionOpts = {};    // id -> {fill:..., stroke:...}
  this.defaultSelection = 1;
  if(opt.defaultTileOpt){
    this.defaultTileOpt = opt.defaultTileOpt;
  }
  else{
    this.defaultTileOpt = {fill: 'grey'};
  }

  this.selectionOpts[1] = {fill: 'blue', fillEnabled:true};

  this.tileSize = {
    x: opt.tileSize.x,
    y: opt.tileSize.y
  }

  this.tileCount = {
    x: opt.tileCount.x,
    y: opt.tileCount.y,
  }


  this.updateSize();

  this._hover = {
    x: 0,
    y: 0,
    width: this.tileSize.x,
    height: this.tileSize.y,
    enable: false
  }



  opt.width = this.size.x;
  opt.height = this.size.y;
  this.group = new Kinetic.Group(opt);

  console.log(this.globalName + JSON.stringify(opt));

  /*this.group.add(this.obj);*/

  for(var i=0; i<this.tileCount.x; i++){
    for(var j=0; j<this.tileCount.y; j++){
      this.createTile(i,j);
    }
  }

  this.hoverOutsideRect = new Kinetic.Rect({
    width : this._hover.width,
    height : this._hover.height,
    stroke : 'yellow',
    strokeWidth: this.margins.x,
    dash : [3, 3]
  });

  this.hoverOutsideRect.hide();
  this.group.add(this.hoverOutsideRect);

  this.layer = new Kinetic.Layer({x:0, y:0, width:opt.width, height: opt.height});
  this.layer.add(this.group);

  //this.update();
  this.setupEventHandlers();
  return this;
}


Ui.TileGrid.prototype.update = function(){

  if(this._hover.enable){
    var pos = this.tilePosToLayer( this.layerPosToTile(this._hover));
    this.hoverOutsideRect.setAttrs(pos);
  }

  for(x in this.selections){
    for(y in this.selections[x]){
      //this.getTile(x,y).setAttrs( this.selectionOpts[this.getSelection()] );
      this.getTile(x,y).draw();
      //console.log('drew selection ' + x + ',' + y);
    }
  }

  this.group.draw();
  this.layer.draw();
  this.group.fire('draw');
}

Ui.TileGrid.prototype.updateSize = function() {
  this.size = {
    x: (this.tileSize.x * this.tileCount.x) + (this.margins.inner.x * (this.tileCount.x - 1)),
    y: (this.tileSize.y * this.tileCount.y) + (this.margins.inner.y * (this.tileCount.y - 1))
  };

  this.size.x += this.margins.outer.x * 2;
  this.size.y += this.margins.outer.y * 2;
}

Ui.TileGrid.prototype.tilePosToLayer = function(tilePos){
  return {
    x: this.margins.outer.x + (this.tileSize.x + this.margins.inner.x) * tilePos.x,
    y: this.margins.outer.y + (this.tileSize.y + this.margins.inner.y) * tilePos.y
  }
}

Ui.TileGrid.prototype.layerPosToTile = function(layerPos){
  return {
    x: Math.floor((layerPos.x - this.margins.outer.x) / (this.tileSize.x + this.margins.inner.x)),
    y: Math.floor((layerPos.y - this.margins.outer.y) / (this.tileSize.y + this.margins.inner.y)),
  }
}

Ui.TileGrid.prototype.createTile = function(x,y){
  x = Math.round(x);
  y = Math.round(y);

  if( this.tiles[x] === undefined ){
    this.tiles[x] = {};
  }

  var pos = this.tilePosToLayer({x:x, y:y});
  this.tiles[x][y] = new Kinetic.Rect({
    x: pos.x,
    y: pos.y,
    width: this.tileSize.x,
    height: this.tileSize.y
  });

  this.tiles[x][y].setAttrs(this.defaultTileOpt);
  this.group.add( this.tiles[x][y] );
  return this.tiles[x][y];
}

Ui.TileGrid.prototype.getTile = function(x,y){
  x = Math.round(x);
  y = Math.round(y);

  if( this.tiles[x] !== undefined ){
    if( this.tiles[x][y] !== undefined){
      return this.tiles[x][y];
    }
    //
  }
  else{
    this.tiles[x] = {};
  }

  return undefined;
}

Ui.TileGrid.prototype.getSelection = function(x,y){
  x = Math.floor(x);
  y = Math.floor(y);

  if( this.selections[x] !== undefined ){
    if( this.selections[x][y] !== undefined){
      return this.selections[x][y];
    }
    //
  }
  else{
    this.selections[x] = {};
  }
  return undefined;
}

Ui.TileGrid.prototype.setSelection = function(x,y,val){
  x = Math.floor(x);
  y = Math.floor(y);

  if( this.selections[x] !== undefined ){
    if( this.selections[x][y] === undefined){
      this.selections[x][y] = {};
    }
  }
  else{
    this.selections[x] = {};
    this.selections[x][y] = {};
  }


  if(val !== undefined){
    this.selections[x][y] = val;
  }
  else{
    this.selections[x][y] = this.defaultSelection;
  }
  console.log(x+','+y+' selectoion=' + this.selections[x][y])
  console.log(this.selectionOpts[this.selections[x][y]]);

  this.getTile(x,y).setAttrs( this.selectionOpts[this.selections[x][y]] );
}

Ui.TileGrid.prototype.removeSelection = function(x,y){
  if( this.selections[x][y] !== undefined ){
    delete this.selections[x][y];
    this.getTile(x,y).setAttrs( this.defaultTileOpt );
  }
}


Ui.TileGrid.prototype.setHoverXY = function(x, y){
  this._hover.x = x;
  this._hover.y = y;
  this._hover.enable = true;
  this.hoverOutsideRect.show();
  this.update();
}

Ui.TileGrid.prototype.getHoverValue = function(){
  return { x: this._hover.x, y: this._hover.y  };
}

Ui.TileGrid.prototype.inputEndCallback = function(evt){
  this._hover.enable = false;
  this.hoverOutsideRect.hide();
  this.update();

  evt.cancelBubble = false;
}

Ui.TileGrid.prototype.selectCallback = function(evt){

  console.log('selected callbacl');
  var pos = this.stage.getPointerPosition();

  if(pos === undefined){
    return;
  }

  pos.x -= (this.group.getLayer().x() + this.group.x());
  pos.y -= (this.group.getLayer().y() + this.group.y());

  if(pos.x < 0 || pos.x > this.size.x-1){
    evt.cancelBubble = false;
    return;
  }
  else if(pos.y < 0 || pos.y > this.size.y-1){
    evt.cancelBubble = false;
    return;
  }

  pos = this.layerPosToTile(pos);

  if( this.getSelection(pos.x, pos.y) === undefined){
    this.setSelection(pos.x, pos.y);
    console.log('selected ' + pos.x + ',' + pos.y);
  }
  else{
    this.removeSelection(pos.x, pos.y);
    console.log('removed selection ' + pos.x + ',' + pos.y);
  }

  

  this.update();

  Ui.emitter.emit('select.'+this.globalName, pos);
}

Ui.TileGrid.prototype.hoverCallback = function(evt){
  var pos = this.stage.getPointerPosition();
  pos.x -= (this.group.getLayer().x() + this.group.x());
  pos.y -= (this.group.getLayer().y() + this.group.y());

  if(pos.x < 0 || pos.x > this.size.x-1){
    evt.cancelBubble = false;
    return;
  }
  else if(pos.y < 0 || pos.y > this.size.y-1){
    evt.cancelBubble = false;
    return;
  }

  this.setHoverXY(pos.x, pos.y);
  Ui.emitter.emit('change.'+this.globalName+'.hover', this.getHoverValue());
  evt.cancelBubble = false;
};

Ui.TileGrid.prototype.setupEventHandlers = function(){
  this.group.on('mousemove touchmove',
    this.hoverCallback.bind(this)
  );

  this.group.on('mouseleave touchend',
    this.inputEndCallback.bind(this)
  );

  this.group.on('click tap',
    this.selectCallback.bind(this)
  );
}
