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
  this.tileSize = {
    x: opt.tileSize.x,
    y: opt.tileSize.y
  }

  this.tileCount = {
    x: opt.tileCount.x,
    y: opt.tileCount.y,
  }


  this.updateSize();

  this.defaultSelection = 0;
  this.defaultTileOpt = opt.defaultTileOpt;

  opt.width = this.size.x;
  opt.height = this.size.y;
  this.group = new Kinetic.Group(opt);

  console.log(this.globalName + JSON.stringify(opt));

  /*this.group.add(this.obj);*/

  for(var i=0; i<this.tileCount.x; i++){
    for(var j=0; j<this.tileCount.y; j++){
      this.getTile(i,j);
    }
  }

  this.layer = new Kinetic.Layer({x:0, y:0, width:opt.width, height: opt.height});
  this.layer.add(this.group);

  //this.update();
  this.setupEventHandlers();
  console.log(this);
  return this;
}


Ui.TileGrid.prototype.update = function(){
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

  this.tiles[x][y] = new Kinetic.Rect({
    x: this.margins.outer.x + (this.tileSize.x * x) + (this.margins.inner.x * x),
    y: this.margins.outer.y + (this.tileSize.y * y) + (this.margins.inner.y * y),
    width: this.tileSize.x,
    height: this.tileSize.y
  });

  var options = JSON.parse( JSON.stringify(this.defaultTileOpt) );

  var hsvColor = tinycolor(options.fill).toHsv();

  hsvColor.v = Math.random();
  hsvColor.s = Math.random();

  options.fill = tinycolor(hsvColor).toHexString();

  this.tiles[x][y].setAttrs(options);
  this.group.add( this.tiles[x][y] );
  return this.tiles[x][y];
}

Ui.TileGrid.prototype.getSelection = function(x,y){
  x = Math.round(x);
  y = Math.round(y);

  if( this.selections[x] !== undefined ){
    if( this.selections[x][y] !== undefined){
      return this.selections[x][y];
    }
    //
  }
  else{
    this.selections[x] = {};
  }

  this.selections[x][y] = this._defaultSelection;
  return this.selections[x][y];
}

Ui.TileGrid.prototype.setupEventHandlers = function(){
  /*this.group.on('mousemove touchmove',
    this.hoverCallback.bind(this)
  );*/
}
