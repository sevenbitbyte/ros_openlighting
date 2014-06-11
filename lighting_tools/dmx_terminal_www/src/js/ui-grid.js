//var Ui = {};

Ui.Grid = function(stage, options){
  this.stage = stage;
  this.margins = { inner: {x: 2, y: 2}, outer: {x: 0, y: 0} };
  this.inc = {
    x: 10,
    y: 10
  }

  if(options !== undefined && options.margins !== undefined){ margins=options.margins }

  this.layer = new Kinetic.Layer(options);
  this.circles = [];

  var stepsX = this.layer.width() / this.inc.x;
  var stepsY = this.layer.height() / this.inc.y;


  console.log("Grid - Drawing " + this.stepsX * this.stepsY);

  for(var i=0; i < stepsX; i++){
    for(var j=0; j < stepsY; j++){
      var c = new Kinetic.Circle({
        x: i * this.inc.x,
        y: j * this.inc.y,
        radius: 1,
        fill: 'white',
        fillAlpha: 0.1
      });

      this.circles.push(c);
      this.layer.add(c);
    }
  }

  //this.stage.add(this.layer);

  return this;
}
