/*
 *  A class that draws a timeline with ui elements added at specific locations
 *
 *
 */

Ui.Timeline = function(stage, opt){
  this.stage = stage;
  this.name = opt.name;
  this.globalName = "Timeline." + this.name;

  this.margins = { inner: {x:1, y: 1}, outer: {x: 0, y: 0} };
  if(opt.margins !== undefined){
    if(opt.margins.inner !== undefined){
      this.margins.inner = opt.margins.inner;
    }

    if(opt.margins.outer !== undefined){
      this.margins.outer = opt.margins.outer;
    }
  }

  this.size = {x: opt.width, y: opt.height};
  this.startTime = opt.start;
  this.endTime = opt.end;
  this.eventFunc = opt.eventFunc;   // A function(start,end)
}