//var Ui = {};

Ui.StageView = function(options){
  this.container = options.container;

  options.container = this.container.id;
  this.stage = new Kinetic.Stage(options);

}
