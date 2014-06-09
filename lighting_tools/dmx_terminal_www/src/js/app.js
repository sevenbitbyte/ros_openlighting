var MoverState = function(){
  this.pan = 0;
  this.tilt = 0;
  this.zoom = 0;
  this.intensity = 0;
  this.white = 0;
  this.color = '';
  this.pose = {};
}

var DmxMoverView = function(domPath){
  this.canvas = $(domPath)[0];;
  var canvas;
  var clickList=[];
  var multiClick = true;
  var displayRadius=0.7;
  var moverState = {
    pan: 0,
    tilt: 0,
    zoom: 0,
    intensity: 0,
    white: 0,
    color: ""
  }
};

var canvas;
var clickList=[];
var multiClick = true;
var displayRadius=0.7;
var pointerRadius=0.1;
var moverState = {
  pan: 0,
  tilt: 0,
  zoom: 0,
  intensity: 0,
  white: 0,
  color: ""
}

var renderLoop={};

var message = "";

var hoverState = {
  time: 0,
  pos: {}
}

function render() {
  drawOutlines(canvas);
  drawMessage(canvas);
  drawPoints(canvas);
  drawHoverPoint(canvas);
}

function drawMessage(canvas) {
  var context = canvas.getContext('2d');
  context.clearRect(0, 0, 10, 10);
  context.font = '8pt Calibri';
  context.fillStyle = 'white';
  context.fillText(message, 10, 25);
}

function getCenter(canvas){
  return {x: canvas.width/2.0, y: canvas.height/2.0};
}

function distanceTo(pt1, pt2){
  return Math.sqrt( Math.pow(pt1.x - pt2.x, 2) + Math.pow(pt1.y - pt2.y, 2) );
}

function distanceToCenter(canvas, pt){
  var center = getCenter(canvas);

  return distanceTo(center, pt);
}

function drawOutlines(canvas){
  var context = canvas.getContext('2d');

  context.fillStyle = 'black';
  context.fillRect(0, 0, 255, 255);

  shift = Math.PI * 0.7;

  var center = getCenter(canvas);

  context.beginPath();
  context.arc(center.x, center.y, displayRadius, shift, shift+(Math.PI*0.6));
  context.strokeStyle= '#B2E9E2';
  context.stroke();

  shift += Math.PI;

  context.beginPath();
  context.arc(center.x, center.y, displayRadius, shift, shift+(Math.PI*0.6));
  context.strokeStyle= '#B2E9E2';
  context.stroke();
}

function drawHoverPoint(canvas){
  var context = canvas.getContext('2d');
  pos=hoverState.pos;

  var dist = distanceToCenter(canvas, pos);

  context.beginPath();
  context.arc(pos.x, pos.y , (255/2) * 0.03 * ((moverState.zoom) + 0.3), 0, Math.PI*2.0);
  context.fillStyle= 'grey';
  context.fill();

  //console.log(dist);
  if(dist > (displayRadius - pointerRadius) ){
    return;
  }


  context.beginPath();
  context.arc(pos.x, pos.y , (255/2) * 0.06, 0, Math.PI*2.0);
  context.strokeStyle= 'grey';
  context.stroke();
}

function drawPoints(canvas){
  var context = canvas.getContext('2d');
  for(idx in clickList){
    pos = clickList[idx];

    context.beginPath();
    context.arc(pos.x, pos.y , (255/2) * 0.03 * ((moverState.zoom) + 0.3), 0, Math.PI*2.0);
    context.fillStyle= '#B2E9E2';
    context.fill();

    context.beginPath();
    context.arc(pos.x, pos.y , (255/2) * 0.06, 0, Math.PI*2.0);
    context.strokeStyle= '#B2E9E2';
    context.stroke();
  }
}

function addPoint(pt){
  console.log(pt);
  if(multiClick){
    while(clickList.length > 10){ clickList.shift(); }

    clickList.push(pt);
  }
  else{
    clickList[0] = pt;
  }
}

function getEventData(canvas, evt) {
  var obj={};

  var rect = canvas.getBoundingClientRect();
  obj.pos = {
    x: evt.clientX - rect.left,
    y: evt.clientY - rect.top
  };


  if(evt.type == 'wheel' || evt.type == 'mousewheel'){

    obj.wheel = {
      x: evt.wheelDeltaX,
      y: evt.wheelDeltaY,
      delta: evt.wheelDelta
    };
  }

  return obj;
}

var ros;
var factory;
var count=0;

var serviceTest = function(){
  var dmxCmd = factory.createMessage('lighting_msgs/DmxCommand');
  var frame = factory.createMessage('lighting_msgs/DmxFrame');
  var dmxValue = factory.createMessage('lighting_msgs/DmxValue');

  dmxValue.universe = 1;
  dmxValue.offset = 0;
  for(i=0; i<255; i++){
    dmxValue.data.push(i);
  }

  frame.values.push(dmxValue);
  frame.durationMs = 1000;
  //frame.duration = {sec: 4, nsec: 0};

  dmxCmd.layers.push(frame);


  var OlaCmdClient = new ROSLIB.Service({
    ros : ros,
    name : '/ola_bridge/run_cmd',
    serviceType : 'lighting_msgs/run_command'
  });

  // Then we create a Service Request. The object we pass in to ROSLIB.ServiceRequest matches the
  // fields defined in the rospy_tutorials AddTwoInts.srv file.
  var request = new ROSLIB.ServiceRequest({
    command : dmxCmd
  });

  console.log(dmxCmd);

  // Finally, we call the /add_two_ints service and get back the results in the callback. The result
  // is a ROSLIB.ServiceResponse object.
  OlaCmdClient.callService(request, function(result) {
    console.log('Result for service call on ' + OlaCmdClient.name + ': ' + result.status);
    console.log(result);
  },
  function(err){console.log(err);}
  );
}

$( document ).ready(function() {

  ros = new ROSLIB.Ros({
    url : 'ws://' + window.location.host + ':4000'
  });

  ros.on('connection',
         function(){
           console.log('connected');
           factory = new ROSUtils.MessageFactory(ros);
           factory.getMessageDetails('lighting_msgs/DmxCommand');

           //setInterval(serviceTest, 2000);
         });

  $('body').on('contextmenu', 'canvas#pan-tilt-view', function(e){ return false; });

  canvas = $('canvas#pan-tilt-view')[0];
  var context = canvas.getContext('2d');

  displayRadius = (canvas.width/2) * 0.75;
  pointerRadius = (255/2) * 0.06;


  canvas.addEventListener('mousewheel', function(evt) {
    var data = getEventData(canvas, evt);
    mousePos=data.pos;

    moverState.zoom = Math.max(0, Math.min(moverState.zoom + data.wheel.delta/1000, 1.0));

    message = 'Mouse wheel: ' + data.wheel.x + ',' + data.wheel.y;
    //writeMessage(canvas, message);
  }, false);

  canvas.addEventListener('mousemove', function(evt) {
    var data = getEventData(canvas, evt);
    mousePos=data.pos;
    hoverState.time = new Date();
    hoverState.pos = mousePos;
    message = 'Mouse position: ' + mousePos.x + ',' + mousePos.y;
  }, false);

  canvas.addEventListener('mousedown', function(evt) {
    var data = getEventData(canvas, evt);
    mousePos=data.pos;

    var dist = distanceToCenter(canvas, mousePos);
    console.log(dist);
    if(dist > (displayRadius - pointerRadius) ){
      return;
    }

    addPoint(mousePos);

    message = 'Mousedown position: ' + mousePos.x + ',' + mousePos.y;
    //writeMessage(canvas, message);
  }, false);

  canvas.addEventListener('mouseup', function(evt) {
    var data = getEventData(canvas, evt);
    mousePos=data.pos;
    message = 'Mouseup position: ' + mousePos.x + ',' + mousePos.y;
    //writeMessage(canvas, message);
  }, false);

  window.addEventListener('keydown', function(evt) {
    //console.log(evt);
    message = "Keydown: " + evt.keyCode;
  }, true);

  renderLoop = setInterval(render, 33);
  //drawOutlines(canvas);

});
