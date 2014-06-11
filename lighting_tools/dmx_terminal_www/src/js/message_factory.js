var ROSUtils = {};


ROSUtils.MessageFactory = function(r){
  this.knownTypes = {};
  this.ros = r;
};


ROSUtils.MessageFactory.prototype.getMessageConstructor = function(name){
  return this.knownTypes[name];
};

ROSUtils.MessageFactory.prototype.createMessage = function(name){
  if(!this.knownTypes[name]){
    return null;
  }
  return new this.knownTypes[name]();
};

ROSUtils.MessageFactory.prototype.addMessageConstructor = function(name, f){
  this.knownTypes[name] = f;
};

ROSUtils.MessageFactory.prototype.getMessageTypes = function() {
  var list = [];
  for(var type in this.knownTypes){
    list.push(type);
  }
  return list;
};

ROSUtils.MessageFactory.prototype.getMessageDetailsCallback = function(that){
  return function(details){
    //Create constructor functions
    console.log(details);
    for(var idx in details){
      (function(detail){
      var fieldList=details[idx].fieldnames;
      var typeList=details[idx].fieldtypes;
      var exampleList=details[idx].examples;

      console.log("Creating constructor[" + idx + "]: " + details[idx].type);
      var currentType = details[idx].type;

        that.addMessageConstructor(currentType,
          function(){
            for(var fIdx in fieldList){
              var fieldType = typeList[fIdx];
              var fieldName = fieldList[fIdx];
              var fieldExample = exampleList[fIdx];

              if(fieldExample == '[]'){
                this[fieldName] = [];
                continue;
              }

              if(fieldType == 'string'){
                this[fieldName] = String();
              }
              else if(fieldType.indexOf('int') > -1){
                this[fieldName] = Number();
              }
              else if(fieldType== 'bool'){
                this[fieldName] = Boolean();
              }
              else if(fieldType.length > 0){
                if(fieldExample == '{}'){
                  this[fieldName] = that.createMessage(fieldType);
                }
              }
              else{
                this[fieldName] = undefined;
              }
            }
          });
        })(details[idx]);
    };
  }.bind(this);
};

ROSUtils.MessageFactory.prototype.getMessageDetails = function(){
  this.ros.getMessageDetails("lighting_msgs/DmxCommand",
    this.getMessageDetailsCallback(this)
  );
};
