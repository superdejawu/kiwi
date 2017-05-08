'use strict';

KIWI = KIWI || {};
CanvasRenderingContext2D.prototype.wrapText = function (text, x, y, maxWidth, lineHeight) {

    var lines = text.split("\n");

    for (var i = 0; i < lines.length; i++) {

        var words = lines[i].split(' ');
        var line = '';

        for (var n = 0; n < words.length; n++) {
            var testLine = line + words[n] + ' ';
            var metrics = this.measureText(testLine);
            var testWidth = metrics.width;
            if (testWidth > maxWidth && n > 0) {
                this.fillText(line, x, y);
                line = words[n] + ' ';
                y += lineHeight;
            } else {
                line = testLine;
            }
        }

        this.fillText(line, x, y);
        y += lineHeight;
    }
};
HTMLCanvasElement.prototype.relMouseCoords = function(event){
    var totalOffsetX = 0;
    var totalOffsetY = 0;
    var canvasX = 0;
    var canvasY = 0;
    var currentElement = this;

    do{
        totalOffsetX += currentElement.offsetLeft - currentElement.scrollLeft;
        totalOffsetY += currentElement.offsetTop - currentElement.scrollTop;
    }
    while(currentElement = currentElement.offsetParent)

    canvasX = event.pageX - totalOffsetX;
    canvasY = event.pageY - totalOffsetY;

    return {x:canvasX, y:canvasY}
};

KIWI.Demo = function(){
  var el, canvas, ctx;
  var animeId;
  var fixedRenderQueue = [];
  var dynamicRenderQueue = [];
  var that = this;
  var options = {
      fq: 85,              //render frequence is 1000/60 ~= 16.67
      renderUpdates: true, //if nothing changes, dont even render
      circleBase: '#1F1F24',
      circleFill: '#AF00FF',
      target: 'DemoCanvas',
      percent: [5,5,5,5],
      animate: [0,0,0,0],
      height: 0,
      width: 0,
      rwdSize: 0,
      size: 160,
      lineWidth: 6,
      rotate: 0,
  };
  
  this.startRender = render;
  this.stopRender = stopRender;

  var drawCircle = function(color, xDis,yDis, lineWidth, percent) {
    var radius = (options.size + options.lineWidth) / 2;
    ctx.beginPath();
    ctx.arc(xDis, yDis, radius, 0, Math.PI * 2 * percent, false);
    ctx.strokeStyle = color;
    ctx.lineCap = 'square'; // butt, round or square
    ctx.lineWidth = lineWidth;
    ctx.stroke();
  };

  var drawCircularProgress = function(index){
    var xDis=0;
    var yDis=0;
    if(index === 0){
      xDis = -options.rwdSize * 0.5;
      yDis = -options.rwdSize * 0.5 ;
    }else if(index===1) {
      xDis = options.rwdSize * 0.5;
      yDis = -options.rwdSize * 0.5;
    }else if(index===2) {
      xDis = -options.rwdSize * 0.5;
      yDis = options.rwdSize * 0.5;
    }else if(index===3) {
      xDis = options.rwdSize * 0.5;
      yDis = options.rwdSize * 0.5;
    }
    drawCircle(options.circleBase, xDis,yDis, options.lineWidth, 100 / 100);
    drawCircle(options.circleFill, xDis,yDis,options.lineWidth+1, options.percent[index] / 100);
  };
  
  var animateGesture = function(i,fd,timer){
    window.clearTimeout(timer);
    if ((options.animate[i] === 5 || fd === false) && options.animate[i]>1){
      options.animate[i] --;
      options.renderUpdates = true;
      timer = window.setTimeout(function(){
        animateGesture(i, false, timer);
      }, options.fq);
    }else if(fd===true){

      options.animate[i]++;
      options.renderUpdates = true;
      timer = window.setTimeout(function(){
        animateGesture(i, true, timer);
      }, options.fq);
    }else{
      options.animate[i] = 0;
      options.renderUpdates = true;
    }

  };
  var animationController = function(index){
    var timer;
    if (options.animate[index] === 0) {
      animateGesture(index, true, timer);
    }
  }
  var clickController = function(e){
    var coords = canvas.relMouseCoords(e);
    coords.x = coords.x - options.width/2;
    coords.y = coords.y - options.height/2;
    var timer;
    if(coords.x > 0){
      if(coords.y > 0){
        animationController(3);
      }else{
        animationController(1);
      }
    }else{
      if(coords.y > 0){
        animationController(2);
      }else{
        animationController(0);
      }
    }
  };

  var stopRender = function(){
    window.cancelAnimationFrame(animeId);
  };

  var render = function(){
    animeId = window.requestAnimationFrame(render);
    if(options.renderUpdates){
      ctx.clearRect(-canvas.width/2, -canvas.height/1.7, canvas.width, canvas.height);
      for(var i=0; i<dynamicRenderQueue.length; i++){
        dynamicRenderQueue[i]();
      }
      for(var i=0; i<fixedRenderQueue.length; i++){
        fixedRenderQueue[i]();
      }
      options.renderUpdates = false;
    }

  };

  this.onData = null;
  this.onScore = function(scores){
    var i=0;
    for (i; i<scores.length; i++){
      if( scores[i].score !== null){
        var confidence = scores[i].threshold * 2 - parseInt(scores[i].score);
        if (confidence <= 0 || confidence === null){
          confidence = 0.01;
        }else{ 
          confidence = confidence / scores[i].threshold ;
          confidence = confidence > 1 ? 1 : confidence;
        }
        if (confidence < 0.7){
          options.percent[i] = confidence*100/2;
        }else{
          options.percent[i] = confidence*100;
        }
      }
    }
    options.renderUpdates = true;
  };

  this.onMotion = function(motion){
    if(motion.motion_id === "a332ea13c27c4a66b1148e219f141a71"){
        animationController(0);
    }else if(motion.motion_id === "acacd67639d64cd3a23215f08a379bfe"){
        animationController(1);
    }else if(motion.motion_id === "fae9a0235bd44da68f4d3ed4315dc335"){
        animationController(2);
    }else if(motion.motion_id === "c37d7f69f6ae45fa8b72ad4b4fbd2a54"){
        animationController(3);
    }
  };

  this.rwd = function(){
    options.width = el.clientWidth;
    options.height = el.clientHeight;
    if(options.width/3 < 160){
      options.rwdSize = 160;
    }else if(options.width > options.height){
      options.rwdSize = options.height / 3;
    }else{
      options.rwdSize = options.width/3;
    }
    canvas.height = options.height;
    canvas.width = options.width;
    ctx.translate(options.width / 2, options.height / 1.7); // change center
    options.renderUpdates = true;
  };
  
  function init(target){
    options.target = target || options.target;
    el = document.getElementById(options.target); // get canvas
    el.removeAttribute("style");
    canvas = document.createElement('canvas');
    canvas.id = 'SimpleMotionRecognizer';
    if (typeof(G_vmlCanvasManager) !== 'undefined') {
        G_vmlCanvasManager.initElement(canvas);
    }
    
    ctx = canvas.getContext('2d');
    that.rwd();
    canvas.addEventListener('click', clickController);
    el.appendChild(canvas);
    
    // load image from data url
    var imageObj = {
      obj: new Image(),
      texts: ["Double Flick", "Chop", "Up Down", "Down Up"],
      src: '/images/fullsprite.png'
    }


    imageObj.obj.onload = function() {
      
      dynamicRenderQueue.push(function(){
        for (var i=0;i<4;i++){
          drawCircularProgress(i);
        }
      });
      fixedRenderQueue.push(function(){
        ctx.font = "100 14px  jaf-facitweb";
        ctx.fillStyle = "#ffffff";
        ctx.textAlign = "left";
      ctx.wrapText("Welcome to your custom demo page! Perform the gestures below with your phone, and watch as our technology recognizes the gestures. The technology works the same way in our software and is available for use in any sensor on any device.\n\nDon't know how to perform a motion? Click on any picture below for a quick how-to", -options.rwdSize * 1.2, -options.rwdSize * 1.5, options.rwdSize * 2.7, 20);
        for (var i=0;i<4;i++){
          var xDis=0;
          var yDis=0;
          if(i===0){
            xDis = -options.rwdSize * 0.5 - options.size/2;
            yDis = -options.rwdSize * 0.5 - options.size/2;
          }else if(i===1) {
            xDis = options.rwdSize * 0.5 - options.size/2;
            yDis = -options.rwdSize * 0.5 - options.size/2;
          }else if(i===2) {
            xDis = -options.rwdSize * 0.5 - options.size/2;
            yDis = options.rwdSize * 0.5 - options.size/2;
          }else if(i===3) {
            xDis = options.rwdSize * 0.5 - options.size/2;
            yDis = options.rwdSize * 0.5 - options.size/2;
          }
          ctx.drawImage(imageObj.obj, options.size*options.animate[i], options.size*i, options. size, options.size, xDis, yDis, options.size, options.size);
          ctx.font = "16px din-light";
          ctx.fillStyle = "#ffffff";
          ctx.textAlign = "center";
          ctx.fillText(imageObj.texts[i], xDis+options.size/2, yDis + options.size * 1.25);
        }
      });
        var timer;
        timer = window.setTimeout(function(){
          window.clearTimeout(timer);
          animationController(0);
          timer = window.setTimeout(function(){
            window.clearTimeout(timer);
            animationController(1);
            timer = window.setTimeout(function(){
              window.clearTimeout(timer);
              animationController(2);
              timer = window.setTimeout(function(){
                window.clearTimeout(timer);
                animationController(3);
              }, 1000);
            },1000);
          },1000);
        },1000);
      options.renderUpdates = true;
    };

    imageObj.obj.src = imageObj.src;
    render();

  }

  init();
};

