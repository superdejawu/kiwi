//TODO: better Init funciton

function Pikanvas(tarID, optsIn){
  // --ALL-- variables are made private
  var tarID, tarDOM, opts;    
  var _this = this;
  // imu and filters vars
  var imuf;
  // --3JS powered-- scene set up
  var scene, controls, renderer;
  // --3JS powered-- objects
  var sphere, sphereTail;
  //GOOOOOO~~~
  init();
  
  function init(){
    //HTML setup
    tarID = tarID;
    tarDOM = document.getElementById(tarID);
    
    //OPTIONS:
    opts = {
      "width" : optsIn.width || tarDOM.getAttribute("data-pk-width") || 500,
      "height" : optsIn.height || tarDOM.getAttribute("data-pk-height") || 500,
      "debug": optsIn.debug || false,
      "debug_show_stats": optsIn.debug_show_stats || false,
      "debug_show_raw": optsIn.debug_show_raw || false,
      "debug_show_filtered": optsIn.debug_show_filtered || false,
      "debug_show_axes": optsIn.debug_show_axes || false,
      "show_axes": optsIn.show_axes || false,
      //IMU filter globals
      "degToRad": (Math.PI / 180.0),
      "gToMperS_2": 9.81,
      "GyrolsbPerDeg": optsIn.GyrolsbPerDeg || (32767 / 500), //assuming the gyro is set to 500 Deg/sec and it's output is the size of a short
      "AcclsbPerG": optsIn.AcclsbPerg || (32767 / 4), //assuming the gyro is set to 4G and it's output is the size of short
      "MaglsbPer_uT": optsIn.MaglsbPer_uT || (30000 / 1000), //mag output is +-1200 uT
    }

    // --3JS powered-- scene set up
    scene = new THREE.Scene();
    view = {};
    view.camera = new THREE.PerspectiveCamera( 75, opts.width/opts.height, 0.1, 1000 );
    view.camera.position.set(50, 0, 0);
    //view.camera.lookAt(new THREE.Vector3(0, 0, 0));
    view.opts = {
      left  :0, bottom:0,
      width :opts.width,
      height:opts.height,
      color :"#2D2D2D"
    }

    // --3JS powered-- le renderer
    renderer = new THREE.WebGLRenderer();
    renderer.setSize( opts.width, opts.height );
    tarDOM.innerHTML="";
    tarDOM.appendChild( renderer.domElement );

    // --3JS plugin-- trackball controls
    controls = new THREE.TrackballControls(view.camera, renderer.domElement );
    controls.target.set( 0, 0, 0 );
    controls.rotateSpeed = 1.0;
    controls.zoomSpeed = 1.2;
    controls.panSpeed = 0.8;

    controls.noZoom = false;
    controls.noPan = false;

    controls.staticMoving = true;
    controls.dynamicDampingFactor = 0.3;

    controls.keys = [ 65, 83, 68 ];

    controls.addEventListener( 'change', render );
    

    // Other DOM setup
    // View Angle Button
    // var button = document.createElement("button");
    // button.appendChild(document.createTextNode("front"));
    // button.addEventListener("click", function(){
    //   view.camera.position.set(50, 0, 0);
    //   view.camera.quaternion.set(0,0,0,0);
    //   view.camera.lookAt(new THREE.Vector3(0, 0, 0));
    // });
    // tarDOM.appendChild(button);

    // debug stuff
    if (opts.debug){
    }

    //show webgl stats
    if (opts.debug_show_stats){
      stats = new Stats();
      stats.domElement.style.position = 'absolute';
      stats.domElement.style.top = '0px';
      stats.domElement.style.zIndex = 100;
      tarDOM.appendChild( stats.domElement );
    }
    // the references axes
    if(opts.show_axes){
      scene.add( 
        basicLine ([1000,0,0],  [0,0,0], rgbToHex(0,0,255)), 
        basicLine ([-1000,0,0], [0,0,0], rgbToHex(0,0,200)),
        basicLine ([0,1000,0],  [0,0,0], rgbToHex(0,255,0)), 
        basicLine ([0,-1000,0], [0,0,0], rgbToHex(0,200,0)),
        basicLine ([0,0,1000],  [0,0,0], rgbToHex(255,0,0)), 
        basicLine ([0,0,-1000], [0,0,0], rgbToHex(200,0,0)) 
      );
    }

    animate();

    // setup the imu filters and AHRS
    imuf = new IMUfilter((1/50),25);
    _this.getImuf = function(){return imuf;}
    madgwick = new MadgwickAHRS();
    mahony = new MahonyAHRS();

    // private var accessors
    _this.getCanvasID = function(){ return tarID;};
    _this.getCanvasDOM = function(){ return tarID;};
    _this.getOpts = function(){ return tarID;};
  }




  this.pikaData = function(data){
    //TODO: renit logic here
    imuf.reset();
    var x_offset=0, y_offset=0, z_offset=0;
    var drawingSet = [{"x":0,"y":0,"z":0}];
    var n = new Date().getTime();
    var c = 0;
    for(var j=0; j< data.ax.length; j++){
      //imuFilter takes the GyroData in rad/sec and Acc data in m/s^2; So first let's do the conversions
      var w_x = ((data.gx[j] / opts.GyrolsbPerDeg) * opts.degToRad);
      var w_y = ((data.gy[j] / opts.GyrolsbPerDeg) * opts.degToRad);
      var w_z = ((data.gz[j] / opts.GyrolsbPerDeg) * opts.degToRad);
      var a_x = ((data.ax[j] / opts.AcclsbPerG) * opts.gToMperS_2);
      var a_y = ((data.ay[j] / opts.AcclsbPerG) * opts.gToMperS_2);
      var a_z = ((data.az[j] / opts.AcclsbPerG) * opts.gToMperS_2);

      
      //AHRS filter takes GyroData in rad/sec Acc data in m/s^2 and Mag Data in I don't know
      // var m_x = (device.MagnetometerData.RawX / MaglsbPer_uT);
      // var m_y = (device.MagnetometerData.RawY / MaglsbPer_uT);
      // var m_z = (device.MagnetometerData.RawZ / MaglsbPer_uT);
      //CURRENT DATA SET HAS NO Magnetometer input
      //var m_x = 0, m_y = 0, m_z = 0;
      
      imuf.routine(w_x, w_y, w_z, a_x, a_y, a_z)
      if(c<=2 && j+1===data.ax.length){
        j=0;
        c++;
        x_offset = imuf.getLinDisp_x();
        y_offset = imuf.getLinDisp_y();
        z_offset = imuf.getLinDisp_z();
      }else if(c==2){
        drawingSet.push({"x":(imuf.getLinDisp_x()-x_offset)*10000, "y":(imuf.getLinDisp_y()-y_offset)*10000, "z":(imuf.getLinDisp_z()-z_offset)*10000});
      }
    }
    n = new Date().getTime() - n;
    console.log("Run Time: " + n + "ms");
    this.drawData(drawingSet);
  }

  this.drawData = function(data){
    if (sphere === undefined){
      sphere = new THREE.Mesh(
        new THREE.SphereGeometry(0.3, 32, 32),
        new THREE.MeshBasicMaterial({
        color: rgbToHex(255,255,0)
      }));
      sphereTail = new tailTrail(sphere);
      sphere.position.set(data[0].x,data[0].z,data[0].y);
      scene.add(sphere);
    }else{
      sphere.position.set(data[0].x,data[0].z,data[0].y);
    }

    var i =1;
    var timeout;
    var slowdown = 10;
    function waitedLoop(){
      clearTimeout(timeout);
      if (i< data.length){
        //scene.add(basicLine ([data[i-1].x,data[i-1].z,data[i-1].y], [data[i].x,data[i].z,data[i].y], rgbToHex(Math.round(255*i/data.length),Math.round(255*i/data.length),0)));
        sphere.position.set(data[i].x, data[i].z, data[i].y);
        
        //console.log(i + ": " + data[i].x + "," + data[i].y + "," + data[i].z);
        i++;
        timeout = setTimeout(waitedLoop, slowdown);
      }
    }
    console.log("Set Size: " + data.length);
    
    waitedLoop();
  };
  


  // Canvas Helpers
  function render(){
      renderer.render(scene, view.camera);  
  }
  function animate() {
    requestAnimationFrame( animate );
    //renderer.setViewport(view.opts.left, view.opts.bottom, view.opts.width, view.opts.height );
    //renderer.setScissor( view.opts.left, view.opts.bottom, view.opts.width, view.opts.height );
    //renderer.enableScissorTest ( true );
    renderer.setClearColor( view.opts.color );
    controls.update();
    render();
  };
  
  // -- 3JS --  
  function tailTrail(tarObj){
    var pts = [];
    seg = [];
    var _tarObj;

    function update(data){
      pts.push(data);
      
      if(pts.length < 20 ){
        var i = seg.length;
        seg.push(basicLine ([pts[i].x,pts[i].z,pts[i].y], [data.x,data.z,data.y], rgbToHex(255,255,0)));
        // if(seg.length>1){
        //   for(var j=0; j<seg.length; j++){
        //     seg[j].material.color = rgbToHex(Math.round(255*(j+5)/seg.length),Math.round(255*(j+5)/seg.length),0);
        //   }
        // }
        scene.add(seg[i]);
      }else{
        
        for(var i=0; i<seg.length; i++){
          seg[i].geometry.vertices.push(new THREE.Vector3(pts[i+2].x,pts[i+2].z,pts[i+2].y));
          seg[i].geometry.vertices.shift();
          seg[i].geometry.verticesNeedUpdate = true;
          //seg[i].material.color = rgbToHex(255,0,0);
        }
        pts.shift();
      }

    }

    this.init = function(tarObj){
      _tarObj = tarObj;
      _tarObj.position.overridedSet = tarObj.position.set;
      tarObj.position.set = function (x,z,y) {
        _tarObj.position.overridedSet(x,z,y);
        update({"x":x,"y":y,"z":z});
      };
      pts[0] = {"x":0,"y":0,"z":0};
    }
    this.init(tarObj);
  }

  function basicLine(a, b, color){
    var material = new THREE.LineBasicMaterial({
      color: color
    });
    var geometry = new THREE.Geometry();
    geometry.vertices.push(
      new THREE.Vector3( a[0], a[1], a[2] ),
      new THREE.Vector3( b[0], b[1], b[2] )
    );
    return new THREE.Line( geometry, material );
  }

  function rgbToHex(r, g, b) {
    var hexr = r.toString(16);
    var hexg = g.toString(16);
    var hexb = b.toString(16);
    hexr = hexr.length == 1 ? "0" + hexr : hexr;
    hexg = hexg.length == 1 ? "0" + hexg : hexg;
    hexb = hexb.length == 1 ? "0" + hexb : hexb;
    return "#" + hexr + hexg + hexb;
  }
};



