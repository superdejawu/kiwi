/*
Summary:
  MadgwickAHRS class. Implementation of Madgwick's IMU and AHRS algorithms.
Remarks:
  See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
*/
function MadgwickAHRS(){
  var _this = this;
  /*   PUBLICS   */
  // Gets or sets the sample period.
  this.SamplePeriod;  
  // Gets or sets the algorithm gain beta.
  this.Beta;
  // Gets or sets the Quaternion output.
  this.Quaternion = [];
  this.rotMatrix = [[],[]];
  // Gets or set the Euler output.
  this.Euler = [];
  
  /*    EXPOSERS    */
  this.Euler_yaw = function(){
    return this.Euler[YAW];
  }
  this.Euler_pitch = function(){
    return this.Euler[PITCH];
  }
  this.Euler_roll = function(){
    return this.Euler[ROLL];
  }
  this.Quat_x = function(){
    return this.Quaternion[1];
  }
  this.Quat_y = function(){
    return this.Quaternion[3];
  }
  this.Quat_z = function(){
    return this.Quaternion[2];
  }
  this.Quat_2 = function(){
    return this.Quaternion[0];
  }

  //CONSTANTS
  var YAW = 0, PITCH = 1, ROLL = 2;

  this.init = function(samplePeriod){
    _this.SamplePeriod = samplePeriod || 1;
  }

  this.init = function(samplePeriod, beta){
    _this.SamplePeriod = samplePeriod || 1;
    _this.Beta = beta;
    _this.Quaternion = [1,0,0,0];
    _this.Euler = [0,0,0];
    _this.rotMatrix = [[0,0,0],[0,0,0],[0,0,0]];
  }


  /*
  Summary:
    Algorithm AHRS update method. Requires only gyroscope and accelerometer data.
  Param:
    gx: Gyroscope x axis measurement in radians/s.
    gy: Gyroscope y axis measurement in radians/s.
    gz: Gyroscope z axis measurement in radians/s.
    ax: Accelerometer x axis measurement in any calibrated units.
    ay: Accelerometer y axis measurement in any calibrated units.
    az: Accelerometer z axis measurement in any calibrated units.
    mx: Magnetometer x axis measurement in any calibrated units.
    my: Magnetometer y axis measurement in any calibrated units.
    mz: Magnetometer z axis measurement in any calibrated units.
  Remarks:
    Optimised for minimal arithmetic.
    Total ±: 160
    Total *: 172
    Total /: 5
    Total sqrt: 5
  */
  var ax_Calib = 0, ay_Calib = 0, az_Calib = 0, mx_Calib = -60, my_Calib = 48, mz_Calib = 0;

  this.Calibrate = function(){
    /*
    mx_Calib = mx;
    my_Calib = my;
    mz_Calib = mz;

    ax_Calib = ax;
    ay_Calib = ay;
    az_Calib = az;
    */
    _this.Quaternion = [1, 0, 0, 0];
  }
  this.Update = function(gx,gy,gz,ax,ay,az,mx,my,mz){
    // short name local variable for readability
    var q1 = _this.Quaternion[0], q2 = _this.Quaternion[1], q3 = _this.Quaternion[2], q4 = _this.Quaternion[3];

    var norm;
    var hx, hy, _2bx, _2bz;
    var s1, s2, s3, s4;
    var qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    var _2q1mx;
    var _2q1my;
    var _2q1mz;
    var _2q2mx;
    var _4bx;
    var _4bz;
    var _2q1 = 2 * q1;
    var _2q2 = 2 * q2;
    var _2q3 = 2 * q3;
    var _2q4 = 2 * q4;
    var _2q1q3 = 2 * q1 * q3;
    var _2q3q4 = 2 * q3 * q4;
    var q1q1 = q1 * q1;
    var q1q2 = q1 * q2;
    var q1q3 = q1 * q3;
    var q1q4 = q1 * q4;
    var q2q2 = q2 * q2;
    var q2q3 = q2 * q3;
    var q2q4 = q2 * q4;
    var q3q3 = q3 * q3;
    var q3q4 = q3 * q4;
    var q4q4 = q4 * q4;

        
    //calibrate mag measurement    
    mx -= mx_Calib;
    my -= my_Calib;
    mz -= mz_Calib;
    /*
    //calibrate accel measurement
    ax -= ax_Calib;
    ay -= ay_Calib;
    az -= az_Calib;
    */
    // Normalise accelerometer measurement
    norm = Math.round(Math.sqrt(ax * ax + ay * ay + az * az));
    if (norm == 0) return NaN; // handle NaN
    norm = 1 / norm;    // use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = Math.round(Math.sqrt(mx * mx + my * my + mz * mz));
    if (norm == 0) return NaN; // handle NaN
    norm = 1 / norm;   // use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;

    //Console.WriteLine(mx + "  -   " + my + "  -   " + mz );//+ "  ----  " + ax + "  -   " + ay + "  -   " + az); 

    // Reference direction of Earth's magnetic field
    _2q1mx = 2 * q1 * mx;
    _2q1my = 2 * q1 * my;
    _2q1mz = 2 * q1 * mz;
    _2q2mx = 2 * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = Math.round(Math.sqrt(hx * hx + hy * hy));
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2 * _2bx;
    _4bz = 2 * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2 * q2q4 - _2q1q3 - ax) + _2q2 * (2 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2 * q2q4 - _2q1q3 - ax) + _2q1 * (2 * q1q2 + _2q3q4 - ay) - 4 * q2 * (1 - 2 * q2q2 - 2 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2 * q2q4 - _2q1q3 - ax) + _2q4 * (2 * q1q2 + _2q3q4 - ay) - 4 * q3 * (1 - 2 * q2q2 - 2 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2 * q2q4 - _2q1q3 - ax) + _2q3 * (2 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
        
    norm = 1 / Math.round(Math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4));    // normalise step magnitude
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - _this.Beta * s1;
    qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - _this.Beta * s2;
    qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - _this.Beta * s3;
    qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - _this.Beta * s4;

    //Console.WriteLine(s4);

    // Integrate to yield quaternion
    q1 += qDot1 * _this.SamplePeriod;
    q2 += qDot2 * _this.SamplePeriod;
    q3 += qDot3 * _this.SamplePeriod;
    q4 += qDot4 * _this.SamplePeriod;
    norm = 1 / Math.round(Math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4));    // normalise quaternion
    _this.Quaternion[0] = q1 * norm;
    _this.Quaternion[1] = q2 * norm;
    _this.Quaternion[2] = q3 * norm;
    _this.Quaternion[3] = q4 * norm;
  }

  this.computeRotMatrix = function(){
    //compute the Rotation matrix out or the quaternion
    _this.rotMatrix[0, 0] = ((2 * Math.pow(_this.Quat_w, 2)) - 1) + (2 * Math.pow(-_this.Quat_x, 2));
    _this.rotMatrix[0, 1] = 2 * ((-_this.Quat_x * -_this.Quat_z) + (_this.Quat_w * _this.Quat_y));
    _this.rotMatrix[0, 2] = 2 * (-_this.Quat_x * _this.Quat_y - _this.Quat_w * -_this.Quat_z);
    _this.rotMatrix[1, 0] = 2 * (-_this.Quat_x * -_this.Quat_z - _this.Quat_w * _this.Quat_y);
    _this.rotMatrix[1, 1] = ((2 * Math.pow(_this.Quat_w, 2)) - 1) + (2 * Math.pow(-_this.Quat_z, 2));
    _this.rotMatrix[1, 2] = 2 * ((-_this.Quat_z * _this.Quat_y) + (_this.Quat_w * -_this.Quat_x));
    _this.rotMatrix[2, 0] = 2 * ((-_this.Quat_x * _this.Quat_y) + (_this.Quat_w * -_this.Quat_z));
    _this.rotMatrix[2, 1] = 2 * ((-_this.Quat_z * _this.Quat_y) - (_this.Quat_w * -_this.Quat_x));
    _this.rotMatrix[2, 2] = ((2 * Math.pow(_this.Quat_w, 2)) - 1) + (2 * Math.pow(_this.Quat_y, 2));
  }

  _this.computeEulers = function(){
    var sqw = _this.Quat_w * _this.Quat_w;
    var sqx = _this.Quat_x * _this.Quat_x;
    var sqy = _this.Quat_y * _this.Quat_y;
    var sqz = _this.Quat_z * _this.Quat_z;
    var unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
        
    var test = _this.Quat_x * _this.Quat_y + _this.Quat_z * _this.Quat_w;
    if (test > 0.499 * unit){
      // singularity at north pole
      _this.Euler[YAW] = 2 * Math.atan2(_this.Quat_x, _this.Quat_w);
      _this.Euler[ROLL] = Math.PI / 2;
      _this.Euler[PITCH] = 0;
      return;
    }
    if (test < -0.499 * unit){
    // singularity at south pole
      _this.Euler[YAW] = -2 * Math.atan2(_this.Quat_x, _this.Quat_w);
      _this.Euler[ROLL] = -Math.PI / 2;
      _this.Euler[PITCH] = 0;
      return;
    }
    _this.Euler[YAW] = Math.atan2(2 * _this.Quat_y * _this.Quat_w - 2 * _this.Quat_x * _this.Quat_z, sqx - sqy - sqz + sqw);
    _this.Euler[ROLL] = Math.asin(2 * test / unit);
    _this.Euler[PITCH] = Math.atan2(2 * _this.Quat_x * _this.Quat_w - 2 * _this.Quat_y * _this.Quat_z, -sqx + sqy - sqz + sqw);
  }

  /*
  Summary:
    Algorithm IMU update method. Requires only gyroscope and accelerometer data.
  Params:
    gx: Gyroscope x axis measurement in radians/s.
    gy: Gyroscope y axis measurement in radians/s.
    gz: Gyroscope z axis measurement in radians/s.
    ax: Accelerometer x axis measurement in any calibrated units.
    ay: Accelerometer y axis measurement in any calibrated units.
    az: Accelerometer z axis measurement in any calibrated units.
  Remarks:
    Optimised for minimal arithmetic.
    Total ±: 45
    Total *: 85
    Total /: 3
    Total sqrt: 3
  */
  _this.Update = function(gx,gy,gz,ax,ay,az){
    // short name local variable for readability
    var q1 = _this.Quaternion[0], 
        q2 = _this.Quaternion[1],
        q3 = _this.Quaternion[2],
        q4 = _this.Quaternion[3];   
    
    var norm;
    var s1, s2, s3, s4;
    var qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    var _2q1 = 2 * q1;
    var _2q2 = 2 * q2;
    var _2q3 = 2 * q3;
    var _2q4 = 2 * q4;
    var _4q1 = 4 * q1;
    var _4q2 = 4 * q2;
    var _4q3 = 4 * q3;
    var _8q2 = 8 * q2;
    var _8q3 = 8 * q3;
    var q1q1 = q1 * q1;
    var q2q2 = q2 * q2;
    var q3q3 = q3 * q3;
    var q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = Math.round(Math.sqrt(ax * ax + ay * ay + az * az));
    if (norm == 0) return NaN; // handle NaN
    norm = 1 / norm;        // use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Gradient decent algorithm corrective step
    s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
    s2 = _4q2 * q4q4 - _2q4 * ax + 4 * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
    s3 = 4 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
    s4 = 4 * q2q2 * q4 - _2q2 * ax + 4 * q3q3 * q4 - _2q3 * ay;
        
    norm = 1 / Math.round(Math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4));    // normalise step magnitude
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - _this.Beta * s1;
    qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - _this.Beta * s2;
    qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - _this.Beta * s3;
    qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - _this.Beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * _this.SamplePeriod;
    q2 += qDot2 * _this.SamplePeriod;
    q3 += qDot3 * _this.SamplePeriod;
    q4 += qDot4 * _this.SamplePeriod;
    norm = 1 / Math.round(Math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4));    // normalise quaternion
    _this.Quaternion[0] = q1 * norm;
    _this.Quaternion[1] = q2 * norm;
    _this.Quaternion[2] = q3 * norm;
    _this.Quaternion[3] = q4 * norm;
  }
}
