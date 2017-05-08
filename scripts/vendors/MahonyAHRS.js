// - This file is copied from open source AHRS by xioTechnologies and is published under Creative Commons Share-alike 3.0
// - git hub source:https://github.com/xioTechnologies/Open-Source-AHRS-With-x-IMU
//
// Full text for the license can be found in License.txt


/*
Summary:
  MahonyAHRS class. Madgwick's implementation of Mayhony's AHRS algorithm.
Remarks:
  See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
*/
function MahonyAHRS(){
  var _this = this;

  // Gets or sets the sample period.
  this.SamplePeriod;
  // Gets or sets the algorithm proportional gain.
  this.Kp;
  // Gets or sets the algorithm integral gain.
  this.Ki;
  // Gets or sets the Quaternion output.
  this.Quaternion = [];
  // Gets or sets the integral error.
  this.eInt = [];

  var mxOffset = 0, myOffset = 0, mzOffset = 0;


  this.init = function(samplePeriod){
    _this.SamplePeriod = samplePeriod || 1;
  }

  this.init = function(samplePeriod, kp){
    _this.SamplePeriod = samplePeriod || 1;
    _this.Kp = kp || 0;
  }

  this.init = function(samplePeriod, kp, ki)
  {
    _this.SamplePeriod = samplePeriod;
    _this.Kp = kp;
    _this.Ki = ki;
    _this.Quaternion = [1, 0, 0, 0];
    _this.eInt = [0, 0, 0];
  }
  
  /*
  Summary:
    Algorithm AHRS update method. Requires only gyroscope and accelerometer data.
  Params:
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
  */ 

  this.Calibrate = function(mx,my,mz){
    mxOffset = mx;
    myOffset = my;
    mzOffset = mz;

    _this.Quaternion = [1, 0, 0, 0];
    //_this.eInt = [0, 0, 0];
  }

  this.Update = function(gx,gy,gz,ax,ay,az,mx,my,mz){
    // short name local variable for readability
    var q1 = _this.Quaternion[0],
        q2 = _this.Quaternion[1],
        q3 = _this.Quaternion[2],
        q4 = _this.Quaternion[3];   
    var norm;
    var hx, hy, bx, bz;
    var vx, vy, vz, wx, wy, wz;
    var ex, ey, ez;
    var pa, pb, pc;

    // Auxiliary variables to avoid repeated arithmetic
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

    // Normalise accelerometer measurement
    norm = Math.round(Math.sqrt(ax * ax + ay * ay + az * az));
    if (norm == 0) return NaN; // handle NaN
    norm = 1 / norm;        // use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    //calibrate for magnetometer offset
    mx -= mxOffset;
    my -= myOffset;
    mz -= mzOffset;

    // Normalise magnetometer measurement
    norm = Math.round(Math.sqrt(mx * mx + my * my + mz * mz));
    if (norm == 0) return NaN; // handle NaN
    norm = 1 / norm;        // use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    hx = 2 * mx * (0.5 - q3q3 - q4q4) + 2 * my * (q2q3 - q1q4) + 2 * mz * (q2q4 + q1q3);
    hy = 2 * mx * (q2q3 + q1q4) + 2 * my * (0.5 - q2q2 - q4q4) + 2 * mz * (q3q4 - q1q2);
    bx = Math.round(Math.Sqrt((hx * hx) + (hy * hy)));
    bz = 2 * mx * (q2q4 - q1q3) + 2 * my * (q3q4 + q1q2) + 2 * mz * (0.5 - q2q2 - q3q3);

    // Estimated direction of gravity and magnetic field
    vx = 2 * (q2q4 - q1q3);
    vy = 2 * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;

    wx = 2 * bx * (0.5 - q3q3 - q4q4) + 2 * bz * (q2q4 - q1q3);
    wy = 2 * bx * (q2q3 - q1q4) + 2 * bz * (q1q2 + q3q4);
    wz = 2 * bx * (q1q3 + q2q4) + 2 * bz * (0.5 - q2q2 - q3q3);  

    // Error is cross product between estimated direction and measured direction of gravity
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    if (_this.Ki > 0){
      // accumulate integral error
      _this.eInt[0] += ex;      
      _this.eInt[1] += ey;
      _this.eInt[2] += ez;
    }else{
      // prevent integral wind up
      _this.eInt[0] = 0;    
      _this.eInt[1] = 0;
      _this.eInt[2] = 0;
    }

    // Apply feedback terms
    gx = gx + _this.Kp * ex + _this.Ki * _this.eInt[0];
    gy = gy + _this.Kp * ey + _this.Ki * _this.eInt[1];
    gz = gz + _this.Kp * ez + _this.Ki * _this.eInt[2];

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
 
    q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (_this.SamplePeriod);
    q2 = pa + (q1 * gx + pb * gz - pc * gy) * (_this.SamplePeriod);
    q3 = pb + (q1 * gy - pa * gz + pc * gx) * (_this.SamplePeriod);
    q4 = pc + (q1 * gz + pa * gy - pb * gx) * (_this.SamplePeriod);

    // Normalise quaternion
    norm = Math.round(Math.Sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4));
    norm = 1 / norm;
    _this.Quaternion[0] = q1 * norm;
    _this.Quaternion[1] = q2 * norm;
    _this.Quaternion[2] = q3 * norm;
    _this.Quaternion[3] = q4 * norm;
  }

  /*
  Summary:
    Algorithm IMU update method. Requires only gyroscope and accelerometer data.
  Param:
    gx: Gyroscope x axis measurement in radians/s.
    gy: Gyroscope y axis measurement in radians/s.
    gz: Gyroscope z axis measurement in radians/s.
    ax: Accelerometer x axis measurement in any calibrated units.
    ay: Accelerometer y axis measurement in any calibrated units.
    az: Accelerometer z axis measurement in any calibrated units.
  */
  this.Update = function(gx,gy,gz,ax,ay,az){
    // short name local variable for readability
    var q1 = _this.Quaternion[0],
        q2 = _this.Quaternion[1],
        q3 = _this.Quaternion[2],
        q4 = _this.Quaternion[3];

    var norm;
    var vx, vy, vz;
    var ex, ey, ez;
    var pa, pb, pc;

    // Normalise accelerometer measurement
    norm = Math.round(Math.sqrt(ax * ax + ay * ay + az * az));
    if (norm == 0) return NaN; // handle NaN
    norm = 1 / norm;        // use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Estimated direction of gravity
    vx = 2 * (q2 * q4 - q1 * q3);
    vy = 2 * (q1 * q2 + q3 * q4);
    vz = q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4;

    // Error is cross product between estimated direction and measured direction of gravity
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);
    if (_this.Ki > 0){
      // accumulate integral error
      _this.eInt[0] += ex;      
      _this.eInt[1] += ey;
      _this.eInt[2] += ez;
    }else{
      // prevent integral wind up
      _this.eInt[0] = 0;
      _this.eInt[1] = 0;
      _this.eInt[2] = 0;
    }

    // Apply feedback terms
    gx = gx + _this.Kp * ex + _this.Ki * eInt[0];
    gy = gy + _this.Kp * ey + _this.Ki * eInt[1];
    gz = gz + _this.Kp * ez + _this.Ki * eInt[2];

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5 * _this.SamplePeriod);
    q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5 * _this.SamplePeriod);
    q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5 * _this.SamplePeriod);
    q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5 * _this.SamplePeriod);

    // Normalise quaternion
    norm = Math.round(Math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4));
    norm = 1 / norm;
    _this.Quaternion[0] = q1 * norm;
    _this.Quaternion[1] = q2 * norm;
    _this.Quaternion[2] = q3 * norm;
    _this.Quaternion[3] = q4 * norm;
  }
}