// <copyright file="IMUFilter.js" company="Kiwi Wearables">
// Copyright (c) 2014 All Rights Reserved
// </copyright>
// <author>Marc Bishara </author>
// <date>14/09/2014</date>
// <summary> This file implements a DSP filter for data received from the Kiwi bluetooth device to derive orientation and displacement </summary>
//
// - Orientation filter is based on code that was written by Aron Berk and published on ARM's mbed.org under Copyright (c) 2010 ARM Limited
// - Rotation matrix math is based on x-IMU motion tracking by xioTechnologies and is published under Creative Commons Share-alike 3.0
// - Digital filters were designed by mkfilter/mkshape/gencode   A.J. Fisher
//
// Full text for the licenses can be found in License.txt


IMUfilter = function(rate, gi){
  
  var _this = this; //global accessor of this
  
  /**
   * FILTER GLOBALS
   * note: unless epecified, everything was a DOUBLE,
   *       and was translated to numbers
   **/
  var //sampling rate
      deltat,
      
      //Gyro error
      beta,
      
      //Euler
      phi,  theta,  psi,
      
      //Quaternion               
      Quat_w,  Quat_x,  Quat_y,  Quat_z,  
      ASq_1,  ASq_2,  ASq_3,  ASq_4,
      
      //Estimated orientation quaternion elements with initial conditions.
      SEq_1,  SEq_2, SEq_3, SEq_4,
      linVel_x, linVel_y, linVel_z,
      linVelHP_x, linVelHP_y, linVelHP_z,
      linDisp_x, linDisp_y, linDisp_z,
      linDispHP_x, linDispHP_y, linDispHP_z,
      kalmanDisp_x, kalmanDisp_y, kalmanDisp_z,

      linAcc, //double array
      tcAcc,  //double array

      //Quaternion orientation of earth frame relative to auxiliary frame.
      AEq_1, AEq_2, AEq_3, AEq_4,

      firstUpdate, //integer

      rotMatrix,//Remainder: 3x3 2D double array

      //High Pass Filter globals
      NZEROS, //integer
      NPOLES, //integer

      //HPF vectors for linear Velocity (double array)
      xVect_linVel_x, yVect_linVel_x, xVect_linVel_y, yVect_linVel_y, xVect_linVel_z, yVect_linVel_z,

      //HPF vectors for Displacement (double array)
      xVect_linDisp_x, yVect_linDisp_x, xVect_linDisp_y, yVect_linDisp_y, xVect_linDisp_z, yVect_linDisp_z,

      //LPF for tcAcc
      xVect_tcAcc_x, yVect_tcAcc_x, xVect_tcAcc_y, yVect_tcAcc_y, xVect_tcAcc_z, yVect_tcAcc_z,
      
      buffSize,
      linAccBuff_x, linAccBuff_y, linAccBuff_z, // was List<double>, now array

      //Kalman filters for displacement
      dispFltr_x, dispFltr_y, dispFltr_z, //Objects of: DisplacementFilter

      velFltr, //int array
      fltrState, //string array
      accelArea, //double array
      decelArea, //double array
      zeroCount; //double array


      var mahony;
  /* FILTER GLOBALS ENDS HERE*/

  function init(sampleRate, gyroErrorIn){
    gyroErrorIn = gyroErrorIn || 25;
    //init or zero every number / array
    beta = Math.sqrt(3.0 / 4.0) * (Math.PI * (gyroErrorIn / 180.0));
    deltat = sampleRate || (1/50);
    phi   = 0,  theta = 0,  psi   = 0;               
    Quat_w= 0,  Quat_x= 0,  Quat_y= 0,  Quat_z= 0;  
    ASq_1 = 0,  ASq_2 = 0,  ASq_3 = 0,  ASq_4 = 0;
    kalmanDisp_x=0, kalmanDisp_y=0, kalmanDisp_z=0;
    rotMatrix = [[0,0,0],[0,0,0],[0,0,0]];
    dispFltr_x = new DisplacementFilter(deltat);
    dispFltr_y = new DisplacementFilter(deltat);
    dispFltr_z = new DisplacementFilter(deltat);
    velFltr = [ 0, 0, 0 ];
    fltrState = [ "Calib", "Calib", "Calib" ];
    accelArea = [ 0, 0, 0 ];
    decelArea = [ 0, 0, 0 ];
    zeroCount = [ 0, 0, 0 ];

    mahony = new MahonyAHRS();
    _this.reset();
  }

  this.reinit = function(sampleRate, gyroErrorIn){
    _this.init(sampleRate, gyroErrorIn);
  }

  this.routine = function(w_x, w_y, w_z, a_x, a_y, a_z){
    _this.updateFilter(w_x, w_y, w_z, a_x, a_y, a_z);
    _this.computeEuler();
    _this.computeRotMatrix();
    _this.computeDisplacement(a_x, a_y, a_z);
  }

  this.resetPosition = function(kalmanModelNoise, kalmanDeviceNoise){
    dispFltr_x.setupKalmanFilter(kalmanModelNoise, kalmanDeviceNoise);
    dispFltr_y.setupKalmanFilter(kalmanModelNoise, kalmanDeviceNoise);
    dispFltr_z.setupKalmanFilter(kalmanModelNoise, kalmanDeviceNoise);

    firstUpdate = 0;
    fltrState = ["Calib", "Calib", "Calib"];

    //Quaternion orientation of earth frame relative to auxiliary frame.
    AEq_1 = 1, AEq_2 = 0, AEq_3 = 0, AEq_4 = 0;

    //Estimated orientation quaternion elements with initial conditions.
    SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0;

    linVel_x   = 0, linVel_y   = 0, linVel_z   = 0;
    linVelHP_x = 0, linVelHP_y = 0, linVelHP_z = 0;

    NZEROS = 2; NPOLES = 2;
    xVect_linVel_x = [0,0,0];
    yVect_linVel_x = [0,0,0];
    xVect_linVel_y = [0,0,0];
    yVect_linVel_y = [0,0,0];
    xVect_linVel_z = [0,0,0];
    yVect_linVel_z = [0,0,0];

    linDisp_x = 0; linDisp_y = 0; linDisp_z = 0;
    linDispHP_x = 0; linDispHP_y = 0; linDispHP_z = 0;

    NZEROS = 4; NPOLES = 4;
    xVect_linDisp_x = [0,0,0,0,0];
    yVect_linDisp_x = [0,0,0,0,0];
    xVect_linDisp_y = [0,0,0,0,0];
    yVect_linDisp_y = [0,0,0,0,0];
    xVect_linDisp_z = [0,0,0,0,0];
    yVect_linDisp_z = [0,0,0,0,0];
  }

  this.reset = function(){
    dispFltr_x.setupKalmanFilter();
    dispFltr_y.setupKalmanFilter();
    dispFltr_z.setupKalmanFilter();

    firstUpdate = 0;
    buffSize = 50;

    //TODO: List<double> translations?
    linAccBuff_x = [];
    linAccBuff_y = [];
    linAccBuff_z = [];

    //Quaternion orientation of earth frame relative to auxiliary frame.
    AEq_1 = 1;
    AEq_2 = 0;
    AEq_3 = 0;
    AEq_4 = 0;

    //Estimated orientation quaternion elements with initial conditions.
    SEq_1 = 1;
    SEq_2 = 0;
    SEq_3 = 0;
    SEq_4 = 0;

    linVel_x = 0; linVel_y = 0; linVel_z = 0;
    linVelHP_x = 0; linVelHP_y = 0; linVelHP_z = 0;

    linDisp_x = 0; linDisp_y = 0; linDisp_z = 0;
    linDispHP_x = 0; linDispHP_y = 0; linDispHP_z = 0; 
    

    linAcc = [0,0,-9.81];
    tcAcc = [0,0,0];

    NZEROS = 2;
    NPOLES = 2;
    xVect_linVel_x = [0,0,0];
    yVect_linVel_x = [0,0,0];
    xVect_linVel_y = [0,0,0];
    yVect_linVel_y = [0,0,0];
    xVect_linVel_z = [0,0,0];
    yVect_linVel_z = [0,0,0];

    NZEROS = 2; NPOLES = 2;
    xVect_linDisp_x = [0,0,0];
    yVect_linDisp_x = [0,0,0];
    
    NZEROS = 4; NPOLES = 4;
    xVect_linDisp_y = [0,0,0,0,0];
    yVect_linDisp_y = [0,0,0,0,0];
    xVect_linDisp_z = [0,0,0,0,0];
    yVect_linDisp_z = [0,0,0,0,0];

    NZEROS = 4; NPOLES = 4;
    xVect_tcAcc_x = [0,0,0,0,0];
    yVect_tcAcc_x = [0,0,0,0,0];
    xVect_tcAcc_y = [0,0,0,0,0];
    yVect_tcAcc_y = [0,0,0,0,0];
    xVect_tcAcc_z = [0,0,0,0,0];
    yVect_tcAcc_z = [0,0,0,0,0];
  }

  this.updateFilter = function(w_x, w_y, w_z, a_x, a_y, a_z){

    //Local system variables.

    //Vector norm.
    var norm=0;
    //Quaternion rate from gyroscope elements.
    var SEqDot_omega_1=0,
        SEqDot_omega_2=0,
        SEqDot_omega_3=0,
        SEqDot_omega_4=0;
    //Objective function elements.
    var f_1=0,
        f_2=0,
        f_3=0;
    //Objective function Jacobian elements.
    var J_11or24=0,
        J_12or23=0,
        J_13or22=0,
        J_14or21=0,
        J_32=0,
        J_33=0;
    //Objective function gradient elements.
    var nablaf_1=0,
        nablaf_2=0,
        nablaf_3=0,
        nablaf_4=0;

    //Auxiliary variables to avoid reapeated calcualtions.
    var halfSEq_1 = 0.5 * SEq_1,
        halfSEq_2 = 0.5 * SEq_2,
        halfSEq_3 = 0.5 * SEq_3,
        halfSEq_4 = 0.5 * SEq_4,
        twoSEq_1 = 2.0 * SEq_1,
        twoSEq_2 = 2.0 * SEq_2,
        twoSEq_3 = 2.0 * SEq_3;

    //Compute the quaternion rate measured by gyroscopes.
    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;

    //Normalise the accelerometer measurement.
    norm = Math.sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x /= norm;
    a_y /= norm;
    a_z /= norm;

    //Compute the objective function and Jacobian.
    f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
    f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
    f_3 = 1.0 - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;

    //J_11 negated in matrix multiplication.
    J_11or24 = twoSEq_3;
    J_12or23 = 2 * SEq_4;
    //J_12 negated in matrix multiplication
    J_13or22 = twoSEq_1;
    J_14or21 = twoSEq_2;
    //Negated in matrix multiplication.
    J_32 = 2 * J_14or21;
    //Negated in matrix multiplication.
    J_33 = 2 * J_11or24;

    //Compute the gradient (matrix multiplication).
    nablaf_1 = J_14or21 * f_2 - J_11or24 * f_1;
    nablaf_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
    nablaf_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
    nablaf_4 = J_14or21 * f_1 + J_11or24 * f_2;

    //Normalise the gradient.
    norm = Math.sqrt(nablaf_1 * nablaf_1 + nablaf_2 * nablaf_2 + nablaf_3 * nablaf_3 + nablaf_4 * nablaf_4);
    nablaf_1 /= norm;
    nablaf_2 /= norm;
    nablaf_3 /= norm;
    nablaf_4 /= norm;

    //Compute then integrate the estimated quaternion rate.
    SEq_1 += (SEqDot_omega_1 - (beta * nablaf_1)) * deltat;
    SEq_2 += (SEqDot_omega_2 - (beta * nablaf_2)) * deltat;
    SEq_3 += (SEqDot_omega_3 - (beta * nablaf_3)) * deltat;
    SEq_4 += (SEqDot_omega_4 - (beta * nablaf_4)) * deltat;

    //Normalise quaternion
    norm = Math.sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
    SEq_1 /= norm;
    SEq_2 /= norm;
    SEq_3 /= norm;
    SEq_4 /= norm;

    //Quaternion describing orientation of sensor relative to earth.
    var ESq_1, ESq_2, ESq_3, ESq_4;
    //Quaternion describing orientation of sensor relative to auxiliary frame.
    ASq_1 = 0; ASq_2 = 0; ASq_3 = 0; ASq_4 = 0;

    //Compute the quaternion conjugate.
    ESq_1 = SEq_1;
    ESq_2 = -SEq_2;
    ESq_3 = -SEq_3;
    ESq_4 = -SEq_4;

    //Compute the quaternion product.
    ASq_1 = ESq_1 * AEq_1 - ESq_2 * AEq_2 - ESq_3 * AEq_3 - ESq_4 * AEq_4;
    ASq_2 = ESq_1 * AEq_2 + ESq_2 * AEq_1 + ESq_3 * AEq_4 - ESq_4 * AEq_3;
    ASq_3 = ESq_1 * AEq_3 - ESq_2 * AEq_4 + ESq_3 * AEq_1 + ESq_4 * AEq_2;
    ASq_4 = ESq_1 * AEq_4 + ESq_2 * AEq_3 - ESq_3 * AEq_2 + ESq_4 * AEq_1;

    //Store latest Quat values
    Quat_w = ASq_1;
    Quat_x = -ASq_2;
    Quat_y = ASq_4;
    Quat_z = -ASq_3;

    if (firstUpdate == 0){
      //Store orientation of auxiliary frame.
      AEq_1 = SEq_1;
      AEq_2 = SEq_2;
      AEq_3 = SEq_3;
      AEq_4 = SEq_4;
      firstUpdate = 1;
    }
  }

  this.computeRotMatrix = function(){
    //compute the Rotation matrix out or the quaternion
    rotMatrix[0][0] = ((2 * Math.pow(ASq_1, 2)) - 1) + (2 * Math.pow(ASq_2, 2));
    rotMatrix[0][1] = 2 * ((ASq_2 * ASq_3) + (ASq_1 * ASq_4));
    rotMatrix[0][2] = 2 * (ASq_2 * ASq_4 - ASq_1 * ASq_3);
    rotMatrix[1][0] = 2 * (ASq_2 * ASq_3 - ASq_1 * ASq_4);
    rotMatrix[1][1] = ((2 * Math.pow(ASq_1, 2)) - 1) + (2 * Math.pow(ASq_3, 2));
    rotMatrix[1][2] = 2 * ((ASq_3 * ASq_4) + (ASq_1 * ASq_2));
    rotMatrix[2][0] = 2 * ((ASq_2 * ASq_4) + (ASq_1 * ASq_3));
    rotMatrix[2][1] = 2 * ((ASq_3 * ASq_4) - (ASq_1 * ASq_2));
    rotMatrix[2][2] = ((2 * Math.pow(ASq_1, 2)) - 1) + (2 * Math.pow(ASq_4, 2));
  }
  this.computeEuler = function(){
    //Compute the Euler angles from the quaternion.
    phi = Math.atan2(2 * ASq_3 * ASq_4 - 2 * ASq_1 * ASq_2, 2 * ASq_1 * ASq_1 + 2 * ASq_4 * ASq_4 - 1);
    theta = Math.asin(2 * ASq_2 * ASq_3 - 2 * ASq_1 * ASq_3);
    psi = Math.atan2(2 * ASq_2 * ASq_3 - 2 * ASq_1 * ASq_4, 2 * ASq_1 * ASq_1 + 2 * ASq_2 * ASq_2 - 1);
  }
  this.computeDisplacement = function(a_x, a_y, a_z)
  {
      var dampningFactor = 0.5;
      //Compute tilt componsated acceleration in m/s^2
      var blaFact = 0.5;
      tcAcc[0] =
          rotMatrix[0][0] * a_x +
          rotMatrix[0][1] * a_y * blaFact +
          rotMatrix[0][2] * a_z * blaFact;
      tcAcc[1] =
          rotMatrix[1][0] * a_x * blaFact +
          rotMatrix[1][1] * a_y +
          rotMatrix[1][2] * a_z * blaFact;
      tcAcc[2] =
          rotMatrix[2][0] * a_x * blaFact +
          rotMatrix[2][1] * a_y * blaFact +
          rotMatrix[2][2] * a_z;
      
      //convert tcAcc to linAcc. The filtering gets rid of the gravity from the z axsis cause it's basically a DC offset
      linAcc[0] = BPFilterLoop(tcAcc[0], xVect_tcAcc_x, yVect_tcAcc_x);
      linAcc[1] = BPFilterLoop(tcAcc[1], xVect_tcAcc_y, yVect_tcAcc_y);
      linAcc[2] = BPFilterLoop(tcAcc[2], xVect_tcAcc_z, yVect_tcAcc_z);
      

      //Moving windows Avg for smoothing
      if (linAccBuff_x.length > buffSize){
        linAccBuff_x.splice(buffSize, 1);
        //linAccBuff_x.RemoveAt(buffSize);
        linAccBuff_x = [linAcc[0]].concat(linAccBuff_x);
        //linAccBuff_x.Insert(0, linAcc[0]);
      }
      else{
        linAccBuff_x = [linAcc[0]].concat(linAccBuff_x);
        //linAccBuff_x.Insert(0, linAcc[0]);
      }
      //linAcc[0] = linAccBuff_x.Average();
      var sum=0;
      for(var i=linAccBuff_x.length; i>=0; i--){
        sum+=linAccBuff_x[i];
      }
      linAcc[0]=sum/linAccBuff_x.length;
      if (linAccBuff_y.length > buffSize){
        linAccBuff_y.splice(buffSize, 1);
        linAccBuff_y = [linAcc[1]].concat(linAccBuff_y);
      }
      else{
        linAccBuff_y = [linAcc[1]].concat(linAccBuff_y);
      }

      var sum=0;
      for(var i=linAccBuff_y.length; i>=0; i--){
        sum+=linAccBuff_y[i];
      }
      linAcc[1]=sum/linAccBuff_y.length;

      if (linAccBuff_z.length > buffSize){
        linAccBuff_z.splice(buffSize, 1);
        linAccBuff_z = [linAcc[2]].concat(linAccBuff_z);
      }
      else{
        linAccBuff_z = [linAcc[2]].concat(linAccBuff_z);
      }

      var sum=0;
      for(var i=linAccBuff_z.length; i>=0; i--){
        sum+=linAccBuff_z[i];
      }
      linAcc[2]=sum/linAccBuff_z.length;

      var deadBand = 0.1;
      //deadband Filter
      linAcc[0] = (linAcc[0] > deadBand || linAcc[0] < -deadBand) ? linAcc[0] : 0;
      linAcc[1] = (linAcc[1] > deadBand || linAcc[1] < -deadBand) ? linAcc[1] : 0;
      linAcc[2] = (linAcc[2] > deadBand || linAcc[2] < -deadBand) ? linAcc[2] : 0;
      
      //integrate linAcc to get linVel in cm/s
      linVel_x += linAcc[0] * deltat * 10 * dampningFactor;
      linVel_y += linAcc[1] * deltat * 10 * dampningFactor;
      linVel_z += linAcc[2] * deltat * 10 * dampningFactor;

      //high pass Filter Vel
      linVelHP_x = HPFilterLoop_1(linVel_x, xVect_linVel_x, yVect_linVel_x);
      linVelHP_y = HPFilterLoop_1(linVel_y, xVect_linVel_y, yVect_linVel_y);
      linVelHP_z = HPFilterLoop_1(linVel_z, xVect_linVel_z, yVect_linVel_z);

      //dead band filter
      deadBand = 0.1;

      linVelHP_x = (linVelHP_x > deadBand || linVelHP_x < -deadBand) ? linVelHP_x : 0;
      linVelHP_y = (linVelHP_y > deadBand || linVelHP_y < -deadBand) ? linVel_y : 0;
      linVelHP_z = (linVelHP_z > deadBand || linVelHP_z < -deadBand) ? linVel_z : 0;
             

      //integrate to get linDisp in cm
      linDisp_x += linVelHP_x * deltat * 10 * dampningFactor;
      linDisp_y += linVelHP_y * deltat * 10 * dampningFactor;
      linDisp_z += linVelHP_z * deltat * 10 * dampningFactor;

      var maxLinDisp_x = 25, maxLinDisp_y = 110, maxLinDisp_z = 50;
      linDisp_x = linDisp_x > maxLinDisp_x ? maxLinDisp_x : linDisp_x;
      linDisp_y = linDisp_y > maxLinDisp_y ? maxLinDisp_y : linDisp_y;
      linDisp_z = linDisp_z > maxLinDisp_z ? maxLinDisp_z : linDisp_z;
      

      dispFltr_x.updateFilter(tcAcc[0] * 10, linDisp_x);
      dispFltr_y.updateFilter(tcAcc[1] * 10, linDisp_y);
      dispFltr_z.updateFilter(tcAcc[2] * 10, linDisp_z);

      //kalmanDisp_x = kalmanFilter(linAcc[0] * 10, linDisp_x);
      kalmanDisp_x = HPFilterLoop_3(dispFltr_x.dispEstimate(), xVect_linDisp_x, yVect_linDisp_x);
      kalmanDisp_y = HPFilterLoop_3(dispFltr_y.dispEstimate(), xVect_linDisp_y, yVect_linDisp_y);
      kalmanDisp_z = HPFilterLoop_3(dispFltr_z.dispEstimate(), xVect_linDisp_z, yVect_linDisp_z);
  }

  //possible states are: Calib, Zerod, Set_1, Set_2
  this.ComputeVelDirFltr = function(curAcc){
    for(var i = 0; i <3; i++){
      zeroCount[i] = curAcc[i] == 0 ? (zeroCount[i] + 1) : 0;
      if (zeroCount[i] > 100){
          zeroCount[i] = 0;
          fltrState[i] = "Calib";
          //Console.WriteLine("Filter State Zerod");
      }
      switch (fltrState[i]){
        //calib state just waits in the very begining for the acceleration to drift to zero
        case "Calib":
          if (curAcc[i] == 0)
            fltrState[i] = "Zerod";
          velFltr[i] = 0;
          break;

        //zerod is for calibrated signal when device is stationary it monitors for the initial device acc direction
        case "Zerod":
          if (curAcc[i] != 0){
              accelArea[i] = 0;
              fltrState[i] = "Set_1";
          }
          velFltr[i] = (curAcc[i] > 0) ? 1 : (curAcc[i] < 0 ? -1 : 0);
          break;

        //Set_1 is when device has accelerated in one direction and we are waiting for it to decelerate back to zero
        case "Set_1":
          accelArea[i] += Math.abs(curAcc[i] * deltat);
          if ((curAcc[i] * velFltr[i]) < 0){
              decelArea[i] = 0;
              fltrState[i] = "Set_2";
          }
          break;
          
        //Set_2 is for a decelerating device we keep filter set until
        case "Set_2":
          decelArea[i] += Math.Abs(curAcc[i] * deltat);
          if ((decelArea[i] > (3 * accelArea[i])) || (curAcc[i] == 0)){
            /*if (decelArea[i] > (30 * accelArea[i]))
                Console.WriteLine("Filterzerod becuase we are higher than the 30 factor!");
            else
                Console.WriteLine("Filter zerod because Accel crossed zero");*/
            
            decelArea[i] = 0; accelArea[i] = 0; velFltr[i] = 0;
            fltrState[i] = "Zerod";

            if (i == 0){
              Array.Clear(xVect_linVel_x, 0, xVect_linVel_x.Length);
              Array.Clear(yVect_linVel_x, 0, yVect_linVel_x.Length); 
              linVel_x = 0;
            }
            if (i == 1){
              Array.Clear(xVect_linVel_y, 0, xVect_linVel_y.Length);
              Array.Clear(yVect_linVel_y, 0, yVect_linVel_y.Length); 
              linVel_y = 0;
            }
            if (i == 2){
              Array.Clear(xVect_linVel_z, 0, xVect_linVel_z.Length);
              Array.Clear(yVect_linVel_z, 0, yVect_linVel_z.Length); 
              linVel_z = 0;
            }
          }
          break;
      }
    }
  }

  /* Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher
   * Command line: /www/usr/fisher/helpers/mkfilter -Bu -Hp -o 2 -a 1.0000000000e-01 0.0000000000e+00 -l
   * http://www-users.cs.york.ac.uk/~fisher/mkfilter/trad.html
   * HIGH PASS FILTER WAS DESIGNED FOR Fc of 10 Hz and SampleRate of 100 HZ. 2nd order butter worth
   */

  //HIGH PASS FILTER WAS DESIGNED FOR Fc of .8 Hz and SampleRate of 100 HZ. 2nd order butter worth
  
  function HPFilterLoop_1(filterIn, xv, yv){
      xv[0] = xv[1];
      xv[1] = xv[2];
      xv[2] = (filterIn) / 1.003560630; //1.045431062;

      yv[0] = yv[1];
      yv[1] = yv[2];
      yv[2] = (xv[0] + xv[2]) - 2 * xv[1]
          //+ (-0.9149758348 * yv[0]) + (1.9111970674 * yv[1]);       
          + (-0.9929165937 * yv[0]) + (1.9928914171 * yv[1]);

      return (yv[2]);
  }

  //1st order 0.1 HZ high pass
  function HPFilterLoop_2(filterIn, xv, yv){
      xv[0] = xv[1];
      xv[1] = filterIn / 1.003141603;
      yv[0] = yv[1];
      yv[1] = (xv[1] - xv[0])
               + (0.9937364715 * yv[0]);
      return (yv[1]);
  }
  
  //2nd order 1 HZ high pass
 function HPFilterLoop_3(filterIn, xv, yv){
      xv[0] = xv[1];
      xv[1] = xv[2];
      xv[2] = (filterIn) / 0.845431062;

      yv[0] = yv[1];
      yv[1] = yv[2];
      yv[2] = (xv[0] + xv[2]) - 2 * xv[1]
               + (-0.9149758348 * yv[0]) + (1.9111970674 * yv[1]);

      return (yv[2]);
  }


  //BP 2nd order Butterworth 0.1-15 HZ
  function BPFilterLoop(filterIn, xv, yv){
      xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4];
      xv[4] = filterIn /  7.536374524;
      yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4];
      yv[4] = (xv[0] + xv[4]) - 2 * xv[2]
                   + (-0.2743409743 * yv[0]) + (1.3000150761 * yv[1])
                   + (-2.7723890658 * yv[2]) + (2.7466942260 * yv[3]);
      return (yv[4]);
  }

  this.getRotMatrix = function(){
    return rotMatrix;
  }

  this.getRoll = function(){
    //convert to Degrees because they make more sense
    return (theta * (180 / Math.PI));
  }
  this.getPitch = function(){
    return (phi * (180 / Math.PI));
    //return theta;
  }
  this.getYaw = function(){
    return (psi * (180 / Math.PI));
    //return psi;
  }

  this.getQuat_w = function(){
    return (Quat_w);
  }
  this.getQuat_x = function(){
    return (Quat_x);
  }
  this.getQuat_y = function(){
    return (Quat_y);
  }
  this.getQuat_z = function(){
    return (Quat_z);
  }

  this.getLinVel_x = function(){
    return (linVel_x);
  }
  this.getLinVel_y = function(){
    return (linVel_y);
  }
  this.getLinVel_z = function(){
    return (linVel_z);
  }

  this.getLinVelHP_x = function(){
    return (linVelHP_x);
  }
  this.getLinVelHP_y = function(){
    return (linVelHP_y);
  }
  this.getLinVelHP_z = function(){
    return (linVelHP_z);
  }

  
  this.getLinDisp_x = function(){
    //return (linDisp_x);
    return (kalmanDisp_x);
  }
  this.getLinDisp_y = function(){
    //return (linDisp_y);
    //return (linDispHP_y);
    return (kalmanDisp_y);
  }
  this.getLinDisp_z = function(){
    //return (linDisp_z);
    //return (linDispHP_z);
    return (kalmanDisp_z);
  }

  this.getTCAcc_x = function(){
    return (linAcc[0]);
    //return (tcAcc[0]);
  }
  this.getTCAcc_y = function(){
    return (linAcc[1]);
    //return (tcAcc[1]);
  }
  this.getTCAcc_z = function(){
    return (linAcc[2]);
    //return (tcAcc[2]);
  }
  init(rate, gi);
};
