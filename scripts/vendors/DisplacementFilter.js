function DisplacementFilter(sampleRate){
  var _this = this;
  
  //Define update equations (Coefficent matrices): A physics based model for where we expect the device to be [state transition (state + velocity)] + [input control (acceleration)]
  var A, B, //double[,] 
      C, //double[]
      //initized state--it has two components: [position; velocity]
      S, //double[,]
      //x_estimate of initial location estimation of where the device is (what we are updating)
      S_estimate,    //double[,]
      predict_state, //double[,] 
      modelNoiseMag,
      deviceNoiseMag,
      Ex, P, //double[,]
      //Ex is the covariance representation of the physics model error (stDev); P is the first estimate of that covariance
      Ez,
      //Ez is the covariance representation of the device error (stDev)
      predict_var, //double[,] 
      K, //double[,] 
      // kalman filter gain 
      Iden,
      //sampling rate
      deltat;
 
  this.setupKalmanFilter = function(_modelNoise, _deviceNoise){
    _modelNoise = _modelNoise || 0.05; //Doubles
    _deviceNoise = _deviceNoise || 10; //Doubles
    A = [[1,deltat],[0,1]];
    B = [[Math.pow(deltat,2)/2], [deltat]];
    C = [1,0];

    S = [[0],[0]];
    //S_estimate = [[],[]];
    S_estimate = S;
    predict_state = [[],[]];

    //These two factors affect the behavior of the kalman filter and should be adjusted with trial and error
    modelNoiseMag = _modelNoise;
    deviceNoiseMag = _deviceNoise;
    Ez = Math.pow(deviceNoiseMag, 2);

    var temp_NoiseSquared = Math.pow(modelNoiseMag, 2); //temp var for readability

    Ex = [[Math.pow(deltat,4)/4,Math.pow(deltat,3)/2],[Math.pow(deltat,3)/2,Math.pow(deltat,2)]];

    Ex = Ex.map(function(arr){
      return arr.map(function(num){
        return num *temp_NoiseSquared;
      });
    });

    P = JSON.parse(JSON.stringify(Ex));

    predict_var = [[],[]];
    K = [[],[]];
  }

  this.updateFilter = function(AccelIn, DispIn){
    //predict the next state of the device with the last state and predicted motion
    //S_estimate = A * S_estimate + B * AccelIn
    S_estimate[0][0] = S_estimate[0][0] * A[0][0] + S_estimate[1][0] * A[0][1] + B[0][0] * AccelIn;
    S_estimate[1][0] = S_estimate[0][0] * A[1][0] + S_estimate[1][0] * A[1][1] + B[1][0] * AccelIn;
    //predict next coveriance: P = A * P * A' + Ex;
    var A_t = [[ A[0][0], A[1][0] ], [ A[0][1], A[1][1] ]];
    var P_temp = [[],[]];
    //A*P
    var _x;
    for (var i = 0; i < 4; i++)
    {
        _x = i > 1 ? 1 : 0;
        P_temp[i%2][_x] = A[i % 2][0] * P[0][_x] + A[i%2][1] * P[1][_x];
    }
    //P_temp * A_t
    for (var i = 0; i < 4; i++)
    {
        _x = i > 1 ? 1 : 0;
        P[i%2][_x] = P_temp[i%2][0] * A_t[0][_x] + P_temp[i%2][1] * A_t[1][_x];
    }

    // + Ex
    P[0][0] = P[0][0] + Ex[0][0];
    P[0][1] = P[0][1] + Ex[0][1];
    P[1][0] = P[1][0] + Ex[1][0];
    P[1][1] = P[1][1] + Ex[1][1];

    //Kalman Gain: K = P*C' * inv(C*P*C' + Ez);
    var tempVal = ((C[0] * P[0][0]) * C[0]) + Ez;
    var tempVal_inv = 1 / tempVal;

    K[0][0] = ((P[0][0] * C[0]) + (P[0][1] * C[1])) * tempVal_inv;
    K[1][0] = ((P[1][0] * C[0]) + (P[1][1] * C[1])) * tempVal_inv;

    //update the state estimate: S_estimate = S_estimate + K * (DispIn - C * S_estimate)
    tempVal = DispIn - (C[0] * S_estimate[0][0]);
    S_estimate[0][0] = S_estimate[0][0] + K[0][0] * tempVal;
    S_estimate[1][0] = S_estimate[1][0] + K[1][0] * tempVal;

    //update covariance estimation: (I - K*C) * P
    //I - K*C
    P_temp[0][0] = Iden[0][0] - (K[0][0] * C[0]);
    P_temp[0][1] = Iden[0][1] - 0;
    P_temp[1][0] = Iden[1][0] - (K[1][0] * C[0]);
    P_temp[1][1] = Iden[1][1] - 0;

    //P_temp * P
    for (var i = 0; i < 4; i++)
    {
        _x = i > 1 ? 1 : 0;
        P[i%2][_x] = (P_temp[i%2][0] * P[0][_x]) + (P_temp[i%2][1] * P[1][_x]);
    }
  }

  this.dispEstimate = function(){
    return S_estimate[0][0];
  }

  function init(sr){
    deltat = sr;
    Iden = [[1, 0], [0, 1]];
    _this.setupKalmanFilter();
  }
  init(sampleRate);
};