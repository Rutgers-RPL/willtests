#include <Arduino.h>
#include <BMI085.h>
#include <Quaternion.h>
#include <MEKFcRP.h>
#include <NormalDistribution.h>
#include <Vec3.h>
#include <DFRobot_BMM150.h>
#include <Ahrs.h>

#define _g_ (9.80665)

Bmi085Accel accel(Wire,0x18);
Bmi085Gyro gyro(Wire, 0x68);
DFRobot_BMM150_I2C bmm150(&Wire, 0x13);

Quaternion dcm2quat(Vec3 north, Vec3 east, Vec3 down){
  // dcm2quat https://intra.ece.ucr.edu/~farrell/AidedNavigation/D_App_Quaternions/Rot2Quat.pdf
  int maximum = 1;
  double q = (1+north.x+east.y+down.z);
  double o = (1+north.x-east.y-down.z);
  if (o > q){
    q = o;
    maximum = 2;
  }
  o = (1-north.x+east.y-down.z);
  if (o > q){
    q = o;
    maximum = 3;
  }
  o = (1-north.x-east.y+down.z);
  if (o > q){
    q = o;
    maximum = 4;
  }

  Quaternion output = Quaternion();
  switch(maximum){
    case 1:
      Serial.println(1);
      output.a = 0.5*sqrt(q);
      output.b = (down.y-east.z)/(4*output.a);
      output.c = (north.z-down.x)/(4*output.a);
      output.d = (east.x-north.y)/(4*output.a);
      break;
    case 2:
      Serial.println(2);
      output.b = 0.5*sqrt(q);
      output.a = (down.y-east.z)/(4*output.b);
      output.c = (north.y+east.x)/(4*output.b);
      output.d = (north.z+down.x)/(4*output.b);
      break;
    case 3:
      Serial.println(3);
      output.c = 0.5*sqrt(q);
      output.a = (north.z-down.x)/(4*output.c);
      output.b = (north.y+east.x)/(4*output.c);
      output.d = (east.z+down.y)/(4*output.c);
      break;
    case 4:
      Serial.println(4);
      output.d = 0.5*sqrt(q);
      output.a = (east.x-north.y)/(4*output.d);
      output.b = (north.z+down.x)/(4*output.d);
      output.c = (east.z+down.y)/(4*output.d);
      break;
  }

  return output;
}

// double accxOffset = 0;
// double accyOffset = 0;
// double acczOffset = 0;
// double accqOffset = 0;

// double sumx = 0;
// double sumy = 0;
// double sumz = 0;

// long previousTime = 0;

// void setup() {
//   previousTime = micros();
//   OrientationEstimator();

//   // put your setup code here, to run once:
//   /* accel object */
//   Serial.begin(9600);
//   int status = accel.begin();
//   if (status < 0) {
//     Serial.println("Accel Initialization Error");
//     Serial.println(status);
//     while (1) {}
//   }
//   Serial.println("Initializing accelerometer offsets");
//   double time1 = micros();
//   int n = 15000;
//   int s = 5000;
//   for(int i = 0; i < n; i++){
//     accel.readSensor();

//     float am[3] = {accel.getAccelX_mss(), accel.getAccelY_mss(), accel.getAccelZ_mss()};
//     float wm[3] = {gyro.getGyroX_rads(), gyro.getGyroY_rads(), gyro.getGyroZ_rads()};
//     float q[4];

//     updateIMU(am, wm, (micros() - previousTime) / 1000000);
//     previousTime = micros();
//     if(i < s){
//       continue;
//     }
//     get_q(q);

//     auto quat = Quaternion();
//     quat.a = q[0];
//     quat.b = q[1];
//     quat.c = q[2];
//     quat.d = q[3];
    
//     auto acc = Quaternion(am[0], am[1], am[2]);
//     acc.a = 0;

//     auto vec = quat.rotate(acc) + Quaternion(0, 0, -9.81);

//     sumx += vec.b;
//     Serial.println(sumx);
//     sumy += vec.c;
//     Serial.println(vec.c);
//     sumz += vec.d;
//     Serial.println(vec.d);

//     Serial.printf("Iter: %d\n", i);
//   }
//   accxOffset = sumx/(n-s);
//   accyOffset = sumy/(n-s);
//   acczOffset = sumz/(n-s);
//   Serial.printf("Time to find offsets: %f\n", (micros()-time1)/1000000);
//   Serial.printf("ACCEL OFFSETS x: %f   y: %f   z: %f\n", accxOffset, accyOffset, acczOffset);
// }

// double stdSumx = 0;
// double count = 0;

// void loop() {
//   // put your main code here, to run repeatedly:
//   /* read the accel */
//   count+=1;
//   accel.readSensor();
//   stdSumx += pow(accel.getAccelX_mss() - accxOffset, 2);

//   Serial.printf("AVG ACCEL x: %f   y: %f    z: %f\n", accel.getAccelX_mss() - accxOffset, accel.getAccelY_mss() - accyOffset, accel.getAccelZ_mss() - accqOffset);
//   //Serial.printf("stdev x: %f", sqrt(stdSumx/count));

// }

// const int nrolls=10000;  // number of experiments
// const int nstars=100;    // maximum number of stars to distribute

// #define ACCELXYRMS 0.04118793
// #define NLMAX 0.784532
// NormalDistribution accelNormXY(0,ACCELXYRMS);
// int trials = 100;
// int iters = 10;
// long previousTime = 0;
// void setup() {
//   previousTime = micros();
//   OrientationEstimator();
//   Serial.begin(9600);
//   int status;
//   status = accel.begin();
//   if (status < 0) {
//     Serial.println("Accel Initialization Error");
//     Serial.println(status);
//     while (1) {}
//   }
//   status = gyro.begin();
//   if (status < 0) {
//     Serial.println("Gyro Initialization Error");
//     Serial.println(status);
//     while (1) {}
//   }
// }

// void loop(){
//   double xsum = 0;
//   // for (int i=0; i < trials; i++) {
//   //   double xnl = random(-1*LONGMAX,LONGMAX)/LONGMAX;
//   //   for (int j = 0; j < iters; j++){
//   //     double xrms = accelNormXY.sample();
//   //     accel.readSensor();
//   //     double aact = ((accel.getAccelX_mss())/(1+NLMAX*xnl))+xrms;
//   //     Serial.println(aact);
//   //     xsum += aact;
//   //   }
//   // }
//   //Serial.println(xsum/(trials*iters));
//   for (int i = 0; i < 1000; i++){
//     accel.readSensor();
//     xsum += accel.getAccelX_mss();
//   }
//   Serial.println((double)(xsum/1000));
//   // for (int i=0; i<10; ++i) {
//   //   Serial.printf("%d-%d:", i, (i+1));
//   //   Serial.println(p[i]*nstars/nrolls);
//   // }
// }

Ahrs thisahrs;
double threshold = 0.5;

long count = 0;
double oldavg = 0;
double newavg = 0;

void setup(){
  Serial.begin(9600);
  int status = accel.begin();
  if (status < 0){
    while(1){Serial.println("accel problem"); delay(100);}
  }
  status = gyro.begin();
  if (status < 0) {
    while(1){Serial.println("gyro problem"); delay(100);}
  }
  bmm150.begin();
  bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
  bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
  bmm150.setRate(BMM150_DATA_RATE_10HZ);
  bmm150.setMeasurementXYZ();

  
}


Quaternion orientation = Quaternion();
long lastTime = micros();

void loop() {
  accel.readSensor();
  Vec3 acc = Vec3(accel.getAccelX_mss(),accel.getAccelY_mss(),accel.getAccelZ_mss());

  sBmm150MagData_t magData = bmm150.getGeomagneticData();
  Vec3 mag(magData.x,magData.y,magData.z);

  gyro.readSensor();
  Vec3 gyr = Vec3(gyro.getGyroX_rads(),gyro.getGyroY_rads(),gyro.getGyroZ_rads());

  thisahrs.update(acc,gyr,mag);
  orientation = thisahrs.q;

  // double dt = ((double) (micros() - lastTime)) / 1000000;
  // if(acc.magnitude()<_g_+threshold && acc.magnitude()>_g_-threshold){
  //   Vec3 down = acc*(-1);
  //   down.normalize();
  //   Vec3 east = down.cross(mag);
  //   east.normalize();
  //   Vec3 north = east.cross(down);
  //   north.normalize();

  //   orientation = dcm2quat(north, east, down);
  // } else {
  // //   Serial.println("asdfghjklkjhgfdsasdfghjkl");
  // //   orientation *= Quaternion::from_euler_rotation(gyro.getGyroX_rads()*dt, gyro.getGyroY_rads()*dt, gyro.getGyroZ_rads()*dt);
  // }
  // lastTime = micros();
  // Serial.printf("orientation: %f, %f, %f, %f\n", orientation.a, orientation.b, orientation.c, orientation.d);
  
  Quaternion accGroundFrame = thisahrs.aglobal;

  // Serial.println("-----");
  count++;
  if(count > 1500)
  {
    newavg = (oldavg * (count-1) + sqrt(pow(accGroundFrame.b,2) + pow(accGroundFrame.c,2) + pow(accGroundFrame.d,2)))/count;
    //Serial.printf("mean: %f\n", newavg);
    oldavg = newavg;  
  }
  
  Serial.printf("rotated acc: %f, %f, %f, %f, %f\n", accGroundFrame.a, accGroundFrame.b, accGroundFrame.c, accGroundFrame.d, sqrt(pow(accGroundFrame.b,2) + pow(accGroundFrame.c,2) + pow(accGroundFrame.d,2)));
  // Serial.printf("angle %f\n", bmm150.getCompassDegree());
  // Serial.printf("orientation: %f, %f, %f\n", pitch, roll, yaw);
  // Serial.printf("mag: %f, %f, %f\n", mag.x, mag.y, mag.z);
  // Serial.printf("acc: %f, %f, %f\n", acc.x, acc.y, acc.z);

  // double accxsum = 0;
  // double accysum = 0;
  // double acczsum = 0;
  // int count = 0;
  // for(int i = 0; i<20; i++){
  //   count+=1;
  //   accxsum+=acc.x;
  //   accysum+=acc.y;
  //   acczsum+=acc.z;
  // }
  // Serial.printf("accavgs: %f, %f, %f\n", accxsum/count, accysum/count, acczsum/count);

  //exit(0);
  // Serial.printf("Down: %f, %f, %f\n", down.x, down.y, down.z);
  // Serial.printf("East: %f, %f, %f\n", east.x, east.y, east.z);
  // Serial.printf("North: %f, %f, %f\n", north.x, north.y, north.z);
}

