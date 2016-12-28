//------------------------------------------------------------
//    姿勢制御フィルタリングプログラム
//                Arduino　IDE　1.6.11
//
//　　　Arduino　　　　　　　　LSM9DS1基板　
//　　　　3.3V　　　------　　　　3.3V
//　　　　GND       ------   　　 GND
//　　　　SCL       ------        SCL
//　　　　SDA       ------        SDA
//
//　センサーで取得した値をシリアルモニターに表示する
//
//　　　　
//----------------------------------------------------------//


#include <SPI.h>                        //SPIライブラリ
#include <Wire.h>                       //I2Cライブラリ
#include <SparkFunLSM9DS1.h>          //LSM9DS1ライブラリ：https://github.com/sparkfun/LSM9DS1_Breakout
#include <SD.h>
#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SoftwareSerial.h>                  
#include <MadgwickAHRS.h>               //MadgwickAHRS : https://github.com/arduino-libraries/MadgwickAHRS
#include <Kalman.h>               //KalmanFilter : https://github.com/TKJElectronics/KalmanFilter

///////////////////////カルマンフィルタ/////////////////////////////
//Madgwick filter;
Kalman kalmanX; // instances
Kalman kalmanY; // instances
unsigned long time;
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double accX, accY, accZ; 
double gyroX, gyroY, gyroZ; 
double magX, magY, magZ; 
float roll, pitch; 
////////////////////////////////////////////////////////////////

//#define ADAddr 0x48//

#define LSM9DS1_M  0x1E                 // SPIアドレス設定 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B                // SPIアドレス設定 if SDO_AG is LOW

//#define PRINT_CALCULATED              //表示用の定義
//#define DEBUG_GYRO                    //ジャイロスコープの表示

#define DECLINATION -8.58               // Declination (degrees) in Boulder, CO.


#define RX 8                            //GPS用のソフトウェアシリアル
#define TX 9                            //GPS用のソフトウェアシリアル
#define SENTENCES_BUFLEN      82        // GPSのメッセージデータバッファの個数

//#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf 

//-------------------------------------------------------------------------
//[Global valiables]

LSM9DS1 imu;
int SAMPLETIME = 10;
int RECORD_INTERVAL = 100;
int WRITE_INTERVAL = 1000;

//###############################################
//MicroSD 
const int chipSelect = 4;//Arduino UNO
//const int chipSelect = 10;//Arduino Micro
//###############################################

const int tact_switch = 7;//タクトスイッチ
boolean switchIs;
boolean switchOn;
boolean switchRelease;

//ジャイロセンサーの積分値
//float pitch_g = 0.0;
//float roll_g = 0.0;

//相補フィルタの保持値
float prev_pitch = 0.0;
float prev_roll = 0.0;

String motionData;

//----------------------------------------------------------------------


void setup(void) {

  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  //=== SD Card Initialize ====================================
  Serial.print(F("Initializing SD card..."));

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Card failed, or not present"));
    // don't do anything more:
    return;
  }
  Serial.println(F("card initialized."));

  //=======================================================

  //タクトスイッチ
  pinMode(tact_switch, INPUT);
  switchIs = false;

  //LED
  pinMode(13, OUTPUT);

  //=== LSM9DS1 Initialize =====================================
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress  = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  if (!imu.begin())              //センサ接続エラー時の表示
  {
    Serial.println(F("Failed to communicate with LSM9DS1."));
    while (1)
      ;
  }
  //=======================================================
delay(100); // Wait for sensor to stabilize
//filter.begin(500);

//初期値計算
initCalmanFilter();

}

/**
 * loop
 * ずっと繰り返される関数（何秒周期？）
 * 【概要】
 * 　10msでセンサーデータをサンプリング。
 * 　記録用に、100ms単位でデータ化。
 * 　蓄積したデータをまとめて、1000ms単位でSDカードにデータを出力する。
 * 　
 */
void loop(void) {

  //START switch============================================
  switch(digitalRead(tact_switch)){

   case 0://ボタンを押した

          switchOn = true;

          break;

   case 1://ボタン押していない

           if(switchOn){

             //すでにOnなら、falseにする
             if(switchIs)
               switchIs = false;
             //すでにOffなら、trueにする
             else
               switchIs = true;


             switchOn = false;
            
           }

           break;

    default:
           break;
  }

  //スイッチの判定
  if(!switchIs){ //falseなら、ループする
    digitalWrite(13, 0);
    return;
  }
  else{
      digitalWrite(13, 1);
  }
  //END switch ============================================

  //MotionSensorの値更新
  motionData = updateMotionSensors(true);

  //Serial.println(motionData);

  delay(2);

}




/**
 * updateMotionSensors
 */
String updateMotionSensors(boolean print)
{

  //Serial.println( millis() - time);
  //time = millis();

  
  //Read three sensors data on the memory
  readMotionSensors();

  //メモリ上の角度データの更新（前回値と今回値が考慮される）  
  //return printAttitude (imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz), imu.ax, imu.ay, imu.az, -imu.mx, -imu.my, imu.mz, print) + "\n";
  //return printAttitude (imu.gx, imu.gy, imu.gz, imu.ax, imu.ay, imu.az, -imu.mx, -imu.my, imu.mz, print) + "\n";
  return printAttitude(print) + "\n";

}




//--------------------　Motion DATA ------------------------------------
void readMotionSensors()
{

  imu.readGyro();
  imu.readAccel();
  imu.readMag();
}

//---------------------------------------------------------
/**
 * printAttitude
 * 取得したデータをシリアル出力する関数
 * gx : ジャイロスコープ X値 deg/s
 * gy : ジャイロスコープ Y値 deg/s
 * gz : ジャイロスコープ Z値 deg/s
 * ax : 加速度センサー X値
 * ay : 加速度センサー Y値
 * az : 加速度センサー Z値
 * mx : 地磁気センサー X値
 * my : 地磁気センサー Y値
 * mz : 地磁気センサー Z値
 * print : 値を返すかどうか
 */

String printAttitude(boolean print)
{

  String output = "";

  // [ g]
  accX = imu.calcAccel(imu.ax); 
  accY = imu.calcAccel(imu.ay); 
  accZ = imu.calcAccel(imu.az); 

  // [deg/s]
  gyroX = imu.calcGyro(imu.gx);
  gyroY = imu.calcGyro(imu.gy); 
  gyroZ = imu.calcGyro(imu.gz); 

  // [ gauss]
  magX = imu.calcMag(imu.mx); 
  magY = imu.calcMag(imu.my); 
  magZ = imu.calcMag(imu.mz); 


  //時間の更新
  double dt = (double)(millis() - time) / 1000; // Calculate delta time  
  time = millis();

#ifdef RESTRICT_PITCH // Eq. 25 and 26 
  roll  = atan2(accY, accZ) * RAD_TO_DEG;//+++++++++++++++++++++++ 
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG; 
#else // Eq. 28 and 29 
  roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG; 
  pitch = atan2(-accX, accZ) * RAD_TO_DEG; 
#endif


  //double gyroXrate = gyroX / 131.0; // Convert to deg/s 
  //double gyroYrate = gyroY / 131.0; // Convert to deg/s 
  //double gyroZrate = gyroZ / 131.0; // Convert to deg/s 

  double gyroXrate = imu.calcGyro(gyroX); // Convert to deg/s 
  double gyroYrate = imu.calcGyro(gyroY); // Convert to deg/s 
  double gyroZrate = imu.calcGyro(gyroZ); // Convert to deg/s 



#ifdef RESTRICT_PITCH 
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees 
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) { 
    kalmanX.setAngle(roll); 
    kalAngleX = roll; 
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter 
  
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); 
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees 
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) { 
    kalmanY.setAngle(pitch); 
    kalAngleY = pitch; 
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter 
  
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter 
#endif



float heading = 0;
//heading = atan2(magX, magY) * RAD_TO_DEG + 180;

    heading = atan2(magX, magY);

     if (heading < 0) heading += 2*PI ;
     if (heading > 2*PI) heading -= 2*PI;
     heading = heading * 180/M_PI ;
     // 西偏(日本)の場合で磁気偏角を調整する
     heading = heading + 6.6 ;// 磁気偏角6.6度
     if (heading > 360.0) heading = heading - 360.0 ;



/*
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);
*/

    Serial.print("CalmanFilter: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(kalAngleY);
    Serial.print(" ");
    Serial.print(kalAngleX);
    Serial.println(" ");

/*    filter.updateIMU(gyroXrate * RAD_TO_DEG, gyroYrate * RAD_TO_DEG, gyroZrate * RAD_TO_DEG,
      imu.calcAccel(accX), imu.calcAccel(accY), imu.calcAccel(accZ));


    // print the heading, pitch and roll
    double roll2 = filter.getRoll();
    double pitch2 = filter.getPitch();
    double heading2 = filter.getYaw();
    Serial.print("\t");
    Serial.print("MadgwickAHRS: ");
    Serial.print(heading2);
    Serial.print(" ");
    Serial.print(pitch2);
    Serial.print(" ");
    Serial.println(roll2);
*/

  return output;
}

/**
 * 
 */
void initCalmanFilter(){

  readMotionSensors();

  // update the filter, which computes orientation
  //filter.updateIMU(imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz), imu.ax, imu.ay, imu.az);
//  filter.updateIMU(imu.gx, imu.gy, imu.gz, imu.ax, imu.ay, imu.az);
  // print the heading, pitch and roll
//  double roll = filter.getRoll();
//  double pitch = filter.getPitch();
//  double heading = filter.getYaw();

  //accX = imu.ax; 
  //accY = imu.ay; 
  //accZ = imu.az; 
  accX = imu.calcAccel(imu.ax); 
  accY = imu.calcAccel(imu.ay); 
  accZ = imu.calcAccel(imu.az);

#ifdef RESTRICT_PITCH // Eq. 25 and 26 
  roll  = atan2(accY, accZ) * RAD_TO_DEG; 
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG; 
#else // Eq. 28 and 29 
  roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG; 
  pitch = atan2(-accX, accZ) * RAD_TO_DEG; 
#endif



  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch); // Set starting angle


  //時間の更新
  time = millis();
}


/**
 * 相補フィルタ
 * prev_val : 前回のOutput
 * deg_g : ジャイロセンサで得た角度
 * deg_a : 加速度センサーで得た角度
 */
 float complementFilter(float prev_val, float deg_g, float deg_a){
    return 0.95 * (prev_val + deg_g) + 0.05 * deg_a;
 }



