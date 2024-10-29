#include <GNSS.h>
#include <cstdio>
#include <stdlib.h>
#include "Class.h"
#include <iostream>
#include <math.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Arduino.h>
#include <SDHCI.h>
#include <File.h>
#include <Camera.h>
#include <EEPROM.h>
#include <DNNRT.h>
//マクロ宣言
#define STRING_BUFFER_SIZE 128
#define RESTART_CYCLE (60 * 5)
#define PI (atan(1.0) * 4.0)
#define DEG2RAD (PI / 180.0)
#define RE (6387137)  //m
#define NUM1 2
#define NUM2 480
#define NUM3 5
#define ledBlue 11
#define ledRed 12
#define SEALEVELPRESSURE_HPA (1013.25)
#define DNN_IMG_W 28  //以下カメラ
#define DNN_IMG_H 28
#define DNN_IMG_C 3  // 3チャンネル（RGB）
#define CAM_CLIP_X 104
#define CAM_CLIP_Y 0
#define CAM_CLIP_W 112
#define CAM_CLIP_H 224
#define TOTAL_COUNT 15  //ゴール条件
// BMX055 加速度センサのI2Cアドレス
#define Addr_Accl 0x19  // (JP1,JP2,JP3 = Openの時)
// BMX055 ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x69  // (JP1,JP2,JP3 = Openの時)
// BMX055 磁気センサのI2Cアドレス
#define Addr_Mag 0x13  // (JP1,JP2,JP3 = Openの時)

Adafruit_BME680 bme;
Servo servo;

DNNRT dnnrt;  //カメラによる推論のオブジェクト
DNNVariable input(DNN_IMG_W *DNN_IMG_H *DNN_IMG_C);

static uint8_t const label[2] = { 0, 1 };  //カラーコーンかそれ以外かのラベル
static SpGnss Gnss;
int a = 0;

const int IN1 = 3;
const int IN2 = 4;
const int IN3 = 7;
const int IN4 = 9;

int count_clock = 0;
int count_up = 0;
int count_ground = 0;

float bme_sum = 0;
float bme_ground = 0;
// センサーの値を保存するグローバル変数
float xAcc1 = 0.00;
float yAcc1 = 0.00;
float zAcc1 = 0.00;
float xAcc2 = 0.00;
float yAcc2 = 0.00;
float zAcc2 = 0.00;
float xAcc3 = 0.00;
float yAcc3 = 0.00;
float zAcc3 = 0.00;
int stack_clock = 0;  //スタックしているかどうかの確認用
float ac_sum = 0;     //各軸の加速度の絶対値の合計
int reco_count = 0;   //赤コーンの認識回数
int stack_jadge = 0;  // スタック判定用
int goal_count = 0;   //予備のゴール条件
clock_t start_time;   //開始時のクロックの時間
clock_t end_time;
double longitudeC;                                                         //degree
double latitudeC;                                                          //degree
double altitudeC;                                                          //終了時のクロックの時間
double elapsed_time = ((double)(end_time - start_time)) / CLOCKS_PER_SEC;  //経過時間
void Led_Advanced();
void Led_Stopped();
void Led_Turned();
void led_red();
void led_blue();
void BMX055_Init();
static void print_pos(SpNavData *pNavData);
LLA InitLLA(double Lo, double La, double Al);
Vec3 Calc(LLA lla);
Vec3 CalcDirecVec(Vec3 vec_0, Vec3 vec_1);
double CalcScalar(Vec3 vec_s);
double CalcAngle(Vec3 gvec, Vec3 cvec, double gvec_s, double cvec_s);
void advance();
void left();
void stop();
void turn();
void BMX055_Accl();
int distance();
void CamCB();
enum ParamSat {
  eSatGps,            /**< GPS                     World wide coverage  */
  eSatGlonass,        /**< GLONASS                 World wide coverage  */
  eSatGpsSbas,        /**< GPS+SBAS                North America        */
  eSatGpsGlonass,     /**< GPS+Glonass             World wide coverage  */
  eSatGpsBeidou,      /**< GPS+BeiDou              World wide coverage  */
  eSatGpsGalileo,     /**< GPS+Galileo             World wide coverage  */
  eSatGpsQz1c,        /**< GPS+QZSS_L1CA           East Asia & Oceania  */
  eSatGpsGlonassQz1c, /**< GPS+Glonass+QZSS_L1CA   East Asia & Oceania  */
  eSatGpsBeidouQz1c,  /**< GPS+BeiDou+QZSS_L1CA    East Asia & Oceania  */
  eSatGpsGalileoQz1c, /**< GPS+Galileo+QZSS_L1CA   East Asia & Oceania  */
  eSatGpsQz1cQz1S,    /**< GPS+QZSS_L1CA+QZSS_L1S  Japan                */
};
static enum ParamSat satType = eSatGps;

SDClass SD;   //SDクラスのオブジェクト
File myFile;  //ファイルオブジェクト
CamErr err;   //カメラのインスタンス
void setup() {
  Wire.begin();  //Wireの初期化
  BMX055_Init();
  while (xAcc1 == 0) {
    BMX055_Accl();
  }  
  pinMode(ledRed, OUTPUT);
  pinMode(ledBlue, OUTPUT);

  servo.attach(5);

  digitalWrite(ledRed, LOW);
  digitalWrite(ledBlue, LOW);

  int error_flag = 0;

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  /* Set serial baudrate. */
  Serial.begin(115200);

  /* Wait HW initialization done. */
  sleep(3);

  /* Set Debug mode to Info */
  Gnss.setDebugMode(PrintInfo);

  int result;

  //Gnssをアクティブにする。
  result = Gnss.begin();



  //esat tipe
  if (result != 0) {
    Serial.println("Gnss begin error!!");
    error_flag = 1;
  } else {
    switch (satType) {
      case eSatGps:
        Gnss.select(GPS);
        break;

      case eSatGpsSbas:
        Gnss.select(GPS);
        Gnss.select(SBAS);
        break;

      case eSatGlonass:
        Gnss.select(GLONASS);
        Gnss.deselect(GPS);
        break;

      case eSatGpsGlonass:
        Gnss.select(GPS);
        Gnss.select(GLONASS);
        break;

      case eSatGpsBeidou:
        Gnss.select(GPS);
        Gnss.select(BEIDOU);
        break;

      case eSatGpsGalileo:
        Gnss.select(GPS);
        Gnss.select(GALILEO);
        break;

      case eSatGpsQz1c:
        Gnss.select(GPS);
        Gnss.select(QZ_L1CA);
        break;

      case eSatGpsQz1cQz1S:
        Gnss.select(GPS);
        Gnss.select(QZ_L1CA);
        Gnss.select(QZ_L1S);
        break;

      case eSatGpsBeidouQz1c:
        Gnss.select(GPS);
        Gnss.select(BEIDOU);
        Gnss.select(QZ_L1CA);
        break;

      case eSatGpsGalileoQz1c:
        Gnss.select(GPS);
        Gnss.select(GALILEO);
        Gnss.select(QZ_L1CA);
        break;

      case eSatGpsGlonassQz1c:
      default:
        Gnss.select(GPS);
        Gnss.select(GLONASS);
        Gnss.select(QZ_L1CA);
        break;
    }

    /* Start positioning */
    result = Gnss.start(COLD_START);
    if (result != 0) {
      Serial.println("Gnss start error!!");
      error_flag = 1;
    } else {
      Serial.println("Gnss setup OK");
    }
  }

  while (!Serial)
    ;

  Serial.println(F("BME680 pressure version start"));

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1)
      ;
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);  // 320*C for 150 ms
  Serial.print("Insert SD card.");
  while (!SD.begin()) {
    ; /* wait until SD card is mounted. */
  }
  SD.mkdir("dir/");  //sdカード内のディレクトリ
  myFile = SD.open("dir/report.txt", FILE_WRITE);
  if (myFile) {  //ファイルが開けているかどうかの確認
    led_blue();  //開けている場合は青に光る
    delay(3000);
    myFile.println("以下が制御履歴");
    myFile.close();
    Serial.println("ファイル開けています");
  }
  // ニューラルネットワークモデルの読み込み
  File nnbfile = SD.open("model1.1.nnb");
  int ret = dnnrt.begin(nnbfile);
  if (ret < 0) {
    Serial.println("dnnrt.begin 失敗: " + String(ret));
    return;
  }
}

void loop() {
  // myFile = SD.open("dir/report.txt", FILE_WRITE);
  LLA *glla;
  glla = (LLA *)malloc(sizeof(LLA) * NUM1);  //lla型 glla(goal point lla):ゴール地点の経度・緯度・高度
  LLA *clla;
  clla = (LLA *)malloc(sizeof(LLA) * NUM1 + 1);  //lla型  clla(current point lla):観測地点の経度・緯度・高度
  Vec3 *gvec;
  gvec = (Vec3 *)malloc(sizeof(Vec3) * NUM1);  //Vec3型  gvec(goal point vector):ゴール地点の位置ベクトル
  Vec3 *cvec;
  cvec = (Vec3 *)malloc(sizeof(Vec3) * NUM1 + 1);    //Vec3型  cvec(current point lla):観測地点の位置ベクトル
  Vec3 *dvec_g;                                      //ベクトルO1G (O1:前回居た地点、G:ゴール地点)
  dvec_g = (Vec3 *)malloc(sizeof(Vec3) * NUM1 + 1);  //Vec3型  dvec_g(direction vector):方向ベクトル
  double *dvec_gs;
  dvec_gs = (double *)malloc(sizeof(double) * NUM1 + 1);  //double  dvec_gs:方向ベクトルの大きさ

  //goal point LLA, position vec
  glla[0] = InitLLA(139.653616, 35.950078, 21.685603);
  gvec[0] = Calc(glla[0]);

  if (!bme.performReading()) {
    myFile = SD.open("dir/report.txt", FILE_WRITE);
    myFile.println("気圧を読み取ることが出来ません");

    return;
  }
  //地上の気圧bme_groundの測定
  for (int i = 0; i < NUM3; i++) {
    myFile = SD.open("dir/report.txt", FILE_WRITE);
    bme_sum += (float)bme.pressure / 100.0;
    myFile.print("気圧の値 : ");
    myFile.println(bme.pressure / 100.0);
    myFile.close();
  }
  bme_ground = bme_sum / NUM3;
  myFile = SD.open("dir/report.txt", FILE_WRITE);
  myFile.print("地上の気圧 : ");
  myFile.println(bme_ground);
  myFile.close();
  delay(1000);

  //5分経ったら強制的に分離を開始する
  while (count_clock != NUM2) {
    //気圧の差が○○以上になったら上空に居ると判定する(標高が約8m上がると、気圧は1hPa変化する。上空35m付近まで上昇するとすると、4hPaほど変化すると予想)
    myFile = SD.open("dir/report.txt", FILE_WRITE);
    myFile.print("気圧センサの値 :  ");
    myFile.println(bme.pressure / 100.0);
    myFile.close();
    if (fabs((bme.pressure / 100.0) - bme_ground) > 4.0) {
      myFile.println(fabs((bme.pressure / 100.0) - bme_ground));
      myFile = SD.open("dir/report.txt", FILE_WRITE);
      myFile.println("上昇の終了を検知");
      myFile.close();
      count_up++;
    }
    if (bme.pressure > bme_ground - 1.0 && bme.pressure < bme_ground + 1.0 && count_up > 5) {
      myFile = SD.open("dir/report.txt", FILE_WRITE);
      myFile.print(count_up + 1);
      myFile.println("回目の着地判定です");
      myFile.close();
      count_ground++;
    }

    if (count_ground > 10) {
      myFile = SD.open("dir/report.txt", FILE_WRITE);
      myFile.println("着地を検知しました");
      myFile.close();
      break;
    }
    delay(1000);
    count_clock++;
    led_blue();
  }
  //分離機構作動
  Serial.println("servo");
  servo.write(20);
  delay(450);
  servo.write(90);
  delay(500);
  myFile = SD.open("dir/report.txt", FILE_WRITE);
  myFile.println("分離機構作動");
  myFile.close();
  start_time = clock();         //時間計測開始
  while (elapsed_time < 720) {  //分離機構作動12分経ってもゴール判定しない場合は強制的にゴール
    if (Gnss.waitUpdate(-1)) {
      //gnssデータを受信すればa++;によりループを抜け出す。
      while (a == 0) {
        //GPSデータを取得する。
        SpNavData NavData;
        Gnss.getNavData(&NavData);
        //Print position information.
        print_pos(&NavData);
      }
      while (fabs(dvec_gs[0] - dvec_gs[1]) <= 5.0) {  //ゴール5m圏内になるまで
        end_time = clock();
        for (int i = 0; i < NUM1; i++) {
          if ((distance(clla, cvec, dvec_g, dvec_gs, gvec)) == 1) {
            turn();
            stop();
          }
          if (i == 0) {
            myFile = SD.open("dir/report.txt", FILE_WRITE);
            myFile.println("前進します");
            myFile.close();
            advance();
            delay(13000);
            stop();
            delay(500);
          }
          delay(1000);
          //ゴール地点までの距離の計算と場合分け
          if (dvec_gs[0] < dvec_gs[1]) {
            //LED red
            led_red();
            myFile = SD.open("dir/report.txt", FILE_WRITE);
            myFile.println("ゴールから遠ざかっています");
            myFile.close();
            //80度回転
            left();
            delay(1500);
          } else {
            //LED blue
            led_blue();
            myFile = SD.open("dir/report.txt", FILE_WRITE);
            myFile.println("ゴールに近づいています");
            myFile.close();
          }
        }
      }
      if ((distance(clla, cvec, dvec_g, dvec_gs, gvec)) == 1) {
        turn();
        stop();
      }

      //ここからカメラ
      if (reco_count != TOTAL_COUNT) {
        end_time = clock();
        err = theCamera.begin();
        myFile = SD.open("dir/report.txt", FILE_WRITE);
        myFile.println("カメラによる推論開始");
        myFile.close();
        err = theCamera.startStreaming(true, CamCB);
        delay(1000);
      } else if (reco_count == TOTAL_COUNT) {  //ゴール条件
        myFile = SD.open("dir/report.txt", FILE_WRITE);
        myFile.println("ゴールしました");
        theCamera.end();
        myFile.close();
        led_blue();
        exit(1);
      }
    }
  }
  myFile = SD.open("dir/report.txt", FILE_WRITE);
  myFile.println("制御終了");
  myFile.close();
  led_red();
  exit(1);
}
