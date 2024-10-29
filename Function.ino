void Led_1() {
  for (int i = 0; i < 2; i++) {
    ledOn(PIN_LED1);
    delay(500);
    ledOff(PIN_LED1);
    delay(500);
  }
}
void Led_2() {
  for (int i = 0; i < 2; i++) {
    ledOn(PIN_LED2);
    delay(500);
    ledOff(PIN_LED2);
    delay(500);
  }
}
void Led_3() {
  for (int i = 0; i < 2; i++) {
    ledOn(PIN_LED3);
    delay(500);
    ledOff(PIN_LED3);
    delay(500);
  }
}
void led_red() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(ledRed, HIGH);
    delay(500);
    digitalWrite(ledRed, LOW);
    delay(500);
  }
}
void led_blue() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(ledBlue, HIGH);
    delay(500);
    digitalWrite(ledBlue, LOW);
    delay(500);
  }
}
static void print_pos(SpNavData *pNavData) {
  char StringBuffer[STRING_BUFFER_SIZE];

  /* print position data */
  if (pNavData->posFixMode == FixInvalid) {
    Serial.print("訂正なし。 ");
    myFile = SD.open("dir/report.txt", FILE_WRITE);
    myFile.print("訂正なし。 ");
    myFile.close();
  } else {
    Serial.print("GNSSデータを受信, ");
    myFile = SD.open("dir/report.txt", FILE_WRITE);
    myFile.println("GNSSデータを受信");
    myFile.close();
  }

  if (pNavData->posDataExist == 0) {
    myFile = SD.open("dir/report.txt", FILE_WRITE);
    myFile.println("GNSSデータをまだ受信していません");
    Serial.print("GNSSデータをまだ受信していません。");
    myFile.close();
    delay(1500);
  } else {
    a++;
  }

  Serial.println("");
}
LLA InitLLA(double Lo, double La, double Al) {
  LLA lla{};
  lla.longitude = Lo * DEG2RAD;
  lla.latitude = La * DEG2RAD;
  lla.altitude = Al;
  return lla;
}
Vec3 Calc(LLA lla) {

  Vec3 vec{};

  double r = RE + lla.altitude;
  vec.x = r * cos(lla.latitude) * cos(lla.longitude);
  vec.y = r * cos(lla.latitude) * sin(lla.longitude);
  vec.z = r * sin(lla.latitude);
  return vec;
}
Vec3 CalcDirecVec(Vec3 vec_0, Vec3 vec_1) {
  Vec3 vec{};
  vec.x = (vec_1.x - vec_0.x);
  vec.y = (vec_1.y - vec_0.y);
  vec.z = (vec_1.z - vec_0.z);
  return vec;
}
double CalcScalar(Vec3 vec_s) {
  double scal = sqrt((vec_s.x * vec_s.x) + (vec_s.y * vec_s.y) + (vec_s.z * vec_s.z));
  return scal;
}
double CalcAngle(Vec3 gvec, Vec3 cvec, double gvec_s, double cvec_s) {
  double dot_product = ((gvec.x * cvec.x) + (gvec.y * cvec.y) + (gvec.z * cvec.z));  //dot_productは内積
  double theta = acos(dot_product / (gvec_s * cvec_s));                              //acosにより内角を求める。
  return theta;
}
void advance() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void left() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
void turn() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HGIH);
  delay(4000);
}
}

// カメラのコールバック関数
void CamCB(CamImage img) {
  if (!img.isAvailable()) {
    Serial.println("画像が利用できません。もう一度試してください。");
    return;
  }

  CamImage small;
  CamErr err = img.clipAndResizeImageByHW(small, CAM_CLIP_X, CAM_CLIP_Y, CAM_CLIP_X + CAM_CLIP_W - 1, CAM_CLIP_Y + CAM_CLIP_H - 1, DNN_IMG_W, DNN_IMG_H);
  if (!small.isAvailable()) {
    Serial.println("クリップおよびリサイズエラー: " + String(err));
    return;
  }

  small.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);
  uint16_t *tmp = (uint16_t *)small.getImgBuff();

  float *dnnbuf = input.data();
  float f_max = 0.0;
  for (int n = 0; n < DNN_IMG_H * DNN_IMG_W; ++n) {
    dnnbuf[n * 3 + 0] = (float)((tmp[n] & 0xF800) >> 11);  // 赤
    dnnbuf[n * 3 + 1] = (float)((tmp[n] & 0x07E0) >> 5);   // 緑
    dnnbuf[n * 3 + 2] = (float)(tmp[n] & 0x001F);          // 青

    f_max = max(f_max, max(dnnbuf[n * 3 + 0], max(dnnbuf[n * 3 + 1], dnnbuf[n * 3 + 2])));
  }

  /* 正規化 */
  for (int n = 0; n < DNN_IMG_W * DNN_IMG_H * DNN_IMG_C; ++n) {
    dnnbuf[n] /= f_max;
  }

  // 推論実行
  dnnrt.inputVariable(input, 0);
  dnnrt.forward();
  DNNVariable output = dnnrt.outputVariable(0);
  int index = output.maxIndex();
  myFile = SD.open("dir/report.txt",FILE_WRITE);
  Serial.println("推論結果: " + String(label[index]) + " 信頼度: " + String(output[index]));
  myFile.print("推論結果: ");
  myFile.print(label[index]);
  myFile.print(" 信頼度: ");
  myFile.println(output[index]);
  myFile.close();

  // 推論結果に基づいてモーターを制御
  if (label[index] == 1) {
    Serial.println("call takePicture()");
    CamImage img = theCamera.takePicture();

    /* Check availability of the img instance. */
    /* If any errors occur, the img is not available. */

    if (img.isAvailable()) {
      /* Create file name */

      char filename[16] = { 0 };
      sprintf(filename, "PICT%03d.JPG", reco_count);
      Serial.print("Save taken picture as ");
      Serial.print(filename);
      Serial.println("");


      SD.remove(filename);
      File myFile = SD.open(filename, FILE_WRITE);
      myFile.write(img.getImgBuff(), img.getImgSize());
      myFile.close();
    } else {
      Serial.println("Failed to take picture");
    }
    myFile = SD.open("dir/report.txt", FILE_WRITE);
    myFile.println("対象が検知されました");
    myFile.close();
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(3000);
    reco_count++;
  } else if (label[index] == 0) {
    // 左折
    myFile = SD.open("dir/report.txt", FILE_WRITE);
    myFile.println("対象が検知されていません");
    myFile.close();
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    delay(1000);
  }
}

void BMX055_Accl() {
  unsigned int data[6];
  for (int i = 0; i < 6; i++) {
    Wire.beginTransmission(Addr_Accl);
    Wire.write((2 + i));  // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Accl, 1);  // Request 1 byte of data
    // Read 6 bytes of data
    // xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data to 12-bits
  xAcc1 = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
  if (xAcc1 > 2047) xAcc1 -= 4096;
  yAcc1 = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
  if (yAcc1 > 2047) yAcc1 -= 4096;
  zAcc1 = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
  if (zAcc1 > 2047) zAcc1 -= 4096;
  xAcc1 = xAcc1 * 0.00098;  // range = +/-2g
  yAcc1 = yAcc1 * 0.00098;  // range = +/-2g
  zAcc1 = zAcc1 * 0.00098;  // range = +/-2g
}

void stack() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  delay(2000);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(2000);
}
void BMX055_Init() {
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x0F);  // Select PMU_Range register
  Wire.write(0x03);  // Range = +/- 2g
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x10);  // Select PMU_BW register
  Wire.write(0x08);  // Bandwidth = 7.81 Hz
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x11);  // Select PMU_LPW register
  Wire.write(0x00);  // Normal mode, Sleep duration = 0.5ms
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x0F);  // Select Range register
  Wire.write(0x04);  // Full scale = +/- 125 degree/s
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x10);  // Select Bandwidth register
  Wire.write(0x07);  // ODR = 100 Hz
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x11);  // Select LPM1 register
  Wire.write(0x00);  // Normal mode, Sleep duration = 2ms
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x83);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x01);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4C);  // Select Mag register
  Wire.write(0x00);  // Normal Mode, ODR = 10 Hz
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4E);  // Select Mag register
  Wire.write(0x84);  // X, Y, Z-Axis enabled
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x51);  // Select Mag register
  Wire.write(0x04);  // No. of Repetitions for X-Y Axis = 9
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x52);  // Select Mag register
  Wire.write(16);    // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();
}

float total(float x1, float y1, float z1, float x2, float y2, float z2) {
  float abs_x = x1 - x2;
  float abs_y = y1 - y2;
  float abs_z = z1 - z2;
  if (abs_x < 0) {
    abs_x = -(abs_x);
  }
  if (abs_y < 0) {
    abs_y = -(abs_y);
  }
  if (abs_z < 0) {
    abs_z = -(abs_z);
  }
  return abs_x + abs_y;
}

int Stack(double dvec0, double dvec1) {
  if (fabs(dvec0 - dvec1) < 2.0) {
    return 1;
  } else {
    return 0;
  }
}

int distance(LLA *clla, Vec3 *cvec, Vec3 *dvec_g, double *dvec_gs, Vec3 *gvec) {  //ゴール地点までの距離の計算
  cvec = (Vec3 *)malloc(sizeof(Vec3) * NUM1 + 1);
  dvec_g = (Vec3 *)malloc(sizeof(Vec3) * NUM1 + 1);
  gvec = (Vec3 *)malloc(sizeof(Vec3) * NUM1);
  for (int i = 0; i < NUM1; i++) {
    //GPSデータを取得する。
    double longitudeS = 0;  //degree
    double latitudeS = 0;   //degree
    double altitudeS = 0;   //meter
    double longitudeC = 0;  //degree
    double latitudeC = 0;   //degree
    double altitudeC = 0;   //meter

    for (int i = 0; i < 10; i++) {
      SpNavData NavData;
      Gnss.getNavData(&NavData);
      //CLLA(current pointでのLLA)を取得。
      longitudeC = NavData.longitude;  //degree
      latitudeC = NavData.latitude;    //degree
      altitudeC = NavData.altitude;    //meter
      longitudeS += longitudeC;
      latitudeS += latitudeC;
      altitudeS += altitudeC;
    }
    longitudeC = longitudeS / 10;
    latitudeC = latitudeS / 10;
    altitudeC = altitudeS / 10;
    myFile = SD.open("dir/report.txt", FILE_WRITE);
    myFile.print("測定結果 : 経度　");
    myFile.print(longitudeC);
    myFile.print(" 緯度 ");
    myFile.print(latitudeC);
    myFile.print(" 高度 ");
    myFile.println(altitudeC);
    //clla,cvec,dvec_g(方向ベクトル)の計算
    clla[i] = InitLLA(longitudeC, latitudeC, altitudeC);
    cvec[i] = Calc(clla[i]);
    dvec_g[i] = CalcDirecVec(cvec[i], gvec[0]);
    //ゴール地点までの距離の計算
    dvec_gs[i] = CalcScalar(dvec_g[i]);
    myFile.print("ゴール地点までの距離");
    myFile.println(dvec_gs[i]);
    myFile.close();
  }
  delay(1000);

  //スタック判定
  if (Stack(dvec_gs[0], dvec_gs[1]) == 1) {
    return 1;
  }else{
    return 0
  }
}
