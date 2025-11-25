#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>
#include <Firebase_ESP_Client.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <time.h>
#include <LittleFS.h>

// Cau hinh WiFi va Firebase
#define WIFI_SSID "Wifi 2.4G"
#define WIFI_PASS "66668888"
#define FIREBASE_HOST "https://bp-project-c19cb-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "5yS6ykFg1XbTePsCa3y5zL9X6lRGCXOWY2CuAsmf"

// Ten file luu tru offline
const char* OFFLINE_FILE = "/bp_offline.txt";

// Khai bao Firebase
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Cau hinh NTP
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 3600, 60000); // GMT+7

// Khoi tao OLED
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

/*
Cau hinh chan GPIO
*/
#define DOUT_PIN 12
#define SCLK_PIN 14
#define PUMP_PIN 13
#define VALVE_PIN 15
#define BUTTON1_PIN 0
#define BUTTON2_PIN 2

// Hang so HX710B
const long   HX_OFFSET = 563780;
const float  HX_SCALE_MMHG = 0.000024;
const float  MAX_PRESSURE = 180.0;
const float  MIN_PRESSURE = 30.0;

// Hang so bom va xa
#define PUMP_DELAY 80
#define DEFLATE_DELAY 60

// Cac bien toan cuc
unsigned long lastUpdate = 0;
bool lastBtn1 = HIGH, lastBtn2 = HIGH;
bool emergencyStop = false;

float pressureData[500];
float filteredPressure[500];
float oscillation[500];
int bpSampleCount = 0;

float lastSystolic = 0, lastDiastolic = 0;

enum systemMode
{
  IDLE,
  MEASURING_BP
};
systemMode currentMode = IDLE;

// Bo loc Kalman Filter
float kalman_x = 0;
float kalman_p = 100;
float kalman_q = 0.1;
float kalman_r = 1;
float kalman_k = 0;

// Chuoi dong bo
String syncCode = "";

// Ham chuong trinh chinh
void connectWiFi();
long readHX710B();
float kalmanFillter(float measurement);
float movingAverage(float *data, int idx, int window);
void adjustKalmanParameters(float variance);
float calculateVariance(float *data, int size);
bool analyzeBPData(float& systolic, float& diastolic);
const char* classifyBloodPressure(float systolic, float diastolic);
void displayBPWarning(float systolic, float diastolic);
void checkEmergency();
void updateDisplay();
void measureBloodPressure();
void initNTP();
unsigned long getUnixTime();
String getFormattedDateTime();
void initFirebase();
void uploadBPToFirebase(float systolic, float diastolic);
void initLittleFS();
void saveOfflineData(String jsonData);
void uploadOfflineData();

// Ham kiem tra ket noi WiFi
int wifiRetries = 3;
void connectWiFi()
{
  for (int attempt = 1; attempt <= wifiRetries; attempt++)
  {
    Serial.printf("Dang ket noi WiFi (lan %d/%d)...\n", attempt, wifiRetries);

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    u8g2.clearBuffer();
    u8g2.drawStr(10, 24, "Dang ket noi WiFi...");
    u8g2.sendBuffer();

    int waitCount = 0;
    const int maxWait = 20;

    while (WiFi.status() != WL_CONNECTED && waitCount < maxWait)
    {
      delay(500);
      Serial.print(".");
      waitCount++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("\nDa ket noi WiFi!");
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());

      u8g2.clearBuffer();
      u8g2.drawStr(10, 24, "WiFi ket noi OK!");
      u8g2.drawStr(10, 44, WiFi.localIP().toString().c_str());
      u8g2.sendBuffer();
      delay(1500);
      return;
    }

    Serial.println("\nKet noi that bai. Thu lai...");
    u8g2.clearBuffer();
    u8g2.drawStr(10, 32, "Ket noi that bai!");
    u8g2.sendBuffer();
    delay(1500);
  }

  Serial.println("Khong the ket noi WiFi.");
  u8g2.clearBuffer();
  u8g2.drawStr(5, 32, "WiFi khong ket noi!");
  u8g2.sendBuffer();
  delay(2000);
}

// Ham doc cam bien HX710B
long readHX710B()
{
  long raw = 0;
  while(digitalRead(DOUT_PIN) == HIGH)
  {
    delayMicroseconds(10);
  }

  for(int i = 0; i < 24; i++)
  {
    digitalWrite(SCLK_PIN, HIGH);
    delayMicroseconds(1);
    raw <<= 1;
    if(digitalRead(DOUT_PIN)) raw |= 1;
    digitalWrite(SCLK_PIN, LOW);
    delayMicroseconds(1);
  }
  digitalWrite(SCLK_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(SCLK_PIN, LOW);
  delayMicroseconds(1);
  if(raw & 0x800000) raw |= ~0xFFFFFF;
  return raw;
}

// Ham bo loc Kalman
float kalmanFillter(float measurement)
{
  kalman_p = kalman_p + kalman_q;
  kalman_k = kalman_p / (kalman_p + kalman_r);
  kalman_x = kalman_x + kalman_k * (measurement - kalman_x);
  kalman_p = (1 - kalman_k) * kalman_p;
  return kalman_x;
}

// Bo loc trung binh dong
float movingAverage(float *data, int idx, int window)
{
  float sum = 0;
  int cnt = 0;
  for(int i = max(0, idx - window + 1); i <= idx; i++)
  {
    sum += data[i];
    cnt++;
  }
  return sum / cnt;
}

// Ham dieu chinh tham so Kalman
void adjustKalmanParameters(float variance)
{
  kalman_q = variance * 0.01;
  kalman_r = variance * 0.1;
}

// Ham tinh phuong sai cua du lieu
float calculateVariance(float *data, int size)
{
  float mean = 0, sumSquaredDiff = 0;
  for(int i = 0; i < size; i++)
  {
    mean += data[i];
  }
  mean /= size;

  for(int i = 0; i < size; i++)
  {
    float diff = data[i] - mean;
    sumSquaredDiff += diff * diff;
  }
  return sumSquaredDiff / size;
}

// Ham phan tich du lieu huyet ap
bool analyzeBPData(float& systolic, float& diastolic)
{
  systolic = 0;
  diastolic = 0;

  if(bpSampleCount < 10)
  {
    Serial.println("Qua it du lieu de phan tich");
    return false;
  }
  for(int i = 1; i < bpSampleCount; i++)
  {
    oscillation[i] = fabs(pressureData[i] - filteredPressure[i]);
  }
  oscillation[0] = 0;
  float maxOsc = 0;
  int idxMax = 0;
  for(int i = 0; i < bpSampleCount; i++)
  {
    if(oscillation[i] > maxOsc)
    {
      maxOsc = oscillation[i];
      idxMax = i;
    }
  }

  Serial.printf("Dao dong toi da: %.3f tai chi so %d (ap suat %.1f mmHg)\n", maxOsc, idxMax, pressureData[idxMax]);
  if(emergencyStop && idxMax < 5)
  {
    float maxPressure = 0, minPressure = 300;
    for(int i = 0; i < bpSampleCount; i++)
    {
      if(pressureData[i] > maxPressure) maxPressure = pressureData[i];
      if(pressureData[i] < minPressure && pressureData[i] > 30) minPressure = pressureData[i];
    }
    systolic = maxPressure * 0.85;
    diastolic = minPressure * 1.3;
    Serial.println("Su dung phuong phap du phong do dung khan cap!");
    Serial.printf("Ap suat max/min: %.1f/%.1f -> SYS/DIA: %.1f/%.1f\n", maxPressure, minPressure, systolic, diastolic);
  }
  else
  {
    for(int i = idxMax; i > 0; i--)
    {
      if(oscillation[i] < 0.55 * maxOsc)
      {
        systolic = pressureData[i];
        Serial.printf("Tam thu: Chi so %d, dao dong %.3f, ap suat %.1f mmHg\n", i, oscillation[i], systolic);
        break;
      }
    }
    for(int i = idxMax; i < bpSampleCount; i++)
    {
      if(oscillation[i] < 0.65 * maxOsc)
      {
        diastolic = pressureData[i];
        Serial.printf("Tam truong: Chi so %d, dao dong %.3f, ap suat %.1f mmHg\n", i, oscillation[i], diastolic);
        break;
      }
    }
  }
  systolic = floor(systolic);
  diastolic = floor(diastolic);
  if(systolic < 70 || systolic > 200 || diastolic < 40 || diastolic > 120 || diastolic > systolic)
  {
    Serial.println("Ket qua khong hop le!");
    return false;
  }
  return true;
}

// Ham phan loai huyet ap
const char* classifyBloodPressure(float systolic, float diastolic) {
  if (systolic >= 140 || diastolic >= 95) return "Huyet ap cao";
  else if (systolic >= 130 || diastolic >= 89) return "Hoi cao";
  else if (systolic >= 105 || diastolic >= 75) return "Binh thuong";
  else if (systolic >= 90 && diastolic >= 60) return "Toi uu";
  else if (systolic < 90 || diastolic < 60) return "Huyet ap thap";
  else return "Khong xac dinh";
}

// Ham canh bao huyet ap
void displayBPWarning(float systolic, float diastolic) {
  const char* status = classifyBloodPressure(systolic, diastolic);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB10_tr);

  if (systolic >= 160 || diastolic >= 100) {
    u8g2.drawStr(2, 20, "CANH BAO");
    u8g2.setFont(u8g2_font_ncenB08_tr);
    char msg1[32];
    sprintf(msg1, "HA: %.0f/%.0f mmHg", systolic, diastolic);
    u8g2.drawStr(2, 36, msg1);
    u8g2.drawStr(2, 48, "Nguy co cao - lien he bac si");
    u8g2.drawStr(2, 58, "hoac di kham som");
  }
  else if (systolic >= 140 || diastolic >= 95) {
    u8g2.drawStr(2, 18, "HA CAO");
    u8g2.setFont(u8g2_font_ncenB08_tr);
    char msg2[32];
    sprintf(msg2, "HA: %.0f/%.0f mmHg", systolic, diastolic);
    u8g2.drawStr(2, 34, msg2);
    u8g2.drawStr(2, 46, "Theo doi - khuyen nghi tu van");
    u8g2.drawStr(2, 58, "bac si neu keo dai");
  }
  else if (systolic >= 130 || diastolic >= 89) {
    u8g2.drawStr(2, 22, "HA HOI CAO");
    u8g2.setFont(u8g2_font_ncenB08_tr);
    char msg3[32];
    sprintf(msg3, "HA: %.0f/%.0f mmHg", systolic, diastolic);
    u8g2.drawStr(2, 38, msg3);
    u8g2.drawStr(2, 50, "Theo doi lai sau");
  }
  else if ((systolic >= 105 && systolic < 130) || (diastolic >= 75 && diastolic < 89)) {
    u8g2.drawStr(2, 26, "HA BT");
    u8g2.setFont(u8g2_font_ncenB08_tr);
    char msg4[32];
    sprintf(msg4, "HA: %.0f/%.0f mmHg", systolic, diastolic);
    u8g2.drawStr(2, 42, msg4);
    u8g2.drawStr(2, 54, "Giu loi song lanh manh");
  }
  // Low blood pressure: warning
  else if (systolic < 90 || diastolic < 60) {
    u8g2.drawStr(2, 18, "HA THAP");
    u8g2.setFont(u8g2_font_ncenB08_tr);
    char msg5[32];
    sprintf(msg5, "HA: %.0f/%.0f mmHg", systolic, diastolic);
    u8g2.drawStr(2, 34, msg5);
    u8g2.drawStr(2, 46, "Can cham soc suc khoe");
  }
  else {
    // fallback
    u8g2.drawStr(2, 26, status);
    u8g2.setFont(u8g2_font_ncenB08_tr);
    char msg6[32];
    sprintf(msg6, "HA: %.0f/%.0f mmHg", systolic, diastolic);
    u8g2.drawStr(2, 44, msg6);
  }

  u8g2.sendBuffer();
  delay(4000);
}

// Ham kiem tra dung khan cap
void checkEmergency()
{
  if(digitalRead(BUTTON2_PIN) == LOW && currentMode == MEASURING_BP)
  {
    emergencyStop = true;
    digitalWrite(PUMP_PIN, LOW);
    digitalWrite(VALVE_PIN, LOW);
    Serial.println(">> Khan cap: dung bom, mo van.");
  }
}

// Ham cap nhat man hinh OLED
void updateDisplay() 
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  if (currentMode == IDLE) 
  {
    u8g2.drawStr(5, 10, "San sang do...");
    if (lastSystolic > 0) 
    {
      char bpStr[20];
      sprintf(bpStr, "HA: %.0f/%.0f", lastSystolic, lastDiastolic);
      u8g2.drawStr(5, 40, bpStr);
    }
  } 
  else if (currentMode == MEASURING_BP) 
  {
    u8g2.drawStr(5, 20, "Do huyet ap...");
    if (bpSampleCount > 0 && pressureData[bpSampleCount-1] > 0) 
    {
      char pressureStr[20];
      sprintf(pressureStr, "Ap: %.0f mmHg", floor(pressureData[bpSampleCount-1]));
      u8g2.drawStr(5, 30, pressureStr);
    }
  }
  u8g2.sendBuffer();
}

// Ham do huyet ap
void measureBloodPressure()
{
  emergencyStop = false;
  bpSampleCount = 0;
  kalman_x = 0;
  kalman_p = 100;
  currentMode = MEASURING_BP;

  updateDisplay();

  digitalWrite(PUMP_PIN, HIGH);
  digitalWrite(VALVE_PIN, HIGH);

  float pressureMmHg = 0, rawPressure = 0;
  while(pressureMmHg < MAX_PRESSURE && !emergencyStop) 
  {
    long raw = readHX710B();
    rawPressure = (raw - HX_OFFSET) * HX_SCALE_MMHG;
    pressureMmHg = kalmanFillter(rawPressure);
    Serial.printf("Bom: %.1f mmHg (raw: %.1f)\n", pressureMmHg, rawPressure);
    u8g2.clearBuffer();
    u8g2.drawStr(5, 20, "Do huyet ap...");
    char pumStr[20];
    sprintf(pumStr, "Ap: %0.f mmHg", floor(pressureMmHg));
    u8g2.drawStr(5, 30, pumStr);
    u8g2.sendBuffer();
    checkEmergency();
    delay(PUMP_DELAY);
  }

  digitalWrite(PUMP_PIN, LOW);

  if(!emergencyStop)
  {
    Serial.println("Dat ap suat toi da, bat dau xa...");
    digitalWrite(VALVE_PIN, LOW);
    u8g2.clearBuffer();
    u8g2.drawStr(5, 20, "Do huyet ap...");
    char deflateStr[20];
    sprintf(deflateStr, "Ap: %.0f mmHg", floor(pressureMmHg));
    u8g2.drawStr(5, 30, deflateStr);
    u8g2.sendBuffer();
    delay(2000);
  }

  float tempData[10];
  int tempCount = 0;
  for(int i = 0; i < 500 && !emergencyStop; i++)
  {
    long raw = readHX710B();
    rawPressure = (raw - HX_OFFSET) * HX_SCALE_MMHG;
    pressureMmHg = kalmanFillter(rawPressure);
    pressureData[i] = pressureMmHg;
    filteredPressure[i] = (i >= 5) ? movingAverage(pressureData, i, 5) : pressureMmHg;
    if(tempCount < 10) tempData[tempCount++] = pressureMmHg;
    if(i % 10 == 0)
    {
      u8g2.clearBuffer();
      u8g2.drawStr(5, 20, "Do huyet ap...");
      char deflateStr[20];
      sprintf(deflateStr, "Ap: %.0f mmHg", floor(pressureMmHg));
      u8g2.drawStr(5, 30, deflateStr);
      u8g2.sendBuffer();
    }

    if(pressureMmHg < MIN_PRESSURE)
    {
      bpSampleCount = i + 1;
      break;
    }
    bpSampleCount++;
    checkEmergency();
    delay(DEFLATE_DELAY);
  }

  digitalWrite(VALVE_PIN, LOW);
  if(tempCount >= 10) adjustKalmanParameters(calculateVariance(tempData, tempCount));
  u8g2.clearBuffer();
  u8g2.drawStr(5, 20, "Tinh toan...");
  u8g2.sendBuffer();
  float systolic = 0, diastolic = 0;
  bool validResult = analyzeBPData(systolic, diastolic);
  if(emergencyStop)
  {
    Serial.println("Do huyet ap dung khan cap, co gang phan tich du lieu da thu thap...");
  }
  if(!validResult)
  {
    u8g2.clearBuffer();
    u8g2.drawStr(5, 20, "Loi phan tich");
    u8g2.sendBuffer();
    delay(3000);
    currentMode = IDLE;
    updateDisplay();
    return;
  }
  lastSystolic = floor(systolic); 
  lastDiastolic = floor(diastolic); 
  Serial.printf("Ket qua HA → TT: %.0f, TT: %.0f mmHg\n", lastSystolic, lastDiastolic);
  u8g2.clearBuffer();
  char bpResultStr[20];
  sprintf(bpResultStr, "HA: %.0f/%.0f", lastSystolic, lastDiastolic);
  u8g2.drawStr(5, 20, bpResultStr);
  // const char* bpStatus = classifyBloodPressure(lastSystolic, lastDiastolic);
  // u8g2.drawStr(5, 30, bpStatus);
  u8g2.sendBuffer();
  // Hien thi canh bao tren OLED
  displayBPWarning(lastSystolic, lastDiastolic);
  // Tai len du lieu huyet ap len Firebase
  uploadBPToFirebase(lastSystolic, lastDiastolic);
  currentMode = IDLE;
  updateDisplay();
}

// Ham khoi dong NTP
void initNTP()
{
  Serial.println("Khoi dong NTP...");
  timeClient.begin();

  int retryCount = 0;
  while(!timeClient.update() && retryCount < 10)
  {
    Serial.print(".");
    timeClient.forceUpdate();
    delay(500);
    retryCount++;
  }

  if(timeClient.update())
  {
    Serial.println("\nNTP khoi dong thanh cong!");
    Serial.print("Thoi gian hien tai: ");
    Serial.println(timeClient.getFormattedTime());
  }
  else
  {
    Serial.println("\nNTP khoi dong that bai!");
  }
}

// Ham lay thoi gian Unix
unsigned long getUnixTime()
{
  if(!timeClient.update())
  {
    timeClient.forceUpdate();
  }
  return timeClient.getEpochTime();
}

// Ham lay thoi gian dang ISO 8601
String getFormattedDateTime()
{
  unsigned long epoch = getUnixTime();
  time_t t = (time_t)epoch;
  struct tm *tm = gmtime(&t);
  char buf[32];

  snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d",
           tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
           tm->tm_hour, tm->tm_min, tm->tm_sec);
  return String(buf);
}

// Ham khoi dong Firebase
void initFirebase()
{
  Serial.println("Khoi dong Firebase...");
  config.database_url = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Serial.println("Firebase khoi dong thanh cong!");
}

// Ham tai len du lieu huyet ap len Firebase
void uploadBPToFirebase(float systolic, float diastolic)
{
  if(WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Khong the tai len Firebase: khong ket noi WiFi");
    return;
  }

  unsigned long epoch = getUnixTime();
  String timestampISO = getFormattedDateTime();
  String safeKey = timestampISO;
  safeKey.replace(":", "-");
  String path = "/blood_pressure/" + safeKey;
  
  const char* bpStatus = classifyBloodPressure(systolic, diastolic);
  FirebaseJson json;
  json.set("systolic", systolic);
  json.set("diastolic", diastolic);
  json.set("status", bpStatus);
  json.set("epoch", epoch);
  json.set("timestamp", timestampISO);
  
  String jsonStr;
  json.toString(jsonStr);

  if(WiFi.status() == WL_CONNECTED && Firebase.ready())
  {
    Serial.printf("Dang tai du lieu len Firebase: %s\n", path.c_str());
    u8g2.clearBuffer();
    u8g2.drawStr(5, 24, "Dang tai len FB...");
    u8g2.sendBuffer();

    if(Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json))
    {
      Serial.println("Thanh cong!");
      u8g2.clearBuffer(); u8g2.drawStr(5, 24, "Tai len FB OK!"); u8g2.sendBuffer();
    }
    else
    {
      Serial.printf("FB Loi: %s. Luu offline...\n", fbdo.errorReason().c_str());
      saveOfflineData(jsonStr);
    }
  }
  else
  {
    Serial.println("Khong co mang. Luu offline...");
    saveOfflineData(jsonStr);
  }

  delay(1200);
}

// Ham khoi dong LittleFS
void initLittleFS()
{
  if (!LittleFS.begin()) 
  {
    Serial.println("Khoi dong LittleFS that bai!");
  } 
  else 
  {
    Serial.println("LittleFS da san sang su dung.");
  }
}

// Ham luu du lieu offline
void saveOfflineData(String jsonData)
{
  File file = LittleFS.open(OFFLINE_FILE, "a");
  if (!file) 
  {
    Serial.println("Khong the mo file de luu du lieu offline.");
    return;
  }
  file.println(jsonData);
  file.close();
  Serial.println("Du lieu offline da duoc luu.");

  u8g2.clearBuffer();
  u8g2.drawStr(5, 24, "Luu offline OK!");
  u8g2.sendBuffer();
  delay(1000);
}

// Ham tai du lieu offline len Firebase
void uploadOfflineData()
{
  if(WiFi.status() != WL_CONNECTED || !Firebase.ready())
  {
    Serial.println("Khong the tai du lieu offline: khong ket noi WiFi");
    return;
  }

  if(!LittleFS.exists(OFFLINE_FILE)) 
  {
    Serial.println("Khong co du lieu offline de tai len.");
    return;
  }

  File file = LittleFS.open(OFFLINE_FILE, "r");
  if (!file) 
  {
    Serial.println("Khong the mo file du lieu offline.");
    return;
  }

  if(file.size() == 0) 
  {
    Serial.println("File du lieu offline rong.");
    file.close();
    LittleFS.remove(OFFLINE_FILE);
    return;
  }

  Serial.println("Dang tai du lieu offline len Firebase...");
  u8g2.clearBuffer();
  u8g2.drawStr(5, 24, "Tai offline len FB...");
  u8g2.sendBuffer();

  String pendingData = "";
  bool allSuccess = true;

  while(file.available()) 
  {
    String line = file.readStringUntil('\n');
    line.trim();
    if(line.length() == 0) continue;

    FirebaseJson js;
    FirebaseJsonData result;
    js.setJsonData(line);

    js.get(result, "timestamp");
    String timestampISO = result.stringValue;
    String safeKey = timestampISO;
    safeKey.replace(":", "-");
    String path = "/blood_pressure/" + safeKey;

    if(Firebase.RTDB.setJSON(&fbdo, path.c_str(), &js)) 
    {
      Serial.printf("Da tai len: %s\n", path.c_str());
    } 
    else 
    {
      Serial.printf("Tai len that bai: %s - %s\n", path.c_str(), fbdo.errorReason().c_str());
      allSuccess = false;
      pendingData += line + "\n";
    }
    delay(100);
  }

  file.close();

  if(allSuccess) 
  {
    LittleFS.remove(OFFLINE_FILE);
    Serial.println("Tat ca du lieu offline da duoc tai len thanh cong.");
    u8g2.clearBuffer();
    u8g2.drawStr(5, 24, "Tai offline OK!");
    u8g2.sendBuffer();
  } 
  else 
  {
    file = LittleFS.open(OFFLINE_FILE, "w");
    if (file) 
    {
      file.print(pendingData);
      file.close();
      Serial.println("Du lieu offline con lai da duoc luu lai.");
    } 
    else 
    {
      Serial.println("Khong the mo file de luu du lieu offline con lai.");
    }
    u8g2.clearBuffer();
    u8g2.drawStr(5, 24, "Tai offline mot phan!");
    u8g2.sendBuffer();
  }
  delay(1000);
  updateDisplay();
}

// Ham chuong trinh chinh
void setup()
{
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(VALVE_PIN, LOW);
  delay(100);

  Serial.begin(115200);
  Wire.begin();
  u8g2.begin();
  initLittleFS();

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(5, 24, "Dang khoi dong...");
  u8g2.sendBuffer();
  delay(1000);

  pinMode(DOUT_PIN, INPUT);
  pinMode(SCLK_PIN, OUTPUT);
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);

  connectWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    initNTP();
    initFirebase();
  }

  Serial.println("Thiet bi da khoi tao xong");
  updateDisplay();
}

unsigned long lastSyncCheck = 0;
const unsigned long syncInterval = 60000; // Kiem tra dong bo moi 60 giay

// Vong lap chinh
void loop()
{
  bool btn1 = digitalRead(BUTTON1_PIN);
  bool btn2 = digitalRead(BUTTON2_PIN);
  
  // Kiem tra nut nhan de bat dau do huyet ap
  if(btn1 == LOW && lastBtn1 == HIGH && currentMode == IDLE)
  {
    Serial.println("Nut 1 da nhan, bat dau do huyet ap...");
    delay(500);
    measureBloodPressure();
  }

  // Kiem tra nut nhan de dong bo lai WiFi
  if(btn1 == LOW && btn2 == LOW && currentMode == IDLE)
  {
    delay(2000);
    if (digitalRead(BUTTON1_PIN) == LOW && digitalRead(BUTTON2_PIN) == LOW) 
    {
      Serial.println("Ca hai nut duoc giu, dang thu ket noi lai...");
      u8g2.clearBuffer();
      u8g2.drawStr(5, 20, "Ket noi lai...");
      u8g2.sendBuffer();
      WiFi.disconnect();
      delay(1000);
      connectWiFi();

      updateDisplay();
    }
  }

  // Kiem tra nut nhan de huy do huyet ap
  static unsigned long btn1HoldStart = 0;
  if (btn1 == LOW && currentMode == MEASURING_BP) 
  {
    if (btn1HoldStart == 0) btn1HoldStart = millis();
    else if (millis() - btn1HoldStart >= 1000) 
    {
      emergencyStop = true;
      digitalWrite(PUMP_PIN, LOW);
      digitalWrite(VALVE_PIN, LOW);
      Serial.println("Nut 1 duoc giu, huy do huyet ap...");
      currentMode = IDLE;
      updateDisplay();
      btn1HoldStart = 0;
    }
  } 
  else 
  {
    btn1HoldStart = 0;
  }

  // Cap nhat trang thai nut
  lastBtn1 = btn1;
  lastBtn2 = btn2;
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 1000) 
  {
    lastDisplayUpdate = millis();
    updateDisplay();
  }

  if(currentMode == IDLE && WiFi.status() == WL_CONNECTED && Firebase.ready())
  {
    if(millis() - lastSyncCheck > syncInterval)
    {
      lastSyncCheck = millis();
      uploadOfflineData();
    }
  }

  delay(100);
}