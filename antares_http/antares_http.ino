#include "DFRobot_ESP_PH.h"
#include "EEPROM.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <AntaresESPHTTP.h>

#define ACCESSKEY "983825e56c487fe6:c4d80cf91d63ec37"  // Replace with your Antares account access key
#define WIFISSID "PUTRA 1"                             // Wi-Fi SSID to connect to
#define PASSWORD "CINTAIUSUSMU"                        // Wi-Fi password

#define projectName "Monitoring_PHdanKekeruhan"  // Replace with the Antares application name that was created
#define deviceName "Monitor1"                    // Replace with the Antares device name that was created

AntaresESPHTTP antares(ACCESSKEY);

float minr[16];
float Rule[16];

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

DFRobot_ESP_PH ph;
#define ESPADC 4096.0    //the esp Analog Digital Convertion value
#define ESPVOLTAGE 3300  //the esp voltage supply value
#define PH_PIN 35        //the esp gpio data pin number
// #define TB_PIN 34        //the esp gpio data pin number
float kekeruhan, A, B, voltage, phValue, temperature = 25;

unsigned long previousMillis = 0;  // Untuk menyimpan waktu terakhir LCD diperbarui
const long interval = 1000;        // Interval waktu dalam milidetik (1 detik)

// Lebar OLED display dalam satuan pixels
#define SCREEN_WIDTH 128
// tinggi OLED display dalam satuan pixels
#define SCREEN_HEIGHT 64

// Deklarasi untuk display SSD1306 yang terhubung menggunakan I2C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//======== Fungsi keanggotaan kekeruhan =========================================

float fuKJernih() {
  if (kekeruhan >= 5) return 0;
  else if (4 < kekeruhan && kekeruhan < 5) return (5 - kekeruhan) / (5 - 4);
  else if (kekeruhan <= 4) return 1;
  return 0;
}

float fuKKeruh() {
  if (kekeruhan <= 4 || kekeruhan >= 25) return 0;
  else if (4 < kekeruhan && kekeruhan < 5) return (kekeruhan - 4) / (5 - 4);
  else if (5 <= kekeruhan && kekeruhan <= 24) return 1;
  else if (24 < kekeruhan && kekeruhan < 25) return (25 - kekeruhan) / (25 - 24);
  return 0;
}

float fuKSangatKeruh() {
  if (kekeruhan <= 24) return 0;
  else if (24 < kekeruhan && kekeruhan < 25) return (kekeruhan - 24) / (25 - 24);
  else if (kekeruhan >= 25) return 1;
  return 0;
}


//======== Fungsi keanggotaan pH =========================================

float fuPSangatAsam() {
  if (phValue >= 5.5) return 0;
  else if (5 < phValue && phValue < 5.5) return (5.5 - phValue) / (5.5 - 5);
  else if (phValue <= 5) return 1;
  return 0;
}

float fuPAsam() {
  if (phValue <= 5 || phValue >= 6.5) return 0;
  else if (5 < phValue && phValue < 5.5) return (phValue - 5) / (5.5 - 5);
  else if (5.5 <= phValue && phValue <= 6) return 1;
  else if (6 < phValue && phValue < 6.5) return (6.5 - phValue) / (6.5 - 6);
  return 0;
}

float fuPNormal() {
  if (phValue <= 6 || phValue >= 8) return 0;
  else if (6 < phValue && phValue < 7) return (phValue - 6) / (7 - 6);
  else if (phValue == 7) return 1;
  else if (7 < phValue && phValue < 8) return (8 - phValue) / (8 - 7);
  return 0;
}

float fuPBasa() {
  if (phValue <= 7.5 || phValue >= 9) return 0;
  else if (7.5 < phValue && phValue < 8) return (phValue - 7.5) / (8 - 7.5);
  else if (8 <= phValue && phValue <= 8.5) return 1;
  else if (8.5 < phValue && phValue < 9) return (9 - phValue) / (9 - 8.5);
  return 0;
}

float fuPSangatBasa() {
  if (phValue <= 8.5) return 0;
  else if (8.5 < phValue && phValue < 9) return (phValue - 8.5) / (9 - 8.5);
  else if (phValue >= 9) return 1;
  return 0;
}

//menentukan nilai α-predikat dan nilai z menggunakan fungsi Minimum
float Min(float a, float b) {
  if (a < b) {
    return a;
  } else if (b < a) {
    return b;
  } else {
    return a;
  }
}

// Menentukan rules
void rule() {
  //======== nilai keanggotaan OUTPUT ============
  float SBU = 0.2;  // SBU = Sangat Buruk
  float BU = 0.4;   // BU  = Buruk
  float BA = 0.6;   // BA  = Baik
  float SBA = 0.8;  // SBA = Sangat Baik

  // R1 If pH Normal and kekeruhan Jernih then kualitas Sangat_Baik
  minr[1] = Min(fuPNormal(), fuKJernih());
  Rule[1] = SBA;
  // R2 If pH Normal and kekeruhan Keruh then kualitas Baik
  minr[2] = Min(fuPNormal(), fuKKeruh());
  Rule[2] = BA;
  // R3 If pH Normal and kekeruhan Sangat Keruh then kualitas Sangat Buruk
  minr[3] = Min(fuPNormal(), fuKSangatKeruh());
  Rule[3] = SBU;
  // R4 If pH Asam and kekeruhan Jernih then kualitas Baik
  minr[4] = Min(fuPAsam(), fuKJernih());
  Rule[4] = BA;
  // R5 If pH Asam and kekeruhan Keruh then kualitas Buruk
  minr[5] = Min(fuPAsam(), fuKKeruh());
  Rule[5] = BU;
  // R6 If pH Asam and kekeruhan Sangat Keruh then kualitas Sangat Buruk
  minr[6] = Min(fuPNormal(), fuKSangatKeruh());
  Rule[6] = SBU;
  // R7 If pH Basa and kekeruhan Jernih then kualitas Baik
  minr[7] = Min(fuPBasa(), fuKJernih());
  Rule[7] = BA;
  // R8 If pH Basa and kekeruhan Keruh then kualitas Buruk
  minr[8] = Min(fuPBasa(), fuKKeruh());
  Rule[8] = BU;
  // R9 If pH Basa and kekeruhan Sangat Keruh then kualitas Sangat Buruk
  minr[9] = Min(fuPBasa(), fuKSangatKeruh());
  Rule[9] = SBU;
  // R10 If pH Sangat Asam and kekeruhan Jernih then kualitas Sangat Buruk
  minr[10] = Min(fuPSangatAsam(), fuKJernih());
  Rule[10] = SBU;
  // R11 If pH Sangat Asam and kekeruhan Keruh then kualitas Sangat Buruk
  minr[11] = Min(fuPSangatAsam(), fuKKeruh());
  Rule[11] = SBU;
  // R12 If pH Sangat Asam and kekeruhan Sangat Keruh then kualitas Sangat Buruk
  minr[12] = Min(fuPSangatAsam(), fuKSangatKeruh());
  Rule[12] = SBU;
  // R13 If pH Sangat Basa and kekeruhan Jernih then kualitas Sangat Buruk
  minr[13] = Min(fuPSangatBasa(), fuKJernih());
  Rule[13] = SBU;
  // R14 If pH Sangat Basa and kekeruhan Keruh then kualitas Sangat Buruk
  minr[14] = Min(fuPSangatBasa(), fuKKeruh());
  Rule[14] = SBU;
  // R15 If pH Sangat Basa and kekeruhan Sangat Keruh then kualitas Sangat Buruk
  minr[15] = Min(fuPSangatBasa(), fuKSangatKeruh());
  Rule[15] = SBU;
}

float defuzzyfikasi() {
  rule();
  A = 0;
  B = 0;

  for (int i = 1; i <= 15; i++) {
    A += Rule[i] * minr[i];
    B += minr[i];
  }
  return A / B;
}

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);  // Menggunakan GPIO16 sebagai RX dan GPIO17 sebagai TX

  EEPROM.begin(32);  //needed to permit storage of calibration value in eeprom
  ph.begin();

  // pinMode(TB_PIN, INPUT);
  antares.setDebug(true);
  antares.wifiConnection(WIFISSID, PASSWORD);

  // SSD1306_SWITCHCAPVCC = menghasilkan tegangan tampilan dari 3.3V secara internal
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Alokasi SSD1306 gagal"));
    for (;;)
      ;
  }
  display.display();
  display.clearDisplay();
  display.setTextSize(1);               // Skala piksel normal 1:1
  display.setTextColor(SSD1306_WHITE);  // Tampilkan teks putih
  display.setCursor(0, 0);              // Mulai di sudut kiri atas
  display.display();
}

void updateLCD() {
  char k1[] = "Sangat Buruk";
  char k2[] = "Buruk";
  char k3[] = "Baik";
  char k4[] = "Sangat Baik";
  static unsigned long timepoint = millis();
  if (millis() - timepoint > 1000U)  //time interval: 1s
  {
    timepoint = millis();
    //voltage = rawPinValue / esp32ADC * esp32Vin
    voltage = analogRead(PH_PIN) / ESPADC * ESPVOLTAGE;  // read the voltage
    phValue = ph.readPH(voltage, temperature = 25);  // convert voltage to pH with temperature compensation
  }

  String ntuString = "";
  kekeruhan = 0;
  String voltageString = "";
  float voltage1 = 0;

  if (Serial2.available()) {
    ntuString = Serial2.readString();
    kekeruhan = ntuString.toFloat();
  }

  if (kekeruhan < 0) {
    kekeruhan = 0;
  }

  Serial.println("-----------------------");
  Serial.print("PH : ");
  Serial.println(phValue);
  Serial.print("NTU : ");
  Serial.println(kekeruhan);
  Serial.println("-----------------------");

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.print("PH :");
  display.println(phValue);
  display.print("NTU:");
  display.println(kekeruhan, 4);
  display.setTextSize(1);
  display.println("");
  display.print("Kwt Air : ");
  display.println(defuzzyfikasi());

  // Himpunan Fuzzy PH
  String HimpunanPH;

  if (phValue >= 0 && phValue <= 5.5) {
    HimpunanPH = "Sangat Asam";
  } else if (phValue > 5.5 && phValue <= 6.5) {
    HimpunanPH = "Asam";
  } else if (phValue > 6.5 && phValue <= 7.5) {
    HimpunanPH = "Normal";
  } else if (phValue > 7.5 && phValue <= 9) {
    HimpunanPH = "Basa";
  } else if (phValue > 9) {
    HimpunanPH = "Sangat Basa";
  }

  // Himpunan Fuzzy Kekeruhan (KK)
  String HimpunanKK;

  if (kekeruhan >= 0 && kekeruhan <= 5) {
    HimpunanKK = "Jernih";
  } else if (kekeruhan > 5 && kekeruhan <= 25) {
    HimpunanKK = "Keruh";
  } else if (kekeruhan > 25) {
    HimpunanKK = "Sangat Keruh";
  }

  // Hasil Defuzzifikasi
  if (defuzzyfikasi() < 0.30) {
    display.print(k1);
  } else if (defuzzyfikasi() > 0.30 && defuzzyfikasi() < 0.50) {
    display.print(k2);
  } else if (defuzzyfikasi() > 0.50 && defuzzyfikasi() < 0.70) {
    display.print(k3);
  } else if (defuzzyfikasi() >= 0.70) {
    display.print(k4);
  }
  display.display();

  String kualitasAir;

  if (defuzzyfikasi() < 0.30) {
    kualitasAir = "Sangat Buruk";
  } else if (defuzzyfikasi() < 0.50 && defuzzyfikasi() >= 0.30) {
    kualitasAir = "Buruk";
  } else if (defuzzyfikasi() < 0.70 && defuzzyfikasi() >= 0.50) {
    kualitasAir = "Baik";
  } else if (defuzzyfikasi() > 0.70) {
    kualitasAir = "Sangat Baik";
  }
  float deffuz = defuzzyfikasi();
  String phValueStr = String(phValue, 2);      // Batasi pH menjadi 2 angka desimal
  String kekeruhanStr = String(kekeruhan, 2);  // Batasi Kekeruhan menjadi 2 angka desimal

  antares.add("pH", phValueStr);           // Masukkan pH dengan dua desimal
  antares.add("Kekeruhan", kekeruhanStr);  // Masukkan Kekeruhan dengan dua desimal
  antares.add("Kualitas Air", kualitasAir);
  antares.add("HimpunanPH", HimpunanPH);
  antares.add("HimpunanKK", HimpunanKK);
  antares.add("Defuzzifikasi", deffuz);
  antares.send(projectName, deviceName);
  delay(1000);
}

void loop() {
  unsigned long currentMillis = millis();

  // Update LCD jika interval waktu telah terpenuhi
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    updateLCD();
  }
}
