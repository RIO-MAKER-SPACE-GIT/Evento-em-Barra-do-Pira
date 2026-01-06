#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_SSD1306.h>
#include <SD.h>
#include <RTClib.h>  // RTC DS3231

#define OLED_WIDTH 128
#define OLED_HEIGHT 64
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// Cria objetos SPI separados
//SPIClass SPI_LoRa(VSPI);  // VSPI padrão
SPIClass SPI_SD(HSPI);  // HSPI separado

RTC_DS3231 rtc;
File logFile;

unsigned long lastDisplaySwitch = 0;
bool showPage1 = true;
unsigned long lastPacketTime = 0;           // armazena quando chegou o último pacote
const unsigned long signalTimeout = 30000;  // 5s sem comunicação = NO SIGNAL
String logFileName = "/log.txt";

//SPI
#define SD_SCK 14   // SD SCK
#define SD_MISO 2   // SD MISO
#define SD_MOSI 15  // SD MOSI
#define SD_SS 13    // SD CS

//LORA
#define LORA_SCK 5    // GPIO5  -- SX1278's SCK
#define LORA_MISO 19  // GPIO19 -- SX1278's MISnO
#define LORA_MOSI 27  // GPIO27 -- SX1278's MOSI
#define LORA_SS 18
#define LORA_RST -1
#define LORA_DIO0 26
#define LORA_BAND 915E6

String lastMsg = "";
int displayScreen = 0;
unsigned long lastSwitch = 0;
const unsigned long switchInterval = 1000;  // 1s

void setup() {
  Serial.begin(115200);

  // OLED
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Receptor LoRa");
  display.display();

  // RTC
  if (!rtc.begin()) {
    Serial.println("ERRO RTC!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println("Erro RTC");
    display.display();
    Serial.println("Erro RTC");
    while (1)
      ;  // trava
  }

  // Se o RTC perdeu energia, inicializa com a hora da compilação
  if (rtc.lostPower()) {
    Serial.println("RTC sem energia, ajustando para hora de compilação...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // // SD
  pinMode(SD_SS, OUTPUT);
  digitalWrite(SD_SS, HIGH);
  SPI_SD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_SS);
  if (!SD.begin(SD_SS, SPI_SD)) {
    Serial.println("ERRO SD!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println("Erro SD");
    display.display();
    Serial.println("Erro SD");
    while (1)
      ;  // trava
  }

  // no setup(): SE quiser criar/cabecalho
  File f = SD.open(logFileName.c_str(), FILE_WRITE);
  if (f) {
    f.println("==== Iniciando Log ====");
    f.close();
  }

  // // LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_BAND)) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println("Erro LoRa");
    display.display();
    Serial.println("Erro LoRa");
    while (1)
      ;  // trava
  }

  // // Configurações para máximo alcance
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);

  // --- Se chegou até aqui, tudo OK ---
  display.clearDisplay();
  display.setCursor(0, 20);
  display.setTextSize(2);
  display.println("READY");
  Serial.println("Sistema inicializado com sucesso");
  display.display();
  delay(2000);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("LoRa OK - RX");
  display.display();
  delay(2000);
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String msg = "";

    while (LoRa.available()) {
      msg += (char)LoRa.read();
    }

    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();
    lastPacketTime = millis();  // atualiza último pacote recebido

    Serial.println("Recebido: " + msg);
    Serial.println("RSSI: " + String(rssi) + " SNR: " + String(snr));

    // Variáveis dos sensores
    String rtcStr = "", gpsStr = "", dhtStr = "", bmpStr = "", mpuStr = "";
    String lat = "", lon = "", alt = "", spd = "";
    String tempDHT = "", humDHT = "";
    String tempBMP = "", presBMP = "", altBMP = "";
    String ax = "", ay = "", az = "", gx = "", gy = "", gz = "";

    // ----- Quebrar por ";"
    int lastIdx = 0, idx;
    while ((idx = msg.indexOf(';', lastIdx)) != -1) {
      String part = msg.substring(lastIdx, idx);
      lastIdx = idx + 1;

      if (part.startsWith("RTC:")) rtcStr = part.substring(4);
      if (part.startsWith("GPS:")) gpsStr = part.substring(4);
      if (part.startsWith("DHT:")) dhtStr = part.substring(4);
      if (part.startsWith("BMP:")) bmpStr = part.substring(4);
      if (part.startsWith("MPU:")) mpuStr = part.substring(4);
    }

    //Ajusta RTC
    if (rtcStr.length() > 0) {
      int d1 = rtcStr.indexOf('/');
      int d2 = rtcStr.indexOf('/', d1 + 1);
      int d3 = rtcStr.indexOf('_', d2 + 1);
      int c1 = rtcStr.indexOf(':', d3 + 1);
      int c2 = rtcStr.indexOf(':', c1 + 1);

      int day = rtcStr.substring(0, d1).toInt();
      int month = rtcStr.substring(d1 + 1, d2).toInt();
      int year = rtcStr.substring(d2 + 1, d3).toInt();
      int hour = rtcStr.substring(d3 + 1, c1).toInt();
      int min = rtcStr.substring(c1 + 1, c2).toInt();
      int sec = rtcStr.substring(c2 + 1).toInt();

      DateTime txTime(year, month, day, hour, min, sec);

      if (labs((long)(txTime.unixtime() - rtc.now().unixtime())) > 5) {
        rtc.adjust(txTime);
        Serial.println("RTC RX atualizado pelo TX");
      }

      DateTime rtcNow = rtc.now();

      String tsDate = String(rtcNow.day()) + "/" + String(rtcNow.month()) + "/" + String(rtcNow.year());

      String tsTime = String(rtcNow.hour()) + ":" + String(rtcNow.minute()) + ":" + String(rtcNow.second());

      // ----- Processar GPS (lat,lon,alt,spd)
      if (gpsStr.length() > 0 && gpsStr != "NO_GPS") {
        int i1 = gpsStr.indexOf(',');
        int i2 = gpsStr.indexOf(',', i1 + 1);
        int i3 = gpsStr.indexOf(',', i2 + 1);

        lat = gpsStr.substring(0, i1);
        lon = gpsStr.substring(i1 + 1, i2);
        alt = gpsStr.substring(i2 + 1, i3);
        spd = gpsStr.substring(i3 + 1);  // até o fim

        Serial.println("Lat: " + lat);
        Serial.println("Lon: " + lon);
        Serial.println("Alt: " + alt);
        Serial.println("Spd: " + spd);
      } else {
        lat = lon = alt = spd = "NO_GPS";
      }

      // ----- Processar DHT (temp,hum)
      if (dhtStr.length() > 0) {
        int i = dhtStr.indexOf(',');
        tempDHT = dhtStr.substring(0, i);
        humDHT = dhtStr.substring(i + 1);

        Serial.println("Temp DHT: " + tempDHT);
        Serial.println("Hum DHT: " + humDHT);
      }

      // ----- Processar BMP (temp,press,alt)
      if (bmpStr.length() > 0) {
        int i1 = bmpStr.indexOf(',');
        int i2 = bmpStr.indexOf(',', i1 + 1);
        tempBMP = bmpStr.substring(0, i1);
        presBMP = bmpStr.substring(i1 + 1, i2);
        altBMP = bmpStr.substring(i2 + 1);

        Serial.println("Temp BMP: " + tempBMP);
        Serial.println("Press BMP" + presBMP);
        Serial.println("Alt BMP: " + altBMP);
      }

      // ----- Processar MPU (ax,ay,az,gx,gy,gz)
      if (mpuStr.length() > 0) {
        int i1 = mpuStr.indexOf(',');
        int i2 = mpuStr.indexOf(',', i1 + 1);
        int i3 = mpuStr.indexOf(',', i2 + 1);
        int i4 = mpuStr.indexOf(',', i3 + 1);
        int i5 = mpuStr.indexOf(',', i4 + 1);
        ax = mpuStr.substring(0, i1);
        ay = mpuStr.substring(i1 + 1, i2);
        az = mpuStr.substring(i2 + 1, i3);
        gx = mpuStr.substring(i3 + 1, i4);
        gy = mpuStr.substring(i4 + 1, i5);
        gz = mpuStr.substring(i5 + 1);
      }

      // ----- Gravar no SD
      File logFile = SD.open(logFileName.c_str(), FILE_APPEND);  // FILE_WRITE já faz append
      if (logFile) {
        logFile.println(msg + "," + String(rssi) + "," + String(snr, 1));
        logFile.close();
      } else {
        Serial.println("Erro E07: Nao foi possivel abrir o arquivo para escrita!");
      }

      // ----- Mostrar no OLED (duas telas alternando)
      if (millis() - lastDisplaySwitch > 1000) {
        showPage1 = !showPage1;
        lastDisplaySwitch = millis();
      }

      display.clearDisplay();
      display.setCursor(0, 0);
      display.setTextSize(1);

      if (showPage1) {
        display.println("GPS RX:");
        display.println(" ");
        display.print("Lat: ");
        display.println(lat);
        display.print("Lon: ");
        display.println(lon);
        display.print("Alt: ");
        display.println(alt + " m");
        display.print("Spd: ");
        display.println(spd + " km/h");
      } else {
        display.println("SINAL E TEMPO:");
        display.println(" ");
        display.print("RSSI: ");
        display.println(rssi);
        display.print("SNR: ");
        display.println(snr, 1);
        display.print("Data: ");
        display.println(tsDate);
        display.print("Hora: ");
        display.println(tsTime);
      }

      display.display();
    }
    // --- Verificação de timeout ---
    if ((millis() - lastPacketTime) > signalTimeout) {
      display.clearDisplay();
      display.setCursor(0, 20);
      display.setTextSize(2);
      display.println("NO SIGNAL");
      display.display();
    }
  }
}