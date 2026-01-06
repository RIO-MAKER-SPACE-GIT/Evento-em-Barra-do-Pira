#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP085_U.h>
#include <DHT.h>
#include <MPU6050.h>
#include <RTClib.h>
#include <SD.h>

// ----- Configurações de pinos -----
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
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

//DHT11
#define DHTPIN 4
#define DHTTYPE DHT11

//#define RTC 0x57;

// ----- Objetos -----
DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
MPU6050 mpu;
RTC_DS3231 rtc;
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
HardwareSerial GPS(2);
TinyGPSPlus gps;
SPIClass SPI_SD(HSPI);  // HSPI separado

// ----- Variáveis -----
File logFile;
String logFileName;
String msg;
float lat, lon, alt, speed;
float tempDHT, humDHT;
float tempBMP, presBMP, altitude;
int16_t ax, ay, az, gx, gy, gz;
DateTime rtcNow;
int screen = 0;  // alterna telas
unsigned long lastSwitch = 0;

// ----- ERROS / FLAGS -----
uint8_t errorMask = 0;  // bit0 RTC, bit1 GPS, bit2 MPU, bit3 BMP, bit4 DHT, bit5 LoRa
bool rtcOK = false, gpsOK = false, mpuOK = false, bmpOK = false, dhtOK = false, loraOK = false, SD_OK = false;

// ----- Protótipos -----
bool initBMP280();
bool initDHTQuick();
bool initLoRaModule();
void checkSensorsAndShow();

void setup() {
  Serial.begin(115200);
  Wire.begin();
  GPS.begin(9600, SERIAL_8N1, 34, -1);  // RX, TX – ajuste conforme seu LilyGO

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) Serial.println("Erro OLED");
  display.clearDisplay();
  display.setCursor(0, 10);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  checkSensorsAndShow();  // faz o autoteste ao ligar
}

void loop() {
  // ----- Ler GPS -----
  while (GPS.available()) {
    char c = GPS.read();
    gps.encode(c);
    Serial.write(c);
  }

  if (gps.location.isValid() && gps.location.age() < 2000) {
    //GPS válido
    lat = gps.location.lat();
    lon = gps.location.lng();
    alt = gps.altitude.meters();
    speed = gps.speed.kmph();

    // ----- Sincronizar RTC com GPS -----
    if (gps.date.isValid() && gps.time.isValid()) {
      DateTime gpsTime(
        gps.date.year(), gps.date.month(), gps.date.day(),
        gps.time.hour(), gps.time.minute(), gps.time.second());
      rtcNow = rtc.now();
      if (labs((long)(gpsTime.unixtime() - rtcNow.unixtime())) > 5) {
        rtc.adjust(gpsTime);
        Serial.println("RTC atualizado pelo GPS");
      }
    }
  } else {
    // Sem fix do GPS
    lat = lon = alt = speed = NAN;
  }

  // ----- Ler RTC (sempre válido) -----
  rtcNow = rtc.now();
  int hour = rtcNow.hour() - 3;  // UTC-3
  if (hour < 0) hour += 24;
  int minute = rtcNow.minute();
  int second = rtcNow.second();

  // ----- Ler DHT11 -----
  tempDHT = dht.readTemperature();
  humDHT = dht.readHumidity();

  // ----- Ler BMP180 -----
  sensors_event_t event;
  bmp.getEvent(&event);
  if (event.pressure) {
    bmp.getTemperature(&tempBMP);
    presBMP = event.pressure;
    altitude = bmp.pressureToAltitude(1022, presBMP);
  }

  // ----- Ler MPU6050 -----
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // ----- Montar mensagem -----
  msg = "RTC:" + String(rtcNow.day()) + "/" + String(rtcNow.month()) + "/" + String(rtcNow.year()) + "_" + String(hour) + ":" + String(minute) + ":" + String(second);
  msg += ";GPS:";
  msg += isnan(lat) ? "NO_GPS" : String(lat, 6) + "," + String(lon, 6) + "," + String(alt, 1) + "," + String(speed, 1);
  msg += ";DHT:" + String(tempDHT, 1) + "," + String(humDHT, 1);
  msg += ";BMP:" + String(tempBMP, 1) + "," + String(presBMP, 1) + "," + String(altitude, 1);
  msg += ";MPU:" + String(ax) + "," + String(ay) + "," + String(az) + "," + String(gx) + "," + String(gy) + "," + String(gz);

  // Enviar via LoRa
  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();

  // Grava no SD
  logToSD(msg);

  // ----- Alternar telas no OLED -----
  if (millis() - lastSwitch > 1000) {  // troca a cada 1s
    screen = !screen;
    lastSwitch = millis();
  }

  // ----- Mostrar no OLED -----
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("BEC ASA TX:");

  if (screen == 0) {
    if (gps.location.isValid()) {
      display.println(" ");
      display.print("Lat: ");
      display.println(lat, 6);
      display.print("Lon: ");
      display.println(lon, 6);
      display.print("Alt: ");
      display.print(alt, 1);
      display.println(" m");
      display.print("Spd: ");
      display.print(speed, 1);
      display.println(" km/h");
      display.println("RTC: " + String(hour) + ":" + String(minute) + ":" + String(second));
      display.print("RTC: " + String(rtcNow.day()) + "/" + String(rtcNow.month()) + "/" + String(rtcNow.year()));
    } else {
      display.println("NO GPS FIX");
      display.println("RTC: " + String(hour) + ":" + String(minute) + ":" + String(second));
      display.print("RTC: " + String(rtcNow.day()) + "/" + String(rtcNow.month()) + "/" + String(rtcNow.year()));
    }
  } else {
    display.println(" ");
    display.print("DHT T:");
    display.print(tempDHT, 1);
    display.print("C H:");
    display.println(humDHT, 1);
    display.println("BMP T:" + String(tempBMP, 1) + " P:" + String(presBMP, 1));
    display.println("A:" + String(altitude, 1));
    display.println("MPU:" + String(ax) + "," + String(ay) + "," + String(az));
    display.println("MPU:" + String(gx) + "," + String(gy) + "," + String(gz));
  }

  display.display();
  delay(1000);
}

bool initBMP180() {
  // tenta ambos endereços comuns
  if (!bmp.begin()) return true;
  return false;
}

bool initDHTQuick() {
  dht.begin();
  delay(1200);  // DHT11 é lentinho
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(t) || isnan(h)) {
    // tenta 2ª leitura
    delay(1200);
    t = dht.readTemperature();
    h = dht.readHumidity();
  }
  return !(isnan(t) || isnan(h));
}

bool initLoRaModule() {
  // LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_BAND)) return false;

  // Configurações para alcance máximo
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setTxPower(20);  // Máximo

  display.clearDisplay();
  display.setCursor(0, 10);
  display.println("LoRa OK - TX");
  display.display();
  return true;
}

void checkSensorsAndShow() {
  errorMask = 0;

  // RTC
  rtcOK = rtc.begin();
  if (!rtcOK) errorMask |= (1 << 0);

  // MPU6050
  mpu.initialize();
  mpuOK = mpu.testConnection();
  if (!mpuOK) errorMask |= (1 << 2);

  // BMP280 (GY-68)
  bmpOK = initBMP180();
  if (!bmpOK) errorMask |= (1 << 3);

  // DHT11
  dhtOK = initDHTQuick();
  if (!dhtOK) errorMask |= (1 << 4);

  // LoRa
  loraOK = initLoRaModule();
  if (!loraOK) errorMask |= (1 << 5);

  //SD CARD
  SD_OK = initSD();
  if (!SD_OK) errorMask |= (1 << 6);

  // GPS: verifica se está chegando algo na serial por ~1s
  size_t before = gps.charsProcessed();
  unsigned long t0 = millis();
  while (millis() - t0 < 1000) {
    if (GPS.available()) gps.encode(GPS.read());
  }
  gpsOK = (gps.charsProcessed() > before);
  if (!gpsOK) errorMask |= (1 << 1);

  // Mostrar status no OLED
  display.clearDisplay();
  display.setCursor(0, 10);
  display.setTextSize(1);
  display.println("AUTOTESTE:");

  if (errorMask == 0) {
    display.println("MODULOS OK");
  } else {
    if (errorMask & (1 << 0)) display.println("E01 RTC");
    if (errorMask & (1 << 1)) display.println("E02 GPS");
    if (errorMask & (1 << 2)) display.println("E03 MPU");
    if (errorMask & (1 << 3)) display.println("E04 BMP");
    if (errorMask & (1 << 4)) display.println("E05 DHT");
    if (errorMask & (1 << 5)) display.println("E06 LORA");
    if (errorMask & (1 << 6)) display.println("E07 SD");
  }
  display.display();
  delay(2500);  // dá tempo de ler os erros
}

bool initSD() {
  pinMode(SD_SS, OUTPUT);
  digitalWrite(SD_SS, HIGH);
  SPI_SD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_SS);
  if (!SD.begin(SD_SS, SPI_SD)) {
    Serial.println("Erro E07: Falha ao inicializar SD!");
    return false;
  }

  // Obtém hora do RTC
  DateTime now;
  bool rtcValido = false;

  if (rtc.begin()) {
    now = rtc.now();
    // Considera inválido se ainda estiver no default (2000/01/01 00:00:00)
    if (now.year() > 2020) {
      rtcValido = true;
    }
  }

  // Se RTC válido → cria arquivo com timestamp real
  if (rtcValido) {
    // Monta nome do arquivo: YYYYMMDD_HHMMSS.txt
    char filename[32];
    sprintf(filename, "/%04d%02d%02d_%02d%02d%02d.txt",
            now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());

    logFileName = String(filename);
  } else {
    // Se RTC inválido → cria arquivo com millis
    char filename[32];
    sprintf(filename, "bootMillis_%lu.txt", millis());
    logFileName = String(filename);
  }

  // Cria arquivo
  logFile = SD.open(logFileName.c_str(), FILE_WRITE);
  if (!logFile) {
    Serial.println("Erro E07: Falha ao criar arquivo no SD!");
    return false;
  }

  Serial.print("Arquivo de log criado: ");
  Serial.println(logFileName);
  logFile.println("==== Iniciando Log ====");
  logFile.close();  // fecha aqui, abre só quando for gravar
  return true;
}

// Função para gravar dados
void logToSD(String data) {
  logFile = SD.open(logFileName.c_str(), FILE_APPEND);
  if (logFile) {
    logFile.println(data);
    logFile.close();
  } else {
    Serial.println("Erro E07: Nao foi possivel abrir o arquivo para escrita!");
  }
}