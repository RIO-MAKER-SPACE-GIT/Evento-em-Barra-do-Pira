#include <Wire.h>
#include <Servo.h>  // Inclui a biblioteca Servo para controlar servos
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

Servo meuServoL;     // Cria um objeto Servo para controlar o servo motor esquerdo
Servo meuServoR;     // Cria um objeto Servo para controlar o servo motor direito
int posL, posR;      // Variáveis para armazenar as posições dos servos motores
int ADXL345 = 0x53;  // The ADXL345 sensor I2C address
float X_out, Y_out, Z_out, Xin, Yin, Lng0, Lt0, Lt1, Lt2, Lg0, Lg1, Lg2,
  va1, va2, pGPS;                                                                         // Outputs
int X_offset = 0, Y_offset = 0, Z_offset = 0, it0, it1, it2, tc, rampsR = 0, rampsL = 0;  //Offset values
static const int Rx = 3, Tx = 4;
static const uint32_t GPSBaud = 9600;
bool liberacaoOK = false;

TinyGPSPlus gps;
SoftwareSerial ss(Rx, Tx);

void setup() {
  Serial.begin(9600);
  ss.begin(GPSBaud);  //inicia o GPS
  Wire.begin();
  configureADXL345();  // Configure the sensor
  // Note: You should calibrate upon re-powering the sensor
  // Uncomment if you want to calibrate
  // calibrateADXL345(); // Calibrate the sensor
  meuServoL.attach(6);   // Associa o servo motor ao pino digital 6 do Arduino
  meuServoL.write(45);   // Define a posição inicial do servo motor para 90 graus
  meuServoR.attach(5);   // Associa o servo motor ao pino digital 5 do Arduino
  meuServoR.write(135);  // Define a posição inicial do servo motor para 90 graus
  Lt0 = -22.471;         //latitude do local de pouso
  Lg0 = -42.796;         //Longitude do local de pouso
  tc = 0;                // contador de voltas
  it0 = 7200000;         //timer para liberação da asa (em minutos)
  it1 = 1000;            //intervalo do sistema de estabilização (em milissegundos)
  it2 = 60000;           //intervalo dos inputs de correção de trajetória (em milissegundos)
  pGPS = 0;
}
void calibrateADXL345() {
  // calibrar com o acelerômetro disposto sobre uma superfície plana ao (re)iniciar o sistema.

  float numReadings = 500;
  float zSum = 0;
  Serial.print("Beginning Calibration");
  Serial.println();
  for (int i = 0; i < numReadings; i++) {
    Serial.print(i);
    Serial.println();
    Wire.beginTransmission(ADXL345);
    Wire.write(0x32);  // Start with register 0x32 (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(ADXL345, 6, true);
    X_out = (Wire.read() | Wire.read() << 8);
    Y_out = (Wire.read() | Wire.read() << 8);
    Z_out = (Wire.read() | Wire.read() << 8);
    // xSum += X_out;
    // ySum += Y_out;
    zSum += Z_out;
  }

  Z_offset = (256 - (zSum / numReadings)) / 4;
  Serial.print("Z_offset= ");
  Serial.print(Z_offset);
  Serial.println();
  delay(1000);

  Wire.beginTransmission(ADXL345);
  Wire.write(0x20);
  Wire.write(Z_offset);
  Wire.endTransmission();
}

void configureADXL345() {
  Wire.beginTransmission(ADXL345);
  Wire.write(0x2D);  // Access/ talk to POWER_CTL Register - 0x2D
  Wire.write(8);     // Enable measurement (D3 bit high)
  Wire.endTransmission();
  delay(10);
}

void loop() {

  if ((millis() >= it0) && !liberacaoOK) {  // tempo máximo de voo, hora da liberação

    liberacaoOK = true;

    rampsR = 135;
    rampsL = 45;

    while ((rampsR != 45) && (rampsL != 135)) {
      rampsR--;
      meuServoR.write(rampsR);  // liberação!
      rampsL++;
      meuServoL.write(rampsL);
      delay(90);
    }
  }

  delay(5000);

  if (tc * tc > 64) {
    it2 = 30000;
  } else if (tc * tc > 256) {
    it2 = 17000;
  } else {
    it2 = 60000;
  }

  if (ss.available()) {
    char c = ss.read();
    gps.encode(c);
    Serial.write(c);

    if (millis() - pGPS >= it2) {
      pGPS = millis();
      if (gps.location.isValid()) {
        Lt1 = Lt2;
        Lg1 = Lg2;
        Lt2 = gps.location.lat();
        Lg2 = gps.location.lng();
        va1 = (Lt1 - Lt2) * (Lt2 - Lt0) - (Lg2 - Lg1) * (Lg2 - Lg0);
        va2 = (Lt1 - Lt2) * (Lg2 - Lg0) + (Lg2 - Lg1) * (Lg2 - Lt0);
        if (va1 < -0.1) {  //virando para a direita 60 graus
          tc = tc + 1;
          meuServoL.write(135);  // Define a posição do servo motor esquerdo
          meuServoR.write(135);  // Define a posição do servo motor direito
          delay(2000);
          meuServoL.write(45);
          delay(4000);
          meuServoL.write(135);
          delay(2000);
          meuServoL.write(45);  // Define a posição do servo motor esquerdo
          meuServoR.write(45);  // Define a posição do servo motor direito
          delay(1000);
          meuServoL.write(90);  // Define a posição do servo motor esquerdo
          meuServoR.write(90);  // Define a posição do servo motor direito
          delay(100);           // Aguarda um curto intervalo para estabilizar o movimento dos servos
        }
        if (va1 > 0.1) {  //virando para a esquerda 60 graus
          tc = tc - 1;
          meuServoL.write(45);  // Define a posição do servo motor esquerdo
          meuServoR.write(45);  // Define a posição do servo motor direito
          delay(2000);
          meuServoL.write(135);
          delay(4000);
          meuServoL.write(45);
          delay(2000);
          meuServoL.write(135);  // Define a posição do servo motor esquerdo
          meuServoR.write(135);  // Define a posição do servo motor direito
          delay(1000);
          meuServoL.write(90);  // Define a posição do servo motor esquerdo
          meuServoR.write(90);  // Define a posição do servo motor direito
          delay(100);           // Aguarda um curto intervalo para estabilizar o movimento dos servos
        } else if (va2 > 0) {   //virando para a direita 60 graus
          tc = tc + 1;
          meuServoL.write(135);  // Define a posição do servo motor esquerdo
          meuServoR.write(135);  // Define a posição do servo motor direito
          delay(2000);
          meuServoL.write(45);
          delay(4000);
          meuServoL.write(135);
          delay(2000);
          meuServoL.write(45);  // Define a posição do servo motoresquerdo
          meuServoR.write(45);  // Define a posição do servo motor direito
          delay(1000);
          meuServoL.write(90);  // Define a posição do servo motor esquerdo
          meuServoR.write(90);  // Define a posição do servo motor direito
          delay(100);           // Aguarda um curto intervalo para estabilizar o movimento dos servos
        }
      }
    } else {

      tc = tc + 1;
      meuServoL.write(135);  // Define a posição do servo motor esquerdo
      meuServoR.write(135);  // Define a posição do servo motor direito
      delay(2000);
      meuServoL.write(45);
      delay(4000);
      meuServoL.write(135);
      delay(2000);
      meuServoL.write(45);  // Define a posição do servo motor esquerdo
      meuServoR.write(45);  // Define a posição do servo motor direito
      delay(1000);
      meuServoL.write(90);  // Define a posição do servo motor esquerdo
      meuServoR.write(90);  // Define a posição do servo motor direito
      delay(100);           // Aguarda um curto intervalo para estabilizar o movimento dos servos
    }
    meuServoL.write(posL);  // Define a posição do servo motor esquerdo
    meuServoR.write(posR);  // Define a posição do servo motor direito
    delay(100);             // Aguarda um curto intervalo para estabilizar o movimento dos servos

    posL = 90;
    posR = 90;  // os valores voltam para a posição neutra
    Wire.beginTransmission(ADXL345);
    Wire.write(0x32);  // Start with register 0x32 (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(ADXL345, 6, true);
    X_out = (Wire.read() | Wire.read() << 8);
    X_out = X_out;
    Y_out = (Wire.read() | Wire.read() << 8);
    Y_out = Y_out;
    Z_out = (Wire.read() | Wire.read() << 8);
    Z_out = Z_out;
    Xin = X_out / Z_out;
    Yin = (Y_out - 25) / Z_out;
    if (Xin <= -0.15) {
      posL = posL - 45;
      posR = posR + 45;
    } else if (Xin >= 0.15) {
      posL = posL + 45;
      posR = posR - 45;
    }

    if (Yin <= -0.15) {
      posL = posL + 45;
      posR = posR + 45;
    } else if (Yin >= 0.15) {
      posL = posL - 45;
      posR = posR - 45;
    }
    Serial.print("Xa= ");
    Serial.print(X_out);
    Serial.print(" Ya= ");
    Serial.print(Y_out);
    Serial.print(" Za= ");
    Serial.print(Z_out);
    Serial.print(" Xin= ");
    Serial.print(Xin);
    Serial.print(" Yin= ");
    Serial.print(Yin);
    Serial.print(" SE= ");
    Serial.print(posL);
    Serial.print(" SD= ");
    Serial.println(posR);
    delay(500);
  }
}