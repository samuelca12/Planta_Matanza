#include "DFRobot_PH.h"
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#ifdef USE_PULSE_OUT
  #include "orp_iso_surveyor.h"       
  Surveyor_ORP_Isolated ORP = Surveyor_ORP_Isolated(A0);         
#else
  #include "orp_surveyor.h"
  Surveyor_ORP ORP = Surveyor_ORP(A0);
#endif

#define PH_PIN A1
#define TdsSensorPin A2
#define VREF 5.0              // analog reference voltage(Volt) of the ADC
#define SCOUNT  30            // sum of sample points for TDS
#define Water_flow_pin 2      // Pin del sensor de flujo de agua

// Configuración de pantalla LCD
LiquidCrystal_I2C lcd(0x27, 20, 2);  // Dirección I2C, 20 columnas, 2 filas

// Variables para pH, TDS y ORP
float voltage, phValue, temperature = 25;
DFRobot_PH ph;
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;
float averageVoltage = 0;
float tdsValue = 0;

// Variables para flujo de agua
volatile double waterFlow = 0;  
volatile int pulseCount = 0;
unsigned long previousMillis = 0;
const unsigned long interval = 1000;  // 1 segundo para medir flujo

uint8_t user_bytes_received = 0;
const uint8_t bufferlen = 32;
char user_data[bufferlen];

// Variables para la visualización alternada
unsigned long displayMillis = 0;
int displayState = 0;  // Alterna entre ORP+pH, Agua+Flujo, TDS

// Algoritmo de filtrado de mediana para TDS
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++) {
    bTab[i] = bArray[i];
  }
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  } else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

// Función para procesar comandos de calibración
void parse_cmd(char* string) {
  strupr(string);
  String cmd = String(string);
  if (cmd.startsWith("CAL")) {
    int index = cmd.indexOf(',');
    if (index != -1) {
      String param = cmd.substring(index + 1, cmd.length());
      if (param.equals("CLEAR")) {
        ORP.cal_clear();
        Serial.println("CALIBRATION CLEARED");
      } else {
        int cal_param = param.toInt();
        ORP.cal(cal_param);
        Serial.println("ORP CALIBRATED");
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("Use command \"CAL,xxx\" to calibrate the ORP circuit to the value xxx \n\"CAL,CLEAR\" clears the calibration"));

  // Configurar sensores de ORP, pH y TDS
  if (ORP.begin()) {
    Serial.println("Loaded EEPROM for ORP");
  }
  ph.begin();
  pinMode(TdsSensorPin, INPUT);

  // Configurar sensor de flujo de agua
  pinMode(Water_flow_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(Water_flow_pin), pulse, RISING);

  // Inicializar LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Iniciando...");
  delay(2000);  // Mostrar mensaje de inicio
  lcd.clear();
}

void loop() {
  // ORP Reading
  if (Serial.available() > 0) {
    user_bytes_received = Serial.readBytesUntil(13, user_data, sizeof(user_data));
  }

  if (user_bytes_received) {
    parse_cmd(user_data);
    user_bytes_received = 0;
    memset(user_data, 0, sizeof(user_data));
  }

  int orpValue = (int)ORP.read_orp();

  // pH Reading
  static unsigned long timepoint = millis();
  if (millis() - timepoint > 1000U) {
    timepoint = millis();
    voltage = analogRead(PH_PIN) / 1024.0 * 5000;
    phValue = ph.readPH(voltage, temperature);
  }

  // TDS Reading
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }
  }

  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    }
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
    float compensationVoltage = averageVoltage / compensationCoefficient;
    tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage 
                - 255.86 * compensationVoltage * compensationVoltage 
                + 857.39 * compensationVoltage) * 0.5;
  }

  // Lectura del flujo de agua
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    double litersPerMinute = (pulseCount / 450.0) * 60.0;
    pulseCount = 0;
    
    // Alternar entre mostrar ORP+pH, Agua+Flujo, TDS
    if (millis() - displayMillis > 5000) {  // Cambiar cada 5 segundos
      displayMillis = millis();
      displayState = (displayState + 1) % 3;
    }

    lcd.clear();
    switch (displayState) {
      case 0:
        lcd.setCursor(0, 0);
        lcd.print("ORP: ");
        lcd.print(orpValue);
        lcd.setCursor(0, 1);
        lcd.print("pH: ");
        lcd.print(phValue, 2);
        break;

      case 1:
        lcd.setCursor(0, 0);
        lcd.print("Total Agua: ");
        lcd.print(waterFlow, 2);
        lcd.setCursor(0, 1);
        lcd.print("L/min: ");
        lcd.print(litersPerMinute, 2);
        break;

      case 2:
        lcd.setCursor(0, 0);
        lcd.print("TDS: ");
        lcd.print(tdsValue, 0);
        lcd.setCursor(8, 0);
        lcd.print("ppm");
        break;
    }
  }

  ph.calibration(voltage, temperature);  // Calibración del sensor de pH
}

// Incrementar el contador de pulsos y sumar al flujo total
void pulse() {
  pulseCount++;
  waterFlow += 1.0 / 450.0;
}
