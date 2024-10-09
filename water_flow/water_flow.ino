volatile double waterFlow;  // Total de litros acumulados
volatile int pulseCount;    // Contador de pulsos por intervalo de tiempo
const int Water_flow_pin = 2;  // Pin del sensor de flujo de agua

unsigned long previousMillis = 0;  // Tiempo anterior para la tasa de flujo
const unsigned long interval = 1000;  // Intervalo de tiempo para medir los pulsos (1 segundo)

void setup()
{
  Serial.begin(9600);  // Inicializar monitor serial
  waterFlow = 0;
  pulseCount = 0;
  pinMode(Water_flow_pin, INPUT);  // Configurar el pin como entrada
  attachInterrupt(digitalPinToInterrupt(Water_flow_pin), pulse, RISING);  // Asignar el pin 2 al interruptor
}

void loop()
{
  unsigned long currentMillis = millis();  // Tiempo actual

  // Cada segundo calcula los litros por minuto
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Calcular litros por minuto (450 pulsos = 1 litro)
    double litersPerMinute = (pulseCount / 450.0) * 60.0;  // Convertir pulsos a litros y luego a litros/minuto

    // Mostrar resultados en el monitor serial
    Serial.print("Total de agua: ");
    Serial.print(waterFlow);
    Serial.println(" L");

    Serial.print("Litros por minuto: ");
    Serial.print(litersPerMinute);
    Serial.println(" L/m");

    // Reiniciar el contador de pulsos para el siguiente intervalo
    pulseCount = 0;
  }
}

void pulse()   // Incrementar el contador de pulsos y sumar al flujo total
{
  pulseCount++;  // Contar cada pulso para calcular los litros por minuto
  waterFlow += 1.0 / 450.0;  // Sumar la cantidad de litros totales
}
