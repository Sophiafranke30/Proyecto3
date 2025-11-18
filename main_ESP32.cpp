//Universidad del Valle de Guatemala
//Electronica Digital II | Proyecto III
//Sistema de Terapia Respiratoria
//Sophia Franke | 23030
//Dulce Ovando | 23441

// ---------- LIBRERÍAS ----------
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_NeoPixel.h>

// ---------- CONFIGURACIÓN PINES ----------
#define I2C_SDA 21
#define I2C_SCL 22
#define NEOPIX_PIN 4
#define LED_COUNT 16
#define RXD2 16
#define TXD2 17

// ---------- DEFINICIÓN DE LOS SENSORES ----------
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;
Adafruit_NeoPixel strip(LED_COUNT, NEOPIX_PIN, NEO_GRB + NEO_KHZ800);

// ---------- VARIABLES GLOBALES ----------
//Las variables que se utilizaron son para la temperatura, humedad y presión. Estas variables almacenan los valores actuales y de línea base. Además, se incluyen variables para el historial de temperatura y la animación de respiración, mejorando la detección de la misma y haciendo el código más eficiente.
#define TEMP_SAMPLES 10
float tempHistory[TEMP_SAMPLES];
int tempIndex = 0;
float baselineTemp = 0;
bool baselineReady = false;
float lastTemp = 0;
bool firstTemp = true;
uint8_t breathBrightness = 0;
bool breathIncreasing = true;
unsigned long lastBreathAnim = 0;
float currentTemp = 0;
float currentHum = 0;
float currentPressure = 0;
unsigned long measurementStartTime = 0;
#define MEASUREMENT_TIMEOUT 15000  // 15 segundos timeout para evitar que se quede trabado

// ---------- VARIABLES DE ESTADO ----------
// Estados de respiración
enum BreathState {
  BREATH_IDLE,
  BREATH_INHALE,
  BREATH_EXHALE
};
BreathState breathState = BREATH_IDLE;

// Estados del sistema
enum SystemState {
  STATE_IDLE,
  STATE_MEASURING,
  STATE_SENDING_DATA,
  STATE_ERROR
};
SystemState currentState = STATE_IDLE;

// ---------- PROTOTIPOS DE FUNCIONES ----------
bool initializeSensors();
void calibrateBaseline();
void updateTemperatureHistory(float temp);
float getTempAverage();
void readSensors();
void handleUARTCommand(uint8_t cmd);
void processMeasurement();
void sendDataToSTM32();
void setNeoPixelColor(uint8_t r, uint8_t g, uint8_t b);
void systemStatusLED();
void detectBreathing(float currentTemp);
void animateBreathing();

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  
  pinMode(RXD2, INPUT_PULLUP);
  pinMode(TXD2, INPUT_PULLUP);

  // Configuración UART2 para comunicación con STM32
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial2.setTimeout(100);
  
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("       SISTEMA TERAPIA RESPIRATORIA       ");
  Serial.println("         Sophia Franke | Dulce Ovando         ");
  Serial.println("========================================");
  
  // Inicializar I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  
  // Inicializar NeoPixel
  strip.begin();
  strip.show();
  strip.setBrightness(60);
  
  // Inicializar sensores
  if (!initializeSensors()) {
    Serial.println(">>> ERROR: Falló inicialización de sensores");
    currentState = STATE_ERROR;
    setNeoPixelColor(255, 0, 0); // Rojo para error
    return;
  }
  
  // Obtener medición de línea base para poder determinar cambios de temperatura
  calibrateBaseline();
  
  Serial.println(">>> Sistema listo - Esperando comandos STM32...");
  setNeoPixelColor(0, 255, 0); // Verde para listo
}

// ---------- LOOP PRINCIPAL ----------
void loop() {
  static unsigned long lastSensorRead = 0;
  unsigned long now = millis();
  
  // Leer comandos UART cada ciclo
  if (Serial2.available() > 0) {
    uint8_t command = Serial2.read();
    handleUARTCommand(command);
  }
  
  // Leer sensores periódicamente (1s)
  if (now - lastSensorRead >= 1000) {
    readSensors();
    lastSensorRead = now;
  }
  
  // Procesar medición si está activa
  if (currentState == STATE_MEASURING) {
    processMeasurement();
    animateBreathing(); 
  }
  
  // LED de estado del sistema
  systemStatusLED();
  
  delay(50);
}

// ---------- INICIALIZACIÓN DE SENSORES ----------
bool initializeSensors() {
  Serial.println(">>> Inicializando sensores...");
  
  bool ahtSuccess = aht.begin();
  bool bmpSuccess = bmp.begin(); //No se usa dirección porque es un integrado con ambos sensores conectado directamente al Pin 21 y 22 I2C del ESP32.
  
  Serial.printf("   AHT20: %s\n", ahtSuccess ? "OK" : "FALLO");
  Serial.printf("   BMP280: %s\n", bmpSuccess ? "OK" : "FALLO");
  
  // Configurar BMP280
  if (bmpSuccess) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                   Adafruit_BMP280::SAMPLING_X2,
                   Adafruit_BMP280::SAMPLING_X16,
                   Adafruit_BMP280::FILTER_X16,
                   Adafruit_BMP280::STANDBY_MS_500);
  }
  
  return ahtSuccess;
}

// ---------- CALIBRACIÓN DE LÍNEA BASE ----------
//Esta función calibra la temperatura base del entorno para mejorar la detección de cambios asociados a la respiración.
void calibrateBaseline() {
  Serial.println(">>> Calibrando temperatura base...");
  setNeoPixelColor(255, 165, 0); // Naranja durante calibración
  
  for (int i = 0; i < TEMP_SAMPLES; i++) {
    sensors_event_t humidity, temperature;
    if (aht.getEvent(&humidity, &temperature)) {
      updateTemperatureHistory(temperature.temperature);
      Serial.printf("   Muestra %d: %.2f°C\n", i + 1, temperature.temperature);
    }
    delay(300);
  }
  
  baselineTemp = getTempAverage();
  baselineReady = true;
  Serial.printf(">>> Linea base establecida: %.2f°C\n", baselineTemp);
}

// ---------- MANEJO DE COMANDOS UART ----------
void handleUARTCommand(uint8_t cmd) {
  Serial.printf(">>> Comando recibido: 0x%02X (%d)\n", cmd, cmd);
  
  uint8_t ack = 0xAA;
  Serial2.write(ack);
  Serial.println(">>> ACK enviado al STM32");
  
  switch (cmd) {
    case 1: // Medir respiración
      if (currentState == STATE_IDLE) {
        Serial.println(">>> INICIANDO MEDICION DE RESPIRACION");
        currentState = STATE_MEASURING;
        measurementStartTime = millis();
        setNeoPixelColor(0, 255, 255); // Azul para medición
      } else {
        Serial.println(">>> Sistema ocupado, espere...");
      }
      break;
      
    case 2: // Guardar datos
      Serial.println(">>> COMANDO: Guardar datos en SD");
      setNeoPixelColor(0, 255, 255);
      delay(1000);
      if (currentState == STATE_IDLE) {
        setNeoPixelColor(0, 0, 255);
      }
      break;

    default:
      Serial.printf(">>> Comando desconocido: 0x%02X\n", cmd);
      break;
  }
}


// ---------- LECTURA DE SENSORES ----------
void readSensors() {
  sensors_event_t humidity, temperature;
  
  if (aht.getEvent(&humidity, &temperature)) {
    currentTemp = temperature.temperature;
    currentHum = humidity.relative_humidity;
    
    // Actualizar historia de temperatura si estamos en modo medición
    if (currentState == STATE_MEASURING) {
      detectBreathing(currentTemp);
      updateTemperatureHistory(currentTemp);
    }
  }
  
  // Leer presión del BMP280
  if (bmp.begin()) {
    currentPressure = bmp.readPressure() / 100.0f; // Convertir a hPa para consistencia.
  }
}

// ---------- PROCESAMIENTO DE MEDICIÓN ----------
//Esta función procesa la medición de respiración, detectando cambios en la temperatura para identificar inhalaciones y exhalaciones.
void processMeasurement() {
  unsigned long now = millis();
  
  // Verificar timeout
  if (now - measurementStartTime > MEASUREMENT_TIMEOUT) {
    Serial.println(">>> TIMEOUT: No se completo la medición");
    currentState = STATE_IDLE;
    setNeoPixelColor(255, 0, 0); //Rojo para error
    return;
  }
  
  // Por ahora, tomamos una medición después de 3 segundos
  if (now - measurementStartTime > 3000) {
    Serial.println(">>> MEDICION COMPLETADA - Enviando datos...");
    sendDataToSTM32();
    currentState = STATE_IDLE;
    setNeoPixelColor(0, 255, 0); // Verde para listo
  }
}

// ---------- ENVÍO DE DATOS A STM32 ----------
//Esta función envía los datos de temperatura, humedad y presión al STM32 en un formato específico.
//Actualmente si manda la información correctamente, pero el STM32 no la está interpretando bien.
void sendDataToSTM32() {
  Serial.println(">>> Tomando medicion final...");
  
  // Tomar múltiples lecturas para promediar
  float tempSum = 0, humSum = 0, presSum = 0;
  int samples = 3;
  
  for (int i = 0; i < samples; i++) { // Tomar 3 muestras rápidas para promediar
    sensors_event_t h, t;
    if (aht.getEvent(&h, &t)) {
      tempSum += t.temperature;
      humSum += h.relative_humidity;
    }
    if (bmp.begin()) {
      presSum += bmp.readPressure() / 100.0f;
    }
    delay(500);
  }
  
  // Calcular promedios para estabilidad
  currentTemp = tempSum / samples;
  currentHum = humSum / samples;
  currentPressure = presSum / samples;
  
  Serial.printf(">>> Datos finales: T=%.2f C | H=%.2f%% | P=%.2f hPa\n", 
               currentTemp, currentHum, currentPressure);
  
  // Crear paquete
  uint8_t packet[6];
  packet[0] = 0xAA; // Header
  packet[1] = (uint8_t)(currentTemp * 10);    // Ej. Temp: 23.7°C → 237
  packet[2] = (uint8_t)(currentHum);          // Ej. Humedad: 90.6% → 90
  uint16_t pressureInt = (uint16_t)(currentPressure * 10); // Ej. 818.2 → 8182
  packet[3] = pressureInt >> 8;
  packet[4] = pressureInt & 0xFF;
  packet[5] = 0x55; // Footer

  
  for (int attempt = 0; attempt < 3; attempt++) {
    Serial.printf(">>> Intento de envio %d/3\n", attempt + 1);
    Serial2.write(packet, 6);
    Serial2.flush(); 
    
    Serial.print(">>> Paquete enviado: ");
    for (int i = 0; i < 6; i++) {
      Serial.printf("0x%02X ", packet[i]);
    }
    Serial.println();
    
    delay(100); // Esperar entre intentos
  }
  
  Serial.println(">>> Datos enviados correctamente a STM32");
}

//Esta fuynción actualiza el historial de temperatura con la última lectura para mejorar la detección de cambios asociados a la respiración.
void updateTemperatureHistory(float temp) {
  tempHistory[tempIndex] = temp;
  tempIndex = (tempIndex + 1) % TEMP_SAMPLES;
}

float getTempAverage() {
  float sum = 0;
  for (int i = 0; i < TEMP_SAMPLES; i++) {
    sum += tempHistory[i];
  }
  return sum / TEMP_SAMPLES;
}

//Esta función controla el color del NeoPixel según los estados del sistema.
void setNeoPixelColor(uint8_t r, uint8_t g, uint8_t b) {
  strip.fill(strip.Color(r, g, b));
  strip.show();
}

void systemStatusLED() {
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  unsigned long now = millis();
  
  if (currentState == STATE_ERROR) {
    // Parpadeo rojo rápido para error
    if (now - lastBlink > 300) {
      ledState = !ledState;
      setNeoPixelColor(ledState ? 255 : 0, 0, 0);
      lastBlink = now;
    }
  } else if (currentState == STATE_MEASURING) {
    // Respiración azul durante medición
    if (now - lastBlink > 100) {
      static uint8_t brightness = 0;
      static bool increasing = true;
      
      if (increasing) {                               //Este módulo de aumenta y disminuye el brillo del LED para simular una respiración visual. Funciona incrementando o decrementando el valor de brillo en pasos, y cambia la dirección cuando alcanza los límites establecidos.
        brightness += 5;
        if (brightness >= 100) increasing = false;
      } else {
        brightness -= 5;
        if (brightness <= 20) increasing = true;
      }
      
      setNeoPixelColor(0, 0, brightness);
      lastBlink = now;
    }
  }
  if (currentState == STATE_IDLE) {
    // Verde fijo para listo
    setNeoPixelColor(0, 255, 0);
  }
}

//Esta función detecta cambios en la temperatura para identificar inhalaciones y exhalaciones basadas en umbrales definidos.
void detectBreathing(float temp) {

  static bool first = true;
  static float last = 0;

  if (first) {
    last = temp;
    first = false;
    return;
  }

  float delta = temp - last;

  Serial.printf("Temp=%.2f  Cambio=%.3f\n", temp, delta);

  const float threshold = 0.05; 

  if (delta > threshold) {
    // EXHALANDO
    if (breathState != BREATH_EXHALE) {
      breathState = BREATH_EXHALE;
      Serial.println(">>> EXHALANDO");
      breathBrightness = 0;
      breathIncreasing = true;
    }
  }
  else if (delta < -threshold) {
    // INHALANDO
    if (breathState != BREATH_INHALE) {
      breathState = BREATH_INHALE;
      Serial.println(">>> INHALANDO");
      breathBrightness = 0;
      breathIncreasing = true;
    }
  }

  last = temp;
}

//Esta función anima el LED NeoPixel para simular una respiración visual, ajustando el brillo según el estado de inhalación o exhalación.
void animateBreathing() {
  unsigned long now = millis();
  
  // velocidad del fade (ms por paso)
  const int animSpeed = 150;

  if (now - lastBreathAnim < animSpeed) return;
  lastBreathAnim = now;

  // controlar brillo
  if (breathIncreasing) {
    breathBrightness += 3;
    if (breathBrightness >= 200) breathIncreasing = false;
  } else {
    breathBrightness -= 3;
    if (breathBrightness <= 30) breathIncreasing = true;
  }

  // colores según estado
  if (breathState == BREATH_INHALE) {
    strip.fill(strip.Color(breathBrightness, 0, 0));     // rojo
  } 
  else if (breathState == BREATH_EXHALE) {
    strip.fill(strip.Color(0, 0, breathBrightness));     // azul
  }

  strip.show();
}

