#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_INA219.h>
#include <Heltec.h>
#include <Adafruit_MPU6050.h>
#include <math.h>

// Configurações LoRa
#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             20        // dBm
#define LORA_BANDWIDTH                              2         // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR                       11         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH                        16         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define RX_TIMEOUT_VALUE                            2000
#define BUFFER_SIZE                                 80 // Tamanho do payload LoRa

// Configurações do display OLED
#define OLED_RST 21 // Pino RST do OLED

// Pinos dos Sensores
#define BOIA_PIN 26
#define YFS201_PIN 7
#define DIAMETRO_TUBO 0.011 // Metros
#define TRIG_PIN 45
#define ECHO_PIN 46

// Constantes do sensor YF-S201
#define PULSOS_POR_LITRO 450.0

// Instâncias dos Sensores
SSD1306Wire oleddisplay(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, OLED_RST);
Adafruit_AHTX0 aht10;
Adafruit_INA219 ina219;
Adafruit_MPU6050 mpu;

// Variáveis dos Sensores
bool sensoresAtivados = false;
unsigned long intervalo = 5000; // Intervalo para medições (5 segundos)
volatile int contagemPulsos = 0;
unsigned long tempoUltimoPulso = 0;
float fluxoLMin = 0.0;
long duracao, distancia;
float nivelAgua = 0.0;
float inclinacaoX, inclinacaoY;
float temp, humidity;
float velocidade;
float busvoltage, current_mA;

// Variáveis LoRa
char txpacket[BUFFER_SIZE];
int16_t txNumber;
bool lora_idle=true;
static RadioEvents_t RadioEvents;

// Funções LoRa
void OnTxDone( void );
void OnTxTimeout( void );

void VextON() {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF() {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH);
}

// Função para converter a vazão (L/min) para m³/s
float vazaoParaM3s(float vazaoLMin) {
  return vazaoLMin / 1000.0 / 60.0;
}

// Função para calcular a área da seção transversal do tubo (m²)
float calcularAreaTubo(float diametro) {
  return 3.14159 * (diametro / 2) * (diametro / 2);
}

// Função para calcular a velocidade do escoamento (m/s)
float calcularVelocidade(float vazaoLMin, float diametroTubo) {
  float vazaoM3s = vazaoParaM3s(vazaoLMin);   // Vazão em m³/s
  float areaTubo = calcularAreaTubo(diametroTubo);  // Área da seção transversal (m²)
  return vazaoM3s / areaTubo;  // Velocidade em m/s
}

// Função que é chamada sempre que um pulso é gerado pelo sensor YF-S201
void contagemDePulsos() {
  contagemPulsos++;
}

void setup() {
  Serial.begin(115200);
  VextON();
  delay(100);

  // Configurar pino RST do OLED
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(100);
  digitalWrite(OLED_RST, HIGH);
  delay(100);

  // Inicializar o display OLED
  oleddisplay.init();
  oleddisplay.clear();
  oleddisplay.drawString(0, 0, "Inicializando...");
  oleddisplay.display();
    
  // Configuração da boia
  pinMode(BOIA_PIN, INPUT_PULLUP); // Pull-up interno para estabilizar o sinal

  Wire1.begin(41, 42, 400000);
  delay(1000);

  // Inicialização dos sensores com tratamento de erros
  // Inicializa o sensor AHT10
  int tentativas = 0;
  while (!aht10.begin(&Wire1) && tentativas < 3) {
    Serial.println("Falha ao inicializar AHT10! Tentando novamente...");
    oleddisplay.clear();
    oleddisplay.drawString(0, 0, "Falha AHT10!");
    oleddisplay.drawString(0, 10, "Tentando...");
    oleddisplay.display();
    delay(1000);
    tentativas++;
  }
  if (tentativas >= 3) {
    Serial.println("Falha ao inicializar AHT10 após 3 tentativas!");
    oleddisplay.clear();
    oleddisplay.drawString(0, 0, "Falha AHT10!");
    oleddisplay.drawString(0, 10, "Erro!");
    oleddisplay.display();
    while (1);
  }

  // Inicializa o sensor INA219
  tentativas = 0;
  while (!ina219.begin(&Wire1) && tentativas < 3) {
    Serial.println("Falha ao inicializar INA219! Tentando novamente...");
    oleddisplay.clear();
    oleddisplay.drawString(0, 0, "Falha INA219!");
    oleddisplay.drawString(0, 10, "Tentando...");
    oleddisplay.display();
    delay(1000);
    tentativas++;
  }
  if (tentativas >= 3) {
    Serial.println("Falha ao inicializar INA219 após 3 tentativas!");
    oleddisplay.clear();
    oleddisplay.drawString(0, 0, "Falha INA219!");
    oleddisplay.drawString(0, 10, "Erro!");
    oleddisplay.display();
    while (1);
  }

  // Inicializa o sensor MPU-6050
  tentativas = 0;
  Serial.println("Inicializando MPU-6050...");
  while (!mpu.begin(0x68, &Wire1) && tentativas < 3) { // Endereço padrão do MPU-6050
    Serial.println("Falha ao inicializar o sensor MPU-6050! Tentando novamente...");
    oleddisplay.clear();
    oleddisplay.drawString(0, 0, "Falha MPU-6050!");
    oleddisplay.drawString(0, 10, "Tentando...");
    oleddisplay.display();
    delay(1000);
    tentativas++;
  }
  if (tentativas >= 3) {
    Serial.println("Falha ao inicializar o sensor MPU-6050 após 3 tentativas!");
    oleddisplay.clear();
    oleddisplay.drawString(0, 0, "Falha MPU-6050!");
    oleddisplay.drawString(0, 10, "Erro!");
    oleddisplay.display();
    while (1);
  }

  // Configuração do sensor
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Configura o pino do YF-S201 como entrada e ativa a interrupção
  pinMode(YFS201_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(YFS201_PIN), contagemDePulsos, RISING);  // Contagem de pulsos na borda de subida

  // Configura o pino do AJ-SR04M
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.println("Nó-sensor hidrológico iniciado...");
     
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  
  txNumber=0;

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
    
  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 ); 
}

void loop() {
  int estadoBoia = digitalRead(BOIA_PIN);

  // Se a boia estiver aberta (nível alto), desativar sensores
  if (estadoBoia == HIGH && sensoresAtivados) {
    sensoresAtivados = false;
    Serial.println("Boia aberta! Sensores desativados.");
    oleddisplay.clear();
    oleddisplay.drawString(0, 0, "Boia aberta!");
    oleddisplay.drawString(0, 10, "Sensores desativados.");
    oleddisplay.display();
    return;  // Interrompe o loop imediatamente, sem realizar medições
  }

  // Se a boia estiver fechada (nível baixo) e os sensores não estiverem ativados, ativá-los
  if (estadoBoia == LOW && !sensoresAtivados) {
    sensoresAtivados = true;
    Serial.println("Boia fechada! Sensores ativados.");
    oleddisplay.clear();
    oleddisplay.drawString(0, 0, "Boia fechada!");
    oleddisplay.drawString(0, 10, "Sensores ativados.");
    oleddisplay.display();
    delay(1000);
  }

  // Se os sensores estiverem ativados, realizar as medições
  if (sensoresAtivados) {
    // Medições de sensores a cada intervalo
    unsigned long tempoInicio = millis();
    while (millis() - tempoInicio < intervalo) {

      // Medição do MPU-6050
      // Obter os dados do acelerômetro e giroscópio usando o método getEvent
      sensors_event_t a, g, tempe;
      mpu.getEvent(&a, &g, &tempe);

      // Cálculo da inclinação em graus
      float inclinacaoX = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / M_PI;
      float inclinacaoY = atan2(a.acceleration.x, a.acceleration.z) * 180.0 / M_PI;

      // Formatação dos dados com classe String 
      String inclinacaoXStr = String(inclinacaoX, 2);
      String inclinacaoYStr = String(inclinacaoY, 2);
      String tempStr = String(tempe.temperature, 2); 

      // Imprime os valores de inclinação no monitor serial
      Serial.print("Inclinação X: ");
      Serial.print(inclinacaoX);
      Serial.println("°");

      Serial.print("Inclinação Y: ");
      Serial.print(inclinacaoY);
      Serial.println("°");

      Serial.print("Temp. MPU6050 (°C): ");
      Serial.println(tempe.temperature);

      // Exibir no display OLED
      oleddisplay.clear();
      oleddisplay.drawString(0, 0, "Incl: X=" + String(inclinacaoX, 1) + "° Y=" + String(inclinacaoY, 1) + "°");

      // Medição do AHT10 (Temperatura e umidade)
      sensors_event_t humidity, temp;
      aht10.getEvent(&humidity, &temp);  

      String humidityStr = String(humidity.relative_humidity, 2); 
      String tempAHTStr = String(temp.temperature, 2); 

      Serial.print("Temperatura: ");
      Serial.print(temp.temperature);
      Serial.println(" °C");
      Serial.print("Umidade: ");      
      Serial.print(humidity.relative_humidity);
      Serial.println(" %");

      // Exibir no display OLED
      oleddisplay.drawString(0, 10, "Temp: " + String(temp.temperature) + "C");
      oleddisplay.drawString(0, 20, "Umid: " + String(humidity.relative_humidity) + "%");

      // Calcular o fluxo de água
      if (millis() - tempoUltimoPulso >= 1000) {  // A cada 1 segundo
        fluxoLMin = (contagemPulsos / PULSOS_POR_LITRO) * 60.0; // Calcula vazão em L/min
        contagemPulsos = 0;  // Reseta o contador de pulsos
        tempoUltimoPulso = millis();  // Reseta o tempo do último pulso

        Serial.print("Vazão de água: ");
        Serial.print(fluxoLMin);
        Serial.println(" L/min");

        // Calcular a velocidade do escoamento
        float velocidade = calcularVelocidade(fluxoLMin, DIAMETRO_TUBO);
        float vazaoLsec = fluxoLMin / 60.0;  // Vazão em L/s

        Serial.print("Velocidade do escoamento: ");
        Serial.print(velocidade);
        Serial.println(" m/s");

        // Exibir no display OLED
        oleddisplay.drawString(0, 30, "Vel: " + String(velocidade) + " m/s, Q: " + String(vazaoLsec) + " L/s");
      }

      // Medição do AJ-SR04M (Distância)
      digitalWrite(TRIG_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);

      duracao = pulseIn(ECHO_PIN, HIGH);
      distancia = (duracao * 0.034 / 2); // Distância em centímetros. O nível d'água deve ser calculado pela altura do sensor até o fundo menos a distância do nível d'água medida pelo sensor.
      long altura = 600 - distancia; // Altura da lâmina d'água com o leito do rio 6 m abaixo do sensor. Substitua 600 pela distância até o fundo do córrego.

      Serial.print("Distância medida: ");
      Serial.print(distancia);
      Serial.println(" cm");
      Serial.print("Nível do rio: ");
      Serial.print(altura);
      Serial.println(" cm");

      // Exibir no display OLED
      oleddisplay.drawString(0, 40, "Nível: " + String(altura) + " cm");

      // Medição do INA219 (Tensão e corrente)
      float busvoltage = ina219.getBusVoltage_V();        // Tensão no barramento
      float current_mA = ina219.getCurrent_mA();          // Corrente em mA
      float shuntvoltage = ina219.getShuntVoltage_mV();   // Tensão no shunt

      Serial.print("Tensão no barramento: ");
      Serial.print(busvoltage);
      Serial.println(" V");
      Serial.print("Corrente: ");
      Serial.print(current_mA);
      Serial.println(" mA");
      Serial.print("Tensão no shunt: ");
      Serial.print(shuntvoltage);
      Serial.println(" mV");
      Serial.println("------------------------------");

      // Exibir no display OLED
      oleddisplay.drawString(0, 50, "U: " + String(busvoltage) + " V,  I: " + String(current_mA) + " mA");
      oleddisplay.display();
      delay(1000); // Aguarda 1 segundo 

      // Formatação dos dados com classes diferentes para envio
      int16_t velocidade_int = (int16_t)(velocidade * 100);
      String busvoltageStr = String(busvoltage, 2);
      String current_mAStr = String(current_mA, 2);

      // Preparação do pacote LoRa
      if (lora_idle == true) {
        delay(100);
        txNumber += 1;

        // Formatação do pacote com os dados dos sensores
        snprintf(txpacket, sizeof(txpacket), "%d,%s,%s,%s,%s,%d,%ld,%s,%s", 
                txNumber, inclinacaoXStr.c_str(), inclinacaoYStr.c_str(), 
                tempAHTStr.c_str(), humidityStr.c_str(), velocidade_int,
                altura, busvoltageStr.c_str(), current_mAStr.c_str());

        Serial.printf("\r\nEnviando pacote \"%s\" , tamanho %d\r\n",txpacket, strlen(txpacket));
        Serial.println("------------------------------");

        // Exibir informações no display
        oleddisplay.clear();
        oleddisplay.drawString(0, 20, "Enviando pacote: " + String(txpacket));
        oleddisplay.display();

        Radio.Send( (uint8_t *)txpacket, strlen(txpacket) ); // Envia o pacote
        lora_idle = false;
      }

      Radio.IrqProcess( );
      delay(1000); // Aguarda 1 segundo antes da próxima medição
    }
  }

  delay(500); // Intervalo de 500ms antes da próxima verificação
}

void OnTxDone( void ) {
  Serial.println("Envio completado!");
  Serial.println("------------------------------");
  lora_idle = true;
}

void OnTxTimeout( void ) {
    Radio.Sleep( );
    Serial.println("Tempo de envio excedido");
    oleddisplay.clear();
    oleddisplay.drawString(0, 0, "Tempo de envio excedido");
    oleddisplay.display();
    lora_idle = true;
}

