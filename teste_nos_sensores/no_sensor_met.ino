#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <queue> 
#include "FS.h" 
#include "SD.h" 
#include "SPI.h"
#include <WiFi.h>
#include <time.h> 

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
#define BUFFER_SIZE                                 80 

// Configurações do display OLED
#define OLED_RST 21 // Pino RST do OLED

// Pinos dos Sensores
#define PIN_PLUVIOMETRO 45
#define PIN_ANEMOMETRO 46
#define PIN_DHT22 47
#define SD_CS_PIN 34 // Pino CS do cartão SD 
SPIClass spi1; // Instância da classe SPI para o cartão SD

// Definições do DHT22
#define DHTTYPE DHT22
DHT dht(PIN_DHT22, DHTTYPE);

const char* ssid = "Rede"; // Substitua pelo seu SSID
const char* password = "39435100"; // Substitua pela sua senha

// Variáveis para data e hora
long timezone = -3; // Fuso horário de Brasília
byte daysavetime = 0; // Não temos horário de verão

SSD1306Wire oleddispay(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, OLED_RST);

unsigned long displayStartTime = 0;
bool displayActive = false;

char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;

bool lora_idle = true;

// Variáveis para os dados recebidos
int16_t txNumber;
float inclinacaoX, inclinacaoY, temp, humidity, velocidade;
long distancia;
float busvoltage, current_mA;

// Variáveis do pluviômetro
volatile uint16_t reedCount = 0;  // Contador de pulsos (modificado na interrupção)
bool novoPulsoChuva = false;      // Flag para indicar novos pulsos
float acumuladoChuva = 0.0;
float chuvaInstante = 0.0;

// Variáveis do anemômetro
volatile int counter = 0;
const float pi = 3.14159265;
unsigned long proximaMedicaoAnemometro = 0; // Tempo da próxima medição do anemômetro
const unsigned long intervaloMedicaoAnemometro = 120000; // 2 minutos em milissegundos
const int raio = 105; // Raio do anemômetro em mm
float velocidadeVento;

// Variáveis do do DHT22
float temperatura;
float umidade;

const int periodo = 5000; 

std::queue<String> sdWriteQueue;

bool conectarWifi() {
  int tentativas = 0;
  while (WiFi.status() != WL_CONNECTED && tentativas < 3) {
    WiFi.begin(ssid, password);
    Serial.print("Tentando conectar ao Wi-Fi...");
    delay(1000);
    tentativas++;
  }
  return WiFi.status() == WL_CONNECTED;
}

// Função de interrupção do pluviômetro
void IRAM_ATTR contarPulsoChuva() {
    reedCount = reedCount + 1;  
    novoPulsoChuva = true; // Apenas sinaliza que há novos pulsos
}

// Função de interrupção do anemômetro
void IRAM_ATTR contarPulsoVento() { 
    counter = counter + 1;
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Escrevendo o arquivo: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Falha em abrir o arquivo para escrita");
        return;
    }
    if(file.println(message)){
        Serial.println("Arquivo escrito");
    } else {
        Serial.println("Falha na escrita");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Adicionando dados ao arquivo: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Falha em abrir o arquivo para adição de dados");
        return;
    }
    if(file.println(message)){
        Serial.println("Dados adicionados");
    } else {
        Serial.println("Falha na adição");
    }
    file.close();
}

void VextON() {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF() {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH);
}

String formatarDataHora() {
    struct tm tmstruct;
    if (!getLocalTime(&tmstruct)) {
        Serial.println("Falha ao obter data e hora");
        return "Data e hora não disponíveis";
    }
    char formattedDateTime[30];
    strftime(formattedDateTime, sizeof(formattedDateTime), "%Y-%m-%d %H:%M:%S", &tmstruct);
    return String(formattedDateTime);
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi_correto, int8_t snr ) {

  // Desativar interrupções do pluviômetro a do anemômetro
  detachInterrupt(digitalPinToInterrupt(PIN_PLUVIOMETRO));
  detachInterrupt(digitalPinToInterrupt(PIN_ANEMOMETRO));

  memcpy(rxpacket, payload, size );
  rxpacket[size]='\0';
  Radio.Sleep( );

  int16_t velocidade_int;	

  // Processamento dos dados recebidos
  sscanf(rxpacket, "%hd,%f,%f,%f,%f,%hd,%ld,%f,%f", 
         &txNumber, &inclinacaoX, &inclinacaoY, &temp, &humidity, &velocidade_int, &distancia, &busvoltage, &current_mA);
  yield();
  // Converter velocidade para float antes de formatar a string
  if (velocidade_int != 0) { // Verifica se não é zero para evitar divisão por zero
    velocidade = (float)velocidade_int / 100.0; // Converte para float e divide por 100.0 (literal float)
  } else {
    velocidade = 0.0; // Ou outro valor padrão, se necessário
  }

  int altura = 600 - distancia; // Essa variável é para nível d'água no rio, que pode ser recebido diretamente do nó-sensor hidrológico. Foi enviada a distância medida, por isso o cálculo está nesse código.

  char linha[350]; // Ajuste o tamanho conforme necessário

  String dataHora = formatarDataHora();

  sprintf(linha, "%d,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%ld,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
          txNumber, dataHora.c_str(), inclinacaoX, inclinacaoY, temp, humidity, velocidade, distancia, 
          altura, busvoltage, current_mA, velocidadeVento, temperatura, umidade, chuvaInstante, acumuladoChuva);

  String dataToWrite = String(linha); // Converte char[] para String
  sdWriteQueue.push(dataToWrite);

  chuvaInstante = 0;

  Serial.printf("\r\nPacote recebido: %s, RSSI %d , Tamanho %u\r\n",rxpacket,rssi_correto,size);//tempSize);

  // Imprime os valores individuais
  Serial.println("------------------------------");
  Serial.print("txNumber: "); Serial.println(txNumber);
  Serial.print("Inclinação X: "); Serial.print(inclinacaoX); Serial.println(" º"); // Unidade de medida: graus (º)
  Serial.print("Inclinação Y: "); Serial.print(inclinacaoY); Serial.println(" º"); // Unidade de medida: graus (º)
  Serial.print("Temperatura Nó-sensor: "); Serial.print(temp); Serial.println(" ºC"); // Unidade de medida: graus Celsius (ºC)
  Serial.print("Umidade Nó-sensor: "); Serial.print(humidity); Serial.println(" %"); // Unidade de medida: porcentagem (%)
  Serial.print("Velocidade do escoamento: "); Serial.print(velocidade); Serial.println(" m/s"); // Unidade de medida: metros por segundo (m/s)
  Serial.print("Distância: "); Serial.print(distancia); Serial.println(" cm"); // Unidade de medida: centímetros (cm)
  Serial.print("Nível do rio: "); Serial.print(altura); Serial.println(" cm"); // Unidade de medida: centímetros (cm)
  Serial.print("Tensão: "); Serial.print(busvoltage); Serial.println(" V"); // Unidade de medida: Volts (V)
  Serial.print("Corrente: "); Serial.print(current_mA); Serial.println(" mA"); // Unidade de medida: miliamperes (mA)
  Serial.print("Velocidade do vento: "); Serial.print(velocidadeVento); Serial.println(" km/h"); // Unidade de medida: quilômetros por hora (km/h)
  Serial.print("Temperatura Gateway: "); Serial.print(temperatura); Serial.println(" ºC"); // Unidade de medida: graus Celsius (ºC)
  Serial.print("Umidade Gateway: "); Serial.print(umidade); Serial.println(" %"); // Unidade de medida: porcentagem (%)
  Serial.print("Chuva acumulada 24h: "); Serial.print(acumuladoChuva); Serial.println(" mm"); // Unidade de medida: milímetros (mm)

  oleddispay.clear();
  oleddispay.drawString(0, 20, "Pacote recebido: " + String(txNumber));
  oleddispay.drawString(0, 30, "Tamanho: " + String(size) + "bytes");
  oleddispay.drawString(0, 40, "RSSI: " + String(rssi_correto));
  oleddispay.display();

  displayStartTime = millis(); // Inicia o temporizador
  displayActive = true;

  // Reativar interrupções do pluviômetro e do anemômetro
  attachInterrupt(digitalPinToInterrupt(PIN_PLUVIOMETRO), contarPulsoChuva, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ANEMOMETRO), contarPulsoVento, RISING);

  lora_idle = true; // Importante: definir lora_idle como true após o recebimento
}

void setup() {
  Serial.begin(115200);
  VextON();
  delay(100);

  // Inicialização do display OLED
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(100);
  digitalWrite(OLED_RST, HIGH);
  delay(100);

  oleddispay.init();
  oleddispay.clear();
  oleddispay.drawString(0, 0, "Inicializando...");
  oleddispay.display(); 

  if (conectarWifi()) {
    Serial.println("Conectado ao Wi-Fi!");

    // Configura o horário via NTP
    configTime(3600*timezone, daysavetime*3600, "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org");

  } else {
    Serial.println("Falha na conexão Wi-Fi após 3 tentativas.");
    oleddispay.clear();
    oleddispay.drawString(0, 0, "Falha na conexão Wi-Fi.");
    oleddispay.display();
    delay(5000); // Aguarda 5 segundos antes de continuar
  }

  // Inicializar o cartão SD
  SPIClass(1);
  spi1.begin(36, 37, 35, 34);

  if(!SD.begin(SD_CS_PIN, spi1)){
    Serial.println("Falha na montagem do Cartão SD");
    oleddispay.clear();
    oleddispay.drawString(0, 0, "Falha na montagem do Cartão SD"); // Exibe mensagem de erro no display
    oleddispay.display();
  }

  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("Sem Cartão SD inserido");
    oleddispay.clear();
    oleddispay.drawString(0, 0, "Sem Cartão SD inserido"); // Exibe mensagem de erro no display
    oleddispay.display();
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("Tamanho do Cartão SD: %lluMB\n", cardSize);

  // Abrir o arquivo para escrita
	File file = SD.open("/dados.txt", FILE_APPEND); // Abre o arquivo em modo de adição

	if (!file) {
	  Serial.println("Erro ao abrir o arquivo /dados.txt");
	}

	// Verifica se o arquivo está vazio (se for a primeira vez que está sendo criado)
	if (file.size() == 0) {
	  file.println("txNumber,DataHora,InclinacaoX,InclinacaoY,TempNoSensor,UmidadeNoSensor,VelocidadeEscoamento,Distancia,NivelAgua,Tensao,Corrente,VelocidadeVento,TempGateway,UmidadeGateway,ChuvaInstante,ChuvaAcumulada"); // Escreve o cabeçalho apenas se o arquivo estiver vazio
	}

	file.close();

  dht.begin();
  pinMode(PIN_PLUVIOMETRO, INPUT_PULLUP);
  pinMode(PIN_ANEMOMETRO, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_PLUVIOMETRO), contarPulsoChuva, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ANEMOMETRO), contarPulsoVento, RISING);

  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);

  RadioEvents.RxDone = OnRxDone;
  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                             LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                             LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                             0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
}

void loop() {
  static unsigned long ultimaMedicao = 0;
  unsigned long tempoAtual = millis();

  if (!sdWriteQueue.empty()) {
    String dataToWrite = sdWriteQueue.front();
    appendFile(SD, "/dados.txt", dataToWrite.c_str()); // Converte String para char*
    sdWriteQueue.pop();
  }
    
  if (tempoAtual - ultimaMedicao >= periodo) {
    unsigned long tempovento = tempoAtual - ultimaMedicao;
    ultimaMedicao = tempoAtual;

    noInterrupts();  // Desativa interrupções para leitura segura
    int pulsosAnemometro = counter;
    counter = 0;     // Reinicia a contagem de pulsos
    interrupts();    // Reativa interrupções      

    // Cálculo da velocidade do vento
    int rpm = (pulsosAnemometro * 60) / (tempovento / 1000);
    velocidadeVento = (((4 * pi * raio * rpm) / 60) / 1000) * 3.6;
    counter = 0;
        
    // Leitura do DHT22
    temperatura = dht.readTemperature();
    umidade = dht.readHumidity();

    if (reedCount > 0) {  // Processa apenas se houver novos pulsos
      noInterrupts();                // Desativa interrupções temporariamente
      uint16_t pulsos = reedCount;    // Copia o valor da interrupção
      reedCount = 0;                  // Reseta a contagem
      novoPulsoChuva = false;         // Reseta a flag
      interrupts();                   // Reativa interrupções

      acumuladoChuva += pulsos * 0.25; // Soma ao total acumulado

      // Exibir dados no Serial Monitor
      Serial.println("------------------------------");
      Serial.print("Chuva medida: ");
      Serial.print(pulsos * 0.25);
      Serial.println(" mm");
      Serial.print("Velocidade do vento: ");
      Serial.print(velocidadeVento);
      Serial.println(" km/h");
      chuvaInstante = pulsos * 0.25;
    }
  }
    
  if(lora_idle) {
    lora_idle = false;
    Serial.println("\r\n------------------------------");
    Serial.println("Entrando em modo de recepção");
    Radio.Rx(0);
  }

  // Atualização do display
  if (displayActive && (millis() - displayStartTime >= 1800)) {
    oleddispay.clear();
    oleddispay.drawString(0, 0, "Incl: X=" + String(inclinacaoX, 1) + "° Y=" + String(inclinacaoY, 1) + "°");
    oleddispay.drawString(0, 10, "Temp: " + String(temp, 1) + "C,  Umid: " + String(humidity, 1) + "%");
    oleddispay.drawString(0, 20, "Dist: " + String(distancia) + "cm,  Vel: " + String(velocidade, 1) + " m/s");
    oleddispay.drawString(0, 30, "U: " + String(busvoltage) + " V,  I: " + String(current_mA) + " mA");
    oleddispay.drawString(0, 40, "V. vento: " + String(velocidadeVento) + " km/h");
    oleddispay.drawString(0, 50, "Chuva acum.: " + String(acumuladoChuva) + " mm");
    oleddispay.display();
    displayActive = false;
  }

  Radio.IrqProcess( );
}


