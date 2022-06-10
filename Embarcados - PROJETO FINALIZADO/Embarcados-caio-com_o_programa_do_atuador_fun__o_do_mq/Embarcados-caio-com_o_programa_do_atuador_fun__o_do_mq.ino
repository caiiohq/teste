#include <WiFi.h>
#include <Adafruit_ADS1X15.h> //Conversor ADS1115
#include <PubSubClient.h>
#include <Wire.h>
#include <stdlib.h>
#include <DHT.h>

Adafruit_ADS1115 ads; //Definição do ADS a ser utilizado

// Definicoes do sensor DHT11
#define DHTPIN 23 // o sensor dht11 foi conectado ao pino D23 - GPIO23 que está ligado o pino de dados do sensor
#define DHTTYPE DHT11//sensor em utilização: DHT11
DHT dht(DHTPIN,DHTTYPE);

//Definição dos outros sensores
#define   ADC_16BIT_MAX   65536
float ads_InputRange = 6.144f;
const float VOLT_STEP = (ads_InputRange * 2) / (ADC_16BIT_MAX - 1);
#define RL_MQ8 10000 // Valor da resistência de carga para o sensor MQ8
#define R_STANDARD_MQ8 20 // Valor de R0 para o sensor MQ8 padrão - antes: 17900
#define RL_MQ135 10000 // Valor da resistência de carga para o sensor MQ135 
#define R_STANDARD_MQ135 27338 // Valor de R0 para o sensor MQ135 padrão


//Definição dos topicos


#define BUZZER 4 //pino D4 para o buzzer

//Dados para conectar no Wi-Fi
const char* ssid = "1618";  
const char* password = "comchuveiro101";

const char* mqtt_server = "caiostefani.duckdns.org"; //aqui pode ser o IP do MQTT ou uma URL
const char* mqtt_username = "admin"; //Usuario criado no broker 
const char* mqtt_password = "trabalho.123"; //senha do usuario
const int mqtt_port = 1883; // define a porta

//inicializando as variaveis que irao ligar o o buzzer por meio dos valores de ppm dos sensores de gases
int BuzzerMQ2 = 0;
int BuzzerMQ135 = 0;


WiFiClient espClient;
PubSubClient client(espClient);

void init_wifi(){ //inicia a função do wifi
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  reconnect_wifi();
}

void init_mqtt(){
client.setServer(mqtt_server, mqtt_port);
client.setCallback(callback);
}

void reconnect_wifi(){ //Função para conectar o Wi-Fi
  if (WiFi.status() == WL_CONNECTED)
        return;
  
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {//Loop que mostra a tentativa de se conectar no Wi-Fi
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  if(String (topic) == "ESP32/BUZZERMQ2"){ //ESP32/BUZZER -> topico de saida
    Serial.print("PPM do MQ2: ");
    BuzzerMQ2 = messageTemp.toInt();
    Serial.print(BuzzerMQ2);
  }
  if(String (topic) == "ESP32/BUZZERMQ135"){
    Serial.print("PPM do MQ135: ");
    BuzzerMQ135 = messageTemp.toInt();
    Serial.print(BuzzerMQ135);
  }  
  Serial.println();
}

void reconnect_mqtt() { //Função para conectar o mqtt
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client", mqtt_username, mqtt_password)) {
      Serial.println("connected");
      client.subscribe("ESP32/BUZZERMQ2");
      client.subscribe("ESP32/BUZZERMQ135");
    } 
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(1000);
    }
  }
}

void verificaConexao (){
  if(!client.connected()){
    reconnect_mqtt();
  }
  reconnect_wifi();
}

//...................................................................................//

float leitura_temperatura(){ //qualquer coisa mudar para float
  float t = dht.readTemperature();
  float result;

  if(!(isnan(t))){
    result = t;
  }
  else
  result = 0;

  Serial.print(result);
  Serial.println(" ºC");
  return result;
}

//...................................................................................//

float leitura_umidade(){
  float h = dht.readHumidity();
  float result;

  if(!(isnan(h))){
    result = h;
  }else{
  result = 0;
}
Serial.print(result);
Serial.print("%");
return result;
}
//...................................................................................//
//função para o mq2//

int leitura_mq8(){
  float adc3 = ads.readADC_SingleEnded(3);
  float RS = RL_MQ8 * ((5 / (VOLT_STEP * adc3)) - 1);
  float ratio = Ajust_ratio_mq8((RS/R_STANDARD_MQ8));
  float PPM = pow(10, ((-log10(ratio) + 4.8965) / 1.59295));
  int result;

  if (! (isnan(PPM)) ) //Caso o valor não é um numero colocará uma valor diferente no resultado
        result = (int)PPM;
    else
        result = 0;

  Serial.print(result);
  Serial.println(" ppm H2");
  return result;
}

//...................................................................................//

//função para o mq135 para leitura do CO2
int leitura_mq135(){
  float adc1 = ads.readADC_SingleEnded(1);
  float RS = RL_MQ135 * ((5 / (VOLT_STEP * adc1)) - 1);
  float ratio = Ajust_ratio_mq135((RS/R_STANDARD_MQ135));
  float PPM = pow(10, ((-log10(ratio) + 0.79349) / 0.38306));
  int result;

  if (! (isnan(PPM)) ) //Caso o valor não é um numero colocará uma valor diferente no resultado
        result = (int)PPM;
    else
        result = 0;

  Serial.print(result);
  Serial.println(" ppm CO2");
  return result;
}

//...................................................................................//
//ratio do mq2
float Ajust_ratio_mq8(float r0_rs_ratio){
  float ajusted_ratio = 0; //Valor da razao ajustada pela temperatura e humidade
  float max_value_ratio = 0; //Valor de correcao da razao a 33% de umidade
  float min_value_ratio = 0; //Valor de correcao da razao a 85% de umidade
  int humidity_ar = dht.readHumidity(); //Leitura da umidade pelo sensor
  int temperature_ar = dht.readTemperature(); //Leitura da temperatura pelo sensor
  if(humidity_ar>85 || humidity_ar<33 || temperature_ar<0 ||temperature_ar>50)
  {
  return NAN;
  }
  delay(100);

  max_value_ratio = 0.00012 * pow(temperature_ar, 2) - 0.00656 * temperature_ar + 0.98828; //Curva para 33% de umidade
  min_value_ratio = 0.00014 * pow(temperature_ar, 2) - 0.00753 * temperature_ar + 1.04125; //Curva para 85% de umidade
  ajusted_ratio = r0_rs_ratio * (max_value_ratio + (min_value_ratio - max_value_ratio) *(humidity_ar - 33) / 52); //Interpolacao entre os valores maximos e minimos e a umidade
  return ajusted_ratio;
}

//...................................................................................//
//ratio mq135
float Ajust_ratio_mq135(float r0_rs_ratio){
  float ajusted_ratio = 0; //Valor da razao ajustada pela temperatura e humidade
  float max_value_ratio = 0; //Valor de correcao da razao a 33% de umidade
  float min_value_ratio = 0; //Valor de correcao da razao a 85% de umidade
  int humidity_ar = dht.readHumidity(); //Leitura da umidade pelo sensor
  int temperature_ar = dht.readTemperature(); //Leitura da temperatura pelo sensor
  if(humidity_ar>85 || humidity_ar<33 || temperature_ar<0 ||temperature_ar>50)
  {
  return NAN;
  }
  delay(100);

  max_value_ratio = -0.000005 * pow(temperature_ar, 3) + 0.000653 * pow(temperature_ar, 2)- 0.028508 * temperature_ar + 1.366829;//Curva para 33% de umidade
  min_value_ratio = -0.0000046 * pow(temperature_ar, 3) + 0.000573 * pow(temperature_ar, 2) - 0.025026 * temperature_ar + 1.2384997; //Curva para 85% de umidade
  ajusted_ratio = r0_rs_ratio * (max_value_ratio + (min_value_ratio - max_value_ratio) *(humidity_ar - 33) / 52); //Interpolacao entre os valores maximos e minimos e a umidade
  return ajusted_ratio;
}

//...................................................................................//

void setup(){
  Serial.begin(9600);
  ads.begin(); //Inicializa o ADS
  dht.begin(); //Inicializa o DHT
  init_wifi(); //Inicializa o Wi_Fi
  init_mqtt(); //Inicializa o MQTT
  pinMode(BUZZER,OUTPUT);
}

//...................................................................................//

void loop(){
  //declaração de variaveis locais
  char temperatura_str[10] = {0};
  char umidade_str[10]     = {0};
  char mq2_str[10]         = {0};
  char mq135_str[10]       = {0};
  double mq2 = leitura_mq8();
  double mq135 = leitura_mq135();
  
  verificaConexao();
  client.loop();

  //Preenchimento da string
  //sprintf(temperatura_str,"%.2f", leitura_temperatura());
  //sprintf(umidade_str, "%.2f", leitura_umidade());
    dtostrf(dht.readTemperature(), 1, 2,temperatura_str);
    dtostrf(dht.readHumidity(), 1, 2,umidade_str);
    dtostrf(mq2, 5, 2, mq2_str);//string para o mq2
    dtostrf(mq135, 5, 2, mq135_str);//string para o mq135
         
  //Envio dos dados atraves de MQTT
  client.publish("TOPICO_PUBLISH_TEMPERATURA", temperatura_str);
  client.publish("TOPICO_PUBLISH_UMIDADE", umidade_str);
  client.publish("TOPICO_PUBLISH_MQ2", mq2_str);//publish para o mq2
  client.publish("TOPICO_PUBLISH_MQ135", mq135_str);//publish para o mq2

  if (BuzzerMQ2 == 1 || BuzzerMQ135 == 1){
    digitalWrite(BUZZER,HIGH);
    Serial.print("LED LIGADO");
  }else{
    digitalWrite(BUZZER,LOW);
    Serial.print("LED DESLIGADO");
  }
  delay(1000);
}
