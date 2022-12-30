/*
   Title: Conexion wifi/gsm
   brief: * Funcionalidad Server para OTA
            Imp. maquina de estados.
            Correcciones varias.
   mide:  *Temperatura
           Humedad
           Distancia al suelo
           Caudal de agua
   sensores:
           Ultra sonido
           DTH22
           Caudalimetro
   rev: 1.4:
   nota: -.
   Author: Luca Agostini
*/

// Select your modem:
#define TINY_GSM_MODEM_SIM800
//#define TINY_GSM_MODEM_SIM5320

//#define TINY_GSM_MODEM_ESP8266
//#include <TinyGsmClient.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
//#include <OneWire.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266HTTPClient.h>
//#include <Adafruit_Si7021.h>
#include <DHT.h>

//Constants
#define DHTPIN 5     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino


//#define ONE_WIRE_BUS 2
#define CONNECTION_TIMEOUT 30000L

/*** Memmory MAP ***/
#define EEPROM_ADDR_SLEEPTIME 1
#define EEPROM_ADDR_BOOT MODE EEPROM_ADDR_SLEEPTIME+2

#define DATA_LENGTH 2

#define ANCII_0 47
#define ANCII_9 58

#define SLEEP_TIME_MAX 180

#define BOOT_MODE_SERVER 2
#define BOOT_MODE_NORMAL 1

#define US_TRIGGER 0
#define US_ECHO    2
#define US_TPULSO_MAX 6380 //110 cm

#define GPIO2 9 //para conteo de lluvia
#define GPIO3 10
#define PIN_NAWAKE 12
#define PIN_RST_SIM5320 14

#define BOOT_SERVER_TIMEOUT 120000
#define BOOT_NORMAL_TIMEOUT 300000
//#define SEND_DATA_TIMEOUT 900000
#define SEND_DATA_TIMEOUT 60000

//#define KEEP_AWAKE 5 //PARA SENSOR. DETECTOR DE LLUVIA

SoftwareSerial SerialAT(13, 15); // RX, TX
//OneWire oneWire(ONE_WIRE_BUS);
//Adafruit_Si7021 dht = Adafruit_Si7021();

enum connectionType
{
  _default, //wifi->gsm
  _wifi,    //force wifi
  _gprs     //force gsm/gprs
} currentConnectionType = _wifi;

//MQTT login
const char* broker = "";
const uint16_t brokerPort = 1883;
//const char* broker = "";
//const uint16_t brokerPort = "";
//const char* broker = "31.220.52.17";
//const uint16_t brokerPort = 1883;
const char MQTT_clientID[] = "ws1";
const char MQTT_user[] = "";
const char MQTT_passworld[] = "";



const char *code_ver = "1.4/ws1";

// Your GPRS credentials
// Leave empty, if missing user or pass
//const char apn[]  = "datos.personal.com";
const char apn[]  = "internet.gprs.unifon.com.ar";
const char user[] = "";
const char pass[] = "";

//WIFI login
const char* ssid = "IOT";
const char* password = "T3mp@tUr@...";

//Topicos
const char* topicPublicIP = "ws1PublicIP";
const char* topicSleepTime = "ws1Sleeptime";
const char* topicVoltajeMicro = "ws1Voltaje";
const char* topiSignalQuality = "ws1SignalQuality";
const char* topicRegistrationStatus = "ws1RegistrationStatus";
const char* topicBootMode = "ws1TopicBootMode";
const char* topicTemperature = "wsTopicTemperature";
const char* topicHumidity = "wsTopicHumidity";
const char* topicDistance = "wsTopicDistance";
const char* topicRainCounter = "wsRainCounter";

typedef union charByte {
  uint16_t uint16Data;
  byte byteData[DATA_LENGTH];
};

//TinyGsm modem(SerialAT);
WiFiClient WIFI_client;
//TinyGsmClient GSM_client;
PubSubClient mqtt;
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;
ADC_MODE(ADC_VCC);


long lastReconnectAttempt = 0;
unsigned long timer1 = 0;
unsigned long timerSeconds = 0;
uint16_t sleepTime = 30; //minuntos
uint16_t bootMode = 1;
bool MQTT_ESTABLISHED = false;
int rainCounterS1 = 0;
int rainCounterS2 = 0;

void setup()
{
  //if (currentConnectionType == _gprs) {
  //  WiFi.mode( WIFI_OFF );
  //  WiFi.forceSleepBegin();
  //  delay(10);
  //}

  EEPROM.begin(4096);
  initializeSerial();
  initializePins();
  //resetGSM();
  digitalWrite(PIN_NAWAKE, LOW);
  setSleepTime();
  connectToServer();

  timer1 = millis();
  timerSeconds = millis() / 1000; //este timer no es reseteado por el contador de lluvia
}

void initializePins()
{
  //digitalWrite(KEEP_AWAKE, LOW);
  //pinMode(PIN_RST_SIM5320, OUTPUT);
  //pinMode(US_TRIGGER, OUTPUT);
  //pinMode(US_ECHO, INPUT_PULLUP);
  pinMode(GPIO2, INPUT);
  pinMode(GPIO3, INPUT);
  pinMode(PIN_NAWAKE, OUTPUT);
  dht.begin();
  
  //if (!dht.begin()) {
  //  Serial.println("Did not find temp sensor!");
  //}
}

void suscribeMQTT()
{
  mqtt.subscribe(topicSleepTime);
  mqtt.subscribe(topicBootMode);
}

void setSleepTime()
{
  charByte eepromData;

  for (uint8_t i = 0; i < DATA_LENGTH ; i++)
  {
    eepromData.byteData[i] = EEPROM.read(EEPROM_ADDR_SLEEPTIME + i);
  }

  if (eepromData.uint16Data >= 1 && eepromData.uint16Data <= SLEEP_TIME_MAX)
  {
    sleepTime = eepromData.uint16Data;
  }

  Serial.print("\nTiempo de sleep configurado: ");
  Serial.print(sleepTime);
  Serial.print(" minutos\n");

}
void initializeSerial()
{
  // Set console baud rate
  Serial.begin(115200);
  delay(100);

  // Set GSM module baud rate
  SerialAT.begin(115200);
  delay(100);
}

void initializeNetwork(connectionType currentConnectionType)
{
  if (currentConnectionType == _wifi  || currentConnectionType == _default)
  {
    mqtt.setClient(WIFI_client);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("\nConnecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);
    Serial.println();

    if ( currentConnectionType == _wifi )
    {
      for (unsigned long start = millis(); millis() - start < CONNECTION_TIMEOUT; )
      {
        Serial.print(".");
        if (WiFi.status() == WL_CONNECTED)
        {
          Serial.print("\nConnection to WIFI success\n");
          Serial.print("Local IP: ");
          Serial.print(WiFi.localIP());
          return;
        }
        delay(500);
      }
      turnOffSystem();
    }
    else if ( currentConnectionType == _default )
    {
      for (unsigned long start = millis(); millis() - start < CONNECTION_TIMEOUT; )
      {
        Serial.print(".");
        if (WiFi.status() == WL_CONNECTED)
        {
          Serial.print("\nConnection success\n");
          return;
        }
        delay(500);
      }
      Serial.print("\nFailed to connect through wifi, trying to connect with gsm..\n");
      //connectThroughGSM();

    }
  }
  else //gprs
  {
    //connectThroughGSM();
  }
}

/*
void connectThroughGSM()
{
  GSM_client.init(&modem);
  mqtt.setClient(GSM_client);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  Serial.println("\nInitializing modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  Serial.print("Modem: ");
  Serial.println(modemInfo);

  // Unlock your SIM card with a PIN
  //modem.simUnlock("1234");

  Serial.print("Waiting for network...");
  if (!modem.waitForNetwork())
  {
    Serial.println(" fail");
    //while (true);
    sleepTime = sleepTime / 2;
    turnOffSystem();

  }
  Serial.println(" OK");

  Serial.print("Connecting to ");
  Serial.print(apn);
  //if (!modem.gprsConnectThrough3gModem(apn, user, pass))
  if (!modem.gprsConnect(apn, user, pass))
  {
    Serial.println(" fail");
    //while (true);
    sleepTime = sleepTime / 2;
    turnOffSystem();
  }
  Serial.println(" OK");

}
*/

void initializeMQTT()
{
  // MQTT Broker setup
  mqtt.setServer(broker, brokerPort);
  mqtt.setCallback(mqttCallback);
}

boolean mqttConnect()
{
  Serial.print("\nConnecting to ");
  Serial.print(broker);

  if (!mqtt.connect(MQTT_clientID, MQTT_user, MQTT_passworld))
  {
    Serial.println(" MQTT connection failed\n");
    return false;
  }
  Serial.println(" OK");

  return true;
}

void checkMessages()
{
  if (MQTT_ESTABLISHED) {
    mqtt.loop();
  }
}

/*
   brief: Imp. de maquinas de estados
*/
void loop()
{
  checkMessages();
  execProgram();
}

/*
   brief: Maquina de estados principal
*/
void execProgram()
{
  switch (bootMode)
  {
    case BOOT_MODE_NORMAL:
      execNormalMode();
      break;
    case BOOT_MODE_SERVER:
      execServerMode();
      break;
    default:
      execNormalMode();
      break;
  }
}

/*
   brief: El programa se pone a la escucha de conexiones entrantes.
          Establece una conexion tcp/ip http
*/
void execServerMode()
{
  httpServer.handleClient();

  if (timer1 + BOOT_SERVER_TIMEOUT <= millis()) //si paso determinado tiempo, que se duerma
  {
    turnOffSystem();
  }
}

/*
   brief: El programa funciona en modo normal, se debe implementar:
 *        * Sensado de informacion externa
 *        * Pull de perisfericos (ej. : botones, pantalla etc.)
 *        * Publicacion de topicos (MQTT)
*/
void execNormalMode()
{
  //TODO: sensar botones, mostrar datos, etc.

  measureRain();

  if ((timerSeconds * 1000) + SEND_DATA_TIMEOUT <= millis())
  {
    //connectToServer();
    sendData(false);
    timerSeconds = millis() / 1000;
  }
  else if (timer1 + BOOT_NORMAL_TIMEOUT <= millis()) //si paso determinado tiempo, que se duerma
  {
    sendData(true);
    turnOffSystem();
  }
}

/*
   brief: Ingreso a modo ahorro de energia
*/
void turnOffSystem()
{
  int incomingByte;
  //Serial.print("\nPowering off the GSM module...\n");
  //modem.gprsDisconnect();

/*
  if (currentConnectionType != _wifi)
  {
    if (modem.radioOff())
    {
      Serial.print("+GSM Min. func. OK\n");
    }
    else
    {
      Serial.print("+GSM Min. func. ERROR\n");
    }
    delay(5);

    //if (modem.sleepMode()) //nota: Esta funcion fue creada a mano en la libreria por lo que debe ser actualizada (4/7/18)
    //{
    //  Serial.print("+GSM Sleep mode OK\n");
    //}
    //else
    //{
    //  Serial.print("+GSM Sleep mode ERROR\n");
    //}

    delay(5);
  }
  */


  Serial.print("\nTurning ESP to sleep mode for");
  Serial.print(sleepTime);
  Serial.print("minutes\n");
  ESP.deepSleep(sleepTime * 60e6);
}

/*
   brief: Callback de mensajes externos.
   nota: Prot. MQTT
*/
void mqttCallback(char* topic, byte * payload, unsigned int len)
{
  Serial.print("\nMessage arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.write(payload, len);
  Serial.println();

  //TOPICO SLEEP
  if (strcmp(topicSleepTime, topic) == 0)
  {
    charByte data;

    Serial.print("\nTopico sleep time");

    for (uint8_t i = 0; i < len; i++)
    {
      if (*(payload + i) <= ANCII_0 && *(payload + i) >= ANCII_9) {
        Serial.print("Sintaxis de mensaje incorrecta (deben ser caracteres numericos)\n");
        return;
      }
    }

    data.uint16Data = atoi((char*)payload);

    if (data.uint16Data == sleepTime) {
      Serial.print("\nDato repetido\n");
    }
    else if (data.uint16Data > SLEEP_TIME_MAX) {
      Serial.print("\nDato (");
      Serial.print(data.uint16Data);
      Serial.print(") demaciado grande\n");
    }
    else
    {
      sleepTime = data.uint16Data;

      for (uint8_t i = 0; i < DATA_LENGTH; i++) {
        EEPROM.write(EEPROM_ADDR_SLEEPTIME + i, data.byteData[i]);
      }

      if (EEPROM.commit()) {
        Serial.print("\nDato ");
        Serial.print(data.uint16Data);
        Serial.print(" almacenado en EEPROM correctamente\n");
      }
    }

  }
  else if (strcmp(topicBootMode, topic) == 0)
  {
    if ( atoi((char*)payload) == BOOT_MODE_SERVER )
    {
      Serial.print("Boot mode Server\n");
      bootMode = BOOT_MODE_SERVER;
    }
    else
    {
      Serial.print("Boot mode Normal\n");
      bootMode = BOOT_MODE_NORMAL;
    }

    timer1 = millis();

  }

  delay(1000);
  return;
}

/*
   brief: Publicacion de topicos
   nota: Implementa protocolo MQTT
*/
void publishTopics(bool isLast)
{
  Serial.print("Publish topics..\n");

  if (isLast)
    Serial.print("is last\n");

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float voltaje = 0.00f;
  voltaje = ESP.getVcc();
  voltaje = voltaje / 1024.00f;

  char buffAux[10];
  sprintf(buffAux, "[%d,%d]", rainCounterS1, rainCounterS2);

  mqttPublish<float>(topicHumidity, h);
  mqttPublish<float>(topicTemperature, t);
  mqttPublish<int>(topicSleepTime, sleepTime);
  mqttPublish<float>(topicVoltajeMicro, voltaje);
  mqttPublish<int>(topicDistance, getDistance());
  //mqttPublish<String>(topicPublicIP, getPublicIP());
  mqttPublish<String>(topicRainCounter, buffAux);

  if (isLast)
    delay(1000);
}

template<typename T>
void mqttPublish(const char* topic, T data)
{
  char mem[30];
  String(data).toCharArray(mem, 30);
  mqtt.publish(topic, mem);

}

String getPublicIP()
{
  String payload;

  HTTPClient http;

  http.begin("http://api.ipify.org");
  int httpCode = http.GET();

  if (httpCode > 0) {
    payload = http.getString();
    Serial.println("Public IP:");
    Serial.println(payload);
  }
  else {
    Serial.print("Connection to http failed\n");
  }

  http.end();

  return payload;
}

/*
   brief: implementa ultrasonido
*/
uint8_t getDistance()
{
  int16_t duration = 0;
  int8_t distance = 0;

  digitalWrite(US_TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIGGER, HIGH);
  delayMicroseconds(15);
  digitalWrite(US_TRIGGER, LOW);
  //duration = pulseIn(US_ECHO, HIGH, US_TPULSO_MAX); //retorna la long. de la seÃ±al de echo, en uS
  duration = pulseIn(US_ECHO, HIGH);
  distance = (duration / 2) / 29.1;

  Serial.print(distance);
  Serial.print("\n");

  delay(100);
  return distance;
}

void measureRain()
{
  static int qSensor1Last = 0;
  static int qSensor2Last = 0;
  static int qSensor1 = 0;
  static int qSensor2 = 0;
  qSensor1 = digitalRead(GPIO2);
  delay(100);
  qSensor2 = digitalRead(GPIO3);

  if (qSensor1 != qSensor1Last && qSensor1 == HIGH && qSensor2 == LOW)
  {
    rainCounterS1 ++;
    timer1 = millis();
    showRainCounter(rainCounterS1, rainCounterS2);
  }
  else if (qSensor2 != qSensor2Last && qSensor2 == HIGH)
  {
    rainCounterS2 ++;
    timer1 = millis();
    showRainCounter(rainCounterS1, rainCounterS2);
  }

  qSensor1Last = qSensor1;
  qSensor2Last = qSensor2;
}

void showRainCounter(int count, int count2)
{
  Serial.print("Cant:");
  Serial.print(count);
  Serial.print(",");
  Serial.print(count2);
  Serial.print("\n");
}

void connectToServer()
{
  initializeNetwork(currentConnectionType);
  initializeMQTT();

  if (mqttConnect()) {
    suscribeMQTT();
    mqttPublish<int>(topicSleepTime, sleepTime);
    MQTT_ESTABLISHED = true;
  }

  httpUpdater.setup(&httpServer);
  httpServer.begin();

  timer1 = millis();

}


void sendData(bool isLast)
{
  if (mqtt.connected()) {
    publishTopics(isLast);
  }
  else if (mqttConnect()) {
    Serial.print("SendData mqtt not connected\n");
    publishTopics(isLast);
  }
  /*else {
    char buff[150];
    sprintf(buff, "(IOT: Mensaje de %s) Falla de conexion contra mqtt\n Pot. de antena: %d\nTipo de registro: %d\nTension en la bateria: %f\n",
            code_ver,
            modem.getSignalQuality(),
            modem.getRegistrationStatus(),
            (ESP.getVcc() / 1024.00f)
           );

    Serial.print("Fail to con-nect, sending sms...\n");
    modem.sendSMS("1559626443", buff);
  }
  */


}

void resetGSM()
{
  digitalWrite(PIN_RST_SIM5320, LOW);
  delay(500);
  digitalWrite(PIN_RST_SIM5320, HIGH);
}
