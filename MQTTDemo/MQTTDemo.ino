#include <SPI.h>
#include <WiFi.h>
#include <WifiIPStack.h>
#include <Countdown.h>
#include <MQTTClient.h>
#include "TM1637.h" 

// your network name also called SSID
char ssid[] = "energia123";
// your network password
char password[] = "launchpad123";

// IBM IoT Foundation Cloud Settings
char typeId[] = "iotsample-ti-energia";
//char pubtopic[] = "iot_2_evt_status_fmt_json";
char pubtopic[] = "iot_1_evt_status_fmt_json";
char deviceId[] = "000000000000";
char clientId[64];

//char mqttAddr[] = "iot.eclipse.org" ;  // "broker.mqttdashboard.com";
char mqttAddr[] = "broker.mqttdashboard.com";
int mqttPort = 1883;

MACAddress mac;


#define CLK               9          /* 4-Digit Display clock pin */
#define DIO               10          /* 4-Digit Display data pin */
#define LED               RED_LED     /* blink LED */
#define ROTARY_ANGLE_P    24          /* pin of rotary angle sensor */

/* Global Variables */
TM1637 tm1637(CLK, DIO);              /* 4-Digit Display object */
int analog_value = 0;                 /* variable to store the value coming from rotary angle sensor */
int blink_interval = 100;               /* LED delay time */
int8_t bits[4] = {0};                 /* array to store the single bits of the value */



// getTemp() function for cc3200
#ifdef TARGET_IS_CC3101
#include <Wire.h>
#include "Adafruit_TMP006.h"
Adafruit_TMP006 tmp006(0x41);
#endif
  
WifiIPStack ipstack;  
MQTT::Client<WifiIPStack, Countdown, 100> client(ipstack);

void setup() {
  uint8_t macOctets[6];
  
  Serial.begin(115200);
  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to Network named: ");
  // print the network name (SSID);
  Serial.println(ssid); 
  // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
  WiFi.begin(ssid, password);
  while ( WiFi.status() != WL_CONNECTED) {
    // print dots while we wait to connect
    Serial.print(".");
    delay(300);
  }
  
  Serial.println("\nYou're connected to the network");
  Serial.println("Waiting for an ip address");
  
  while (WiFi.localIP() == INADDR_NONE) {
    // print dots while we wait for an ip addresss
    Serial.print(".");
    delay(300);
  }

  // We are connected and have an IP address.
  Serial.print("\nIP Address obtained: ");
  Serial.println(WiFi.localIP());

  mac = WiFi.macAddress(macOctets);
  Serial.print("MAC Address: ");
  Serial.println(mac);
  
  // Use MAC Address as deviceId
  sprintf(deviceId, "%02x%02x%02x%02x%02x%02x", macOctets[0], macOctets[1], macOctets[2], macOctets[3], macOctets[4], macOctets[5]);
  Serial.print("deviceId: ");
  Serial.println(deviceId);

  sprintf(clientId, "d:TI:%s:%s", typeId, deviceId);
  
  
     /* Initialize 4-Digit Display */
    tm1637.init();
    tm1637.set(BRIGHT_TYPICAL);
  
    /* declare the LED pin as an OUTPUT */
    pinMode(LED, OUTPUT);

  #ifdef TARGET_IS_CC3101
  if (!tmp006.begin()) {
    Serial.println("No sensor found");
    while (1);
  }
  #endif
}

void loop() {

  int rc = -1;
  if (!client.isConnected()) {
    Serial.print("Connecting to ");
    Serial.print(mqttAddr);
    Serial.print(":");
    Serial.println(mqttPort);
    Serial.print("With client id: ");
    Serial.println(clientId);
    
    while (rc != 0) {
      rc = ipstack.connect(mqttAddr, mqttPort);
    }

    MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;
    connectData.MQTTVersion = 3;
    connectData.clientID.cstring = clientId;
    
    rc = -1;
    while ((rc = client.connect(connectData)) != 0)
      ;
    Serial.println("Connected\n");
  }



    analog_value = analogRead(ROTARY_ANGLE_P);      /* read the value from the sensor */
    
    memset(bits, 0, 4);                             /* reset array before we use it */
    for(int i = 3; i >= 0; i--) 
	{
        /* Convert the value to individual decimal digits for display */
        bits[i] = analog_value % 10;
        analog_value = analog_value / 10;  
        tm1637.display(i, bits[i]);                 /* display on 4-Digit Display */
    }
    
  
  char json[58] = "{\"myName\":\"TILaunchPad_Second\",\"temperature\":";

  //dtostrf(getTemp(),1,2, &json[45]);
  dtostrf(analog_value,4,0, &json[45]);
  json[50] = '}';
  json[51] = '\0';
  Serial.print("Publishing: ");
  Serial.println(json);
  MQTT::Message message;
  message.qos = MQTT::QOS0; 
  message.retained = false;
  message.payload = json; 
  message.payloadlen = strlen(json);
  rc = client.publish(pubtopic, message);
  if (rc != 0) {
    Serial.print("Message publish failed with return code : ");
    Serial.println(rc);
  }
  
  // Wait for one second before publishing again
  client.yield(1000);
}

// getTemp() function for MSP430F5529
#if defined(__MSP430F5529)
// Temperature Sensor Calibration-30 C
#define CALADC12_15V_30C  *((unsigned int *)0x1A1A)
// Temperature Sensor Calibration-85 C
#define CALADC12_15V_85C  *((unsigned int *)0x1A1C)

double getTemp() {
 return (float)(((long)analogRead(TEMPSENSOR) - CALADC12_15V_30C) * (85 - 30)) /
        (CALADC12_15V_85C - CALADC12_15V_30C) + 30.0f;
}

// getTemp() function for Stellaris and TivaC LaunchPad
#elif defined(TARGET_IS_SNOWFLAKE_RA0) || defined(TARGET_IS_BLIZZARD_RB1)

double getTemp() {
  return (float)(147.5 - ((75 * 3.3 * (long)analogRead(TEMPSENSOR)) / 4096));
}

// getTemp() function for cc3200
#elif defined(TARGET_IS_CC3101)
double getTemp() {
  return (double)tmp006.readObjTempC();
}
#else
double getTemp() {
  return 21.05;
}
#endif
