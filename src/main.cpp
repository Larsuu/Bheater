#include <Arduino.h>
#include <QuickPID.h>
#include <math.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <secrets.h>
#include <esp_task_wdt.h>
#include <string>
//#include <BluetoothSerial.h>

const char* ssid     = "Pihalla";
const char* password = "10209997"; 
const char* mqtt_server = "192.168.1.150";

IPAddress staticIP(192, 168, 1, 241);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 1, 150);  

// MQTT Subscrib/Timppa

#define AKKULAMPOTEHOT  "Battery/Timppa/tehot"
#define LAMPOKATKAISU   "Battery/Timppa/lamposw"
#define AKKU_BOOST      "Battery/Timppa/boost_sw"
#define LAMMTOT         "Battery/Timppa/lamm_tot"
#define UPTIMES         "Battery/Timppa/uptimes"
#define BOOSTSENSE      "Battery/Timppa/boostsens"
#define KP              "Battery/Timppa/kp"
#define AKKUJANNITE     "Battery/Timppa/jannite"
#define KI              "Battery/Timppa/ki"
#define LATURI          "Battery/Timppa/laturi"
#define ELOSSA          "Battery/Timppa/elossa"
#define AKKULAMPO       "Battery/Timppa/lampo"

// MCU/Measurement related settings 
#define V_REF 1100
#define WDT_TIMEOUT 60       


// Moving average variables for ADC readings on battery voltage
#define MOVING_AVG_SIZE 5
static float MOVAreadings[MOVING_AVG_SIZE] = { };
static uint8_t MOVAIndex = 0;
float MOVASum = 0;

// Final_paattotyo_jlcpcb
// define FUNKTION PORT // pin_name_footprint
#define HEAT_1        33  // IO33
#define HEAT_2        25  // IO25
#define TEMPs         21  // IO21
#define CHARGER       32  // IO32
#define INPUT_1       19  // IO19
#define INPUT_2       4   // IO4
#define IGNITION      18  // IO18
#define WALL_DETECT   36  // SENSOR_VP
#define BATT_MEAS     39  // SENSOR_VN
#define UART2_TX      17  // IO17
#define UART2_RX      16  // IO16
#define SDAs          26  // IO26
#define SCLs          27  // IO27

//quickpid muuttujat
bool  printOrPlotter = 0;  // on(1) monitor, off(0) plotter
float POn = 1.0;          // proportional on Error to Measurement ratio (0.0-1.0), default = 1.0
float DOn = 0.0;          // derivative on Error to Measurement ratio (0.0-1.0), default = 0.0
float Setpoint, Input, Output;
float Kp = 15, Ki = 0.001, Kd = 0;  // edellinen: P=6, I=0.0015 - toimii riittävän hyvin -> massan hitaus.  - ei kaavoja, liian pitkä aika.
float lammitys_tot;
float kosteus;
float lampo;
float ressuaika = 0;

// MQTT viestin muuttujat, jotka passataan Hassiolle.
float janniteviesti = 0;
float kosteus_viesti = 0;   
float lampo_viesti = 0;
float bttpwr_viesti = 0;
float heatsw_viesti = 0;
float boost_viesti = 0;
float lammtot_viesti = 0;
float millis_viesti = 0;
float bsense_viesti = 0;
float kp_viesti;
float ki_viesti;


// kirjastot

//BluetoothSerial SerialBT;
WiFiClient espClient;
PubSubClient client(espClient);
QuickPID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd,
               myPID.pMode::pOnError,
               myPID.dMode::dOnMeas,
               myPID.iAwMode::iAwClamp,
               myPID.Action::direct);

OneWire Wire(TEMPs);
DallasTemperature sensor1(&Wire);


// Nonblocking code with time checking.
// 
unsigned long lastmillis = 0;
unsigned long prevmillis = 0;
unsigned long lampoMillis = 0;
unsigned long mittausmillit = 0;
unsigned long startti_ms = 0;
unsigned long interval = 30000;
unsigned long full_millis = 0;
unsigned long reset_timer = 0;
unsigned long mqttimer = 0;
unsigned long lastMsg = 0;

// ohjelman määrityksiä nolliin. Alustusta.
float   NewSetpoint = 0;
int     LampoKatkaisu = 1;
int     akku_boost = 0;
float   millisek = 0;
int     akkuboostsensor = 0;
float   akku_lepojannite = 64;
float   laturi = 0;
float   akunjannite = 0;
int     lammitys80; 
float   elohiiri = 0;

//MQTT tallennuksia, aikavertailuja.

char lampo_msg[20];
char v_msg[20];
char bttpwr_msg[20];
char heatsw_msg[20];
char boost_msg[20];
char lammtotmsg[20];
char ms_msg[20];
char boostsens_msg[20];
char kp_msg[20];
char ki_msg[20];
char laturi_msg[20];
char elossa_msg[8];


// MQTT TOPIC GENERATOR
std::string BatteryMqtt(const char* variable_name, int index) {
  const char* designator[] = {"tehot", "lamposw", "boost_sw", "lamm_tot", "uptimes", "boostsens", "kp", "jannite", "ki", "laturi", "elossa", "lampo"};
  const char* prefix_constant = "Battery";
  std::string topic = prefix_constant;
  topic += "/";
  topic += variable_name;
  topic += "/";
  topic += designator[index];
  return topic;
}

void receivedCallback(char* topic, byte* payload, unsigned int length) 
{
  String topicstr = topic;
  String Lasti = String(( char *) payload); 

  // topicstr is for reading the topic AKKU_BOOST
  // Lasti is for reading the payload 1 or 0
  // 49 = 1, 48 = 0 in ASCII codes
    
  if(topicstr == AKKU_BOOST )
  {
      if(payload[0] == 49) // ASCII: 49 = 1, 48 = 0
      {
        akku_boost = 1;
        Serial.println("Boost on päällä");
      }

    if(payload[0] == 48)
      {
        akku_boost = 0;
        Serial.println("Boost on pois päältä");
      }
  }

}

void mqttconnect() {
  /* Loop until reconnected */

  if(!client.connected()) 
  {
    client.connect(DEVICENAME, MQTTUSERNAME, MQTTPASSWORD);
    if(WiFi.waitForConnectResult() == WL_CONNECTED)
      {
        Serial.print("WIFI connected, IP.add = :");
        Serial.println(WiFi.localIP());
        Serial.print("MQTT connecting ...");
      }
      else
      {
        Serial.println("failed to connect to WiFi");
        for(int i=0; i < 3; i++)
         {
          Serial.println("WiFi: Retry...");
          if(!client.connected())
            {
              client.connect(DEVICENAME, MQTTUSERNAME, MQTTPASSWORD);
            }
        }
      }
   
    /* connect now */
    if (client.connected()) {
      Serial.println("connected... ");

      for(int i = 0 ; i < 11 ; i++)
      {
        client.subscribe(BatteryMqtt("Timppa", i).c_str());
      }

      /* 
      client.subscribe(AKKUJANNITE);
      client.subscribe(AKKULAMPO);
      client.subscribe(AKKULAMPOTEHOT);
      client.subscribe(LAMPOKATKAISU);
      client.subscribe(AKKU_BOOST);
      client.subscribe(LAMMTOT);
      client.subscribe(UPTIMES);
      client.subscribe(BOOSTSENSE);
      client.subscribe(KP);
      client.subscribe(KI);
      client.subscribe(LATURI);
      client.subscribe(ELOSSA);   //elohiiri
      */

      } 
      else 
        {
        Serial.print("failed, status code =");
        Serial.print(client.state());
        Serial.println("try again in 5 seconds");
    }
  }
}

float lampomittaus()
{
  sensor1.setResolution(10);
  sensor1.requestTemperatures();
  float temppi = sensor1.getTempCByIndex(0);
  float lampo = 24;
  if(temppi != -255 || temppi != 0)
    { 
      lampo = temppi;
    }
  return lampo;
}

uint32_t battery_read()
{
  esp_adc_cal_characteristics_t characteristics;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, &characteristics);

  // Add the new reading to the array
  
  float lukema = 0;
  
  for (int i= 0; i < 3; i++)
    {
      lukema += adc1_get_raw(ADC1_CHANNEL_3);
      delay(1);
    }

  lukema = lukema / 3;

  MOVAreadings[MOVAIndex] = (esp_adc_cal_raw_to_voltage(lukema, &characteristics) * 30.81);
  MOVAIndex++;

  // If we've reached the end of the array, loop back around
  if (MOVAIndex > MOVING_AVG_SIZE) 
  {
    MOVAIndex = 0;
  }

  // Calculate the sum of the readings 
  for (uint8_t i = 1; i < MOVING_AVG_SIZE; i++)
  {
    MOVASum += MOVAreadings[i];
  }

  MOVASum = MOVASum / MOVING_AVG_SIZE;

  return MOVASum;
}

void setup()
{
  
  Serial.begin(115200);  
  //SerialBT.begin("ESP32test"); //Bluetooth device name  
  pinMode(CHARGER, OUTPUT);                                                           // Laturin rele AC
  pinMode(HEAT_1, OUTPUT);
  pinMode(HEAT_2, OUTPUT);
  pinMode(IGNITION, OUTPUT);

  esp_task_wdt_init(WDT_TIMEOUT, true);  // enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);                           // add current thread to WDT watch

  sensor1.begin();

  // on default akku_boost is on, because when wifi is not available
  // we want to charge the battery, because we are then not at home!
  // 

  akku_boost = 1;   
  
  ledcAttachPin(HEAT_1, 0); 
  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
  ledcSetup(0, 50, 8); //  Channel 0, Freq 50 Hz PWM, 8-bit resolution

  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_11db); //set reference voltage to internal


  // QuickPID
  Setpoint = 25;
  myPID.SetTunings(Kp, Ki, Kd); //apply PID gains
  myPID.SetMode(myPID.Control::automatic);   //turn the PID on
  
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    if (WiFi.config(staticIP, gateway, subnet, dns, dns) == false) 
      {
      Serial.println("Configuration failed.");
      }

    WiFi.begin(ssid, password);
   
    Serial.print("Local IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Gateway IP: ");
    Serial.println(WiFi.gatewayIP());


    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

  
    client.setServer(mqtt_server, 1883);
    /* this receivedCallback function will be invoked 
    when client received subscribed topic */
    client.setCallback(receivedCallback);
    client.connect(DEVICENAME, MQTTUSERNAME, MQTTPASSWORD);
}


void loop()
{

  // Watchdog reset timer, so nothing will get stuck.
  // very important function to keep the program running.
  if(millis() - reset_timer >= 6000) 
    {
      reset_timer = millis();
      esp_task_wdt_reset();
    }
  
 

  if(millis() - mittausmillit >= 5000)
    {
      mittausmillit = millis();



      akunjannite = battery_read();
      lampo = lampomittaus();

      Serial.print("Lampo: ");
      Serial.println(lampo);
      Serial.print("Akun jannite: ");
      Serial.println(akunjannite);
      
    }

  if( lampo < 35  && lampo > -50 && !isnan(lampo) && lampo != -255 && lampo != 0)  // estetään lämmitys liian kuumana ja liian matalassa jännitteessä.
    {
    ledcWrite(0, Output); // PWM  0-255
    lammitys_tot = 1;
    }
  else
    {
    lammitys_tot = 0;
    }
    
   // Jos käydään lämpörajoilla, katkaistaan laturin virta. 
  if( lampo < 2 || lampo >  45 )
    { 
      digitalWrite(CHARGER, LOW);
      LampoKatkaisu = 1;  // tee tästä varoitus! 
      delay(1000);
      Serial.println("Lampokatkaisu=1, charger OFF");
      laturi = 0;
    }
  



  if (millis() - lampoMillis >= 10000)      
    {
        Serial.println("lataus-looppi");
        lampoMillis = millis(); 
          
          if(lampo > 6 && lampo < 40) 
            {
              Serial.print("Latausloop-Akunjannite:  ");
              Serial.println(akunjannite);
              LampoKatkaisu = 0;

              if(akunjannite < 79500)   // mV
                {
                  Serial.println("Battery: CHARGE");
                  digitalWrite(CHARGER, HIGH);  // Lataa
                  laturi = 1;
                  akkuboostsensor = digitalRead(CHARGER);
                  
                }
            
              if(akunjannite > 80000 && millis() - full_millis >= 60000)  // 1min
                {
                  full_millis = millis(); 
                  digitalWrite(CHARGER, LOW);  //sammuta lataus -> tavoite 4.0V kennojännite
                  Serial.println("Battery: ECO");
                  laturi = 0;
                  akkuboostsensor = digitalRead(CHARGER);
                }    

              if(akunjannite < 84000 && akku_boost == 1)
                {
                  digitalWrite(CHARGER, HIGH); 
                  Serial.println("Battery: Boost");
                  laturi = 1;
                  akkuboostsensor = digitalRead(CHARGER);
                }           
            
            }
    Serial.print("Akkuboost: ");        
    Serial.println(akku_boost);
    Serial.print("Boostsensor: ");
    Serial.println(akkuboostsensor);
    }
  
  if (millis() - lastMsg >= 15000) 
    {
      lastMsg = millis();
      Serial.println(WiFi.localIP());
      Serial.print("Uploading: ... ");  
      Serial.println(akku_boost);
      mqttconnect();
            
      janniteviesti = akunjannite;
      kosteus_viesti = kosteus;
      lampo_viesti = lampo;
      bttpwr_viesti = Output;
      heatsw_viesti = LampoKatkaisu;
      boost_viesti = akku_boost; 
      lammtot_viesti = lammitys_tot;
      millis_viesti = millis() / 1000;
      bsense_viesti = akkuboostsensor;


      snprintf(v_msg, 6, "%f", janniteviesti);
      snprintf(lampo_msg, 6, "%f", lampo_viesti);
      snprintf(bttpwr_msg, 4, "%f", bttpwr_viesti);
      snprintf(heatsw_msg, 4, "%f", heatsw_viesti);
      snprintf(boostsens_msg , 4, "%f", bsense_viesti);
      snprintf(lammtotmsg, 4, "%f", lammtot_viesti);
      snprintf(ms_msg, 8, "%f", millis_viesti);
      snprintf(ki_msg, 8, "%f", ki_viesti);
      snprintf(kp_msg, 8, "%f", kp_viesti);
      snprintf(laturi_msg, 4, "%f", laturi);
      snprintf(elossa_msg, 8, "%f", elohiiri);


      /* Pubsubclient työntö serverille */
      client.publish(AKKUJANNITE, v_msg);
      client.publish(AKKULAMPO, lampo_msg);
      client.publish(AKKULAMPOTEHOT, bttpwr_msg);
      client.publish(LAMPOKATKAISU, heatsw_msg);
      client.publish(UPTIMES, ms_msg);
      client.publish(LAMMTOT, lammtotmsg);
      client.publish(BOOSTSENSE, boostsens_msg);
      client.publish(LATURI, laturi_msg);
      client.publish(ELOSSA, elossa_msg);  
  }

  // Tarkistetaan syöttö PID -loopille, ettei sensori anna -255 
  // näin on päässyt tapahtumaan koteloinnin päivityksessä. 
  if (lampo < 50 && lampo > -50 && lampo != -255 && lampo != 0)
    {
        Input = lampo;
    }

  myPID.Compute();   
  client.loop();


}