/*
Ensinmäinen proseduraalinen testikoodi laitteistoa ohjaamaan ja toimintoja testaamaan. 
Seuraavaksi tehdään akusta olio.
*/

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

const char* ssid     = WIFINAME;
const char* password = WIFIPASS;
const char* mqtt_server = MQTTSRV;

IPAddress staticIP(HOSTIP);
IPAddress gateway(GAYWAY);
IPAddress subnet(SUBNET);
IPAddress dns(DNS);

// MQTT Subscrib/Timppa
#define AKKULAMPOTEHOT  "Battery/Anni/tehot"
#define LAMPOKATKAISU   "Battery/Anni/lamposw"
#define AKKU_BOOST      "Battery/Anni/boost_sw"
#define LAMMTOT         "Battery/Anni/lamm_tot"
#define UPTIMES         "Battery/Anni/uptimes"
#define BOOSTSENSE      "Battery/Anni/boostsens"
#define KP              "Battery/Anni/kp"
#define AKKUJANNITE     "Battery/Anni/jannite"
#define KI              "Battery/Anni/ki"
#define LATURI          "Battery/Anni/laturi"
#define ELOSSA          "Battery/Anni/elossa"
#define AKKULAMPO       "Battery/Anni/lampo"

#define V_REF 1100
#define WDT_TIMEOUT 60          // Watchdog timeout 60 sekunttia

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
bool printOrPlotter = 0;  // on(1) monitor, off(0) plotter
float POn = 1.0;          // proportional on Error to Measurement ratio (0.0-1.0), default = 1.0
float DOn = 0.0;          // derivative on Error to Measurement ratio (0.0-1.0), default = 0.0
float Setpoint, Input, Output;
float Kp = 15, Ki = 0.001, Kd = 0;  // edellinen: P=6, I=0.0015 - toimii riittävän hyvin -> massan hitaus.  - ei kaavoja, liian pitkä aika.
float lammitys_tot;
float kosteus;
float lampo;
float ressuaika = 600000;

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
float NewSetpoint = 0;
int LampoKatkaisu = 1;
int akku_boost = 0;
float millisek = 0;
int akkuboostsensor = 0;
float akku_lepojannite = 64;
float laturi = 0;
float akunjannite = 0;
int lammitys80; 
float elohiiri = 0;

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


// akun lukufunktio ADC:ltä.
// Luetaan 5 kertaa. Ei riitä, tuloksissa liikaa noisea (vanhalla protolla). 
// Tähän tulisi tehdä moving avarage ja muutama luku. 

float battery_read()
  {
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_11db); //set reference voltage to internal
    
  // Calculate ADC characteristics i.e. gain and offset factors
  esp_adc_cal_characteristics_t characteristics;
  esp_adc_cal_get_characteristics(V_REF, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, &characteristics);

    uint32_t voltage = 0;
    //future update: Make rolling avarage

    for (int i = 0; i < 5; i++)
      {
        voltage += adc1_to_voltage(ADC1_CHANNEL_3, &characteristics);    // UPDATE: voltage =+ esp_adc_cal_get_voltage(ADC1_CHANNEL_0, &characteristics);
        delay(15);
      }

    voltage = (voltage / 5) * 30.81;
    Serial.printf("%d V\n", voltage);
    return voltage;
   }

void receivedCallback(char* topic, byte* payload, unsigned int length) 
{
  String topicstr = topic;
  String Lasti = String(( char *) payload); 
  
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

// MQTT yhdistys funktio.
void mqttconnect() {
  /* Loop until reconnected */

  if(!client.connected()) {
    client.connect(DEVICENAME, MQTTUSERNAME, MQTTPASSWORD);
    if(WiFi.waitForConnectResult() == WL_CONNECTED)
      {
        Serial.print(", WIFI connected, IP.add = :");
        Serial.println(WiFi.localIP());
        Serial.print("MQTT connecting ...");
        elohiiri = 1;  // Turha? tilatieto hassiolle, elossa tarkistus bitin tönäisy. Epävakautta ollut. Ei juurikan tärkeä/ei käyttöä. 
      }
      else
      {
        Serial.println(", failed to connect to WiFi ");
        for(int i=0;i<10;i++)
        {
          Serial.println("WiFi: Retry...");
          if(!client.connected())
            {
            client.connect(DEVICENAME, MQTTUSERNAME, MQTTPASSWORD);
            delay(50);
            }
            elohiiri = 0; // elohiiri nollaksi, jos ollut vaikeuksia saada ekallakeralla yhteyttä. Tälle ei käyttöä kylläkään.
        }
      }
   
    
    /* client ID */
    String clientId = DEVICENAME;
    
    /* connect now */
    if (client.connected()) {
      Serial.println("connected... ");

      /* subscribe topic with default QoS 0*/
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
      } 
      else 
        {
          Serial.println("connecting to server...");

      Serial.print("failed, status code =");
        Serial.print(client.state());
        Serial.println("try again in 5 seconds");
        /* Wait 5 seconds before retrying */
        delay(500);
        elohiiri = 0;
    }
  }
}



void setup()
{
  
  Serial.begin(115200);    
  pinMode(CHARGER, OUTPUT);                                                           // Laturin rele AC
  pinMode(HEAT_1, OUTPUT);
  pinMode(HEAT_2, OUTPUT);
  pinMode(IGNITION, OUTPUT);

  sensor1.begin();
  akku_boost = 1;   // Boost arvo oletus 1 = ON. 
  
  ledcAttachPin(HEAT_1, 0); 
  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
  ledcSetup(0, 50, 8); //  Channel 0, Freq 50 Hz PWM, 8-bit resolution

  // QuickPID
  Setpoint = SETTEMP;
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
  if(millis() - mittausmillit >= 15000)
    {
      mittausmillit = millis();
      sensor1.setResolution(10);
      
      sensor1.requestTemperatures();
      float temppi = sensor1.getTempCByIndex(0);
      if(temppi != -255 || temppi != 0)
            { 
              lampo = temppi;
            }
        
      akunjannite = battery_read();

      Serial.print("Lampo: ");
      Serial.println(lampo);
      Serial.print("Akun jannite: ");
      Serial.println(akunjannite);
      
    }

  if( lampo < 35  && lampo > -50 && !isnan(lampo) && lampo != -255 && lampo != 0)  // estetään lämmitys liian kuumana ja liian matalassa jännitteessä.
    {
    lammitys80 = map(Output, 0, 255, 0, 255);
    ledcWrite(0, lammitys80);
    lammitys_tot = 1;
    }
  else
    {
    lammitys_tot = 0;
    }
    
   // Jos käydään lämpörajoilla, katkaistaan laturin virta. 
  if( lampo < MINTEMP || lampo >  MAXTEMP )
    { 
      digitalWrite(CHARGER, LOW);
      LampoKatkaisu = 1;  // tee tästä varoitus! 
      delay(1000);
      Serial.println("Lampokatkaisu=1, charger OFF");
      laturi = 0;
    }
  



  if (millis() - lampoMillis >= 60000)      
    {
        Serial.println("lataus-looppi");
        lampoMillis = millis(); 
          
          if(lampo > MINTEMP + 4 && lampo < MAXTEMP - 5) 
            {
              Serial.print("Latausloop-Akunjannite:  ");
              Serial.println(akunjannite);
              LampoKatkaisu = 0;

              if(akunjannite < 40000)  // mV
                {
                  Serial.println("Battery volts: (48 < 64) in ECO mode");
                  digitalWrite(CHARGER, HIGH);  // Lataa
                  laturi = 1;
                  
                }
            
              if(akunjannite > 50000 && millis() - full_millis >= 10000)  // 1min
                {
                  full_millis = millis(); 
                  digitalWrite(CHARGER, LOW);  //sammuta lataus -> tavoite 4.0V kennojännite
                  Serial.println("Battery ECO mode -loop");
                  laturi = 0;
                  //akkuboostsensor = digitalRead(CHARGER);
                }    

              if(akunjannite < 60000 && akku_boost == 1)
                {
                  digitalWrite(CHARGER, HIGH);  //käynnistä lataus -> tavoite 4.2V kennojännite
                  Serial.println("Battery ECO mode override");
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
      Serial.print("mqttimer-connect & boost: ");  
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