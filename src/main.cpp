#include <Arduino.h>
#include <QuickPID.h>
#include <math.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp_task_wdt.h>
#include <string>
#include <pass.h>
#include <BluetoothSerial.h>
//#include <battery.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

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
float Kp = 8, Ki = 0.001, Kd = 0;  // edellinen: P=6, I=0.0015 - toimii riittävän hyvin -> massan hitaus.  - ei kaavoja, liian pitkä aika.
float lammitys_tot;
float lampo;

//String RxBuffer = "";
//char RxByte;

///////////////////////////////////////////////////////////////////
//
// My Battery Class 
//
//
///////////////////////////////////////////////////////////////////

class Battery {
private:
  int   _setSetpoint;
  int   _getSetpoint;
  float _temperature;
  float _voltage;

public:

  void Heater() {
    if( lampo > -30 && lampo < 30 ) {
      ledcWrite(0, Output); // PWM  0-255
    }
  }
  int setHeatpoint(int setpoint) {
      if(setpoint > 0 && setpoint < 30) {
        this->_setSetpoint = setpoint;
      }
      else {
        return 0;
        }
  }
  int getHeatpoint() {
    return this->_getSetpoint;
  }
  float getTemp() {
    return this->_temperature;
  }
  float readTemp() { 
    OneWire Wire(TEMPs);
    DallasTemperature sensor1(&Wire);
    sensor1.begin();
    sensor1.setResolution(9);
    sensor1.requestTemperatures();

    float reading = sensor1.getTempCByIndex(0);
    if (reading > -40 && reading < 80) {                                      // add more filterin, if one is 10 % above the last, then mark it as error. -> reset?
      this->_temperature = reading;
    } 
    else {
      return 255; // fix! -> filter before output -> scaling problem.
    }
    return this->_temperature;
  }
  float getVoltage() {
    return this->_voltage;
  }
  float readVoltage() {
    esp_adc_cal_characteristics_t characteristics;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, &characteristics);

    // Add the new reading to the array
    // this takes avarage of 3 readings.
    float lukema = 0;
    for (int i= 0; i < 3; i++)
      {
        lukema += adc1_get_raw(ADC1_CHANNEL_3);
      }
    lukema = lukema / 3;

    // If we've reached the end of the array, loop back around
    if (MOVAIndex > MOVING_AVG_SIZE) 
    {
       MOVAIndex = 0;
    }

    MOVAreadings[MOVAIndex] = (esp_adc_cal_raw_to_voltage(lukema, &characteristics) * 30.81);
    MOVAIndex++;

    // Calculate the sum of the readings 
    for (int i = 1; i < MOVING_AVG_SIZE; i++)
    {
      MOVASum += MOVAreadings[i];
    }

    MOVASum = MOVASum / MOVING_AVG_SIZE;

    if (MOVASum > 0 && MOVASum < 100000) {                                      // add more filterin, if one is 10 % above the last, then mark it as error. -> reset?
      this->_voltage = MOVASum;
    } 
    else {
      return 255;
    }
    return this->_voltage;
  }
};

// kirjastot
BluetoothSerial SerialBT;
Battery batt;
QuickPID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd,
               myPID.pMode::pOnError,
               myPID.dMode::dOnMeas,
               myPID.iAwMode::iAwClamp,
               myPID.Action::direct);


// 

unsigned long lampoMillis = 0; // Heat condifiton checking loop millis()
unsigned long mittausmillit = 0;  // battery voltage and heat reading millis()
unsigned long full_millis = 0; // Eco-or-Full battery millis counter
unsigned long reset_timer = 0;  // Watchdog reset timer, so nothing will get stuck.

// ohjelman määrityksiä nolliin. Alustusta.
int     LampoKatkaisu = 1;
int     akku_boost = 0;
float   millisek = 0;
int     akkuboostsensor = 0;
float   laturi = 0;
float   akunjannite = 0;

// Bluetooth stuff
const char* BTPIN = "1234";
boolean confirmRequestPending = true;

void BTConfirmRequestCallback(uint32_t numVal)
{
  confirmRequestPending = true;
  Serial.println(numVal);
}

void BTAuthCompleteCallback(boolean success)
{
  confirmRequestPending = false;
  if (success)
  {
    Serial.println("Pairing success!!");
  }
  else
  {
    Serial.println("Pairing failed, rejected by user!!");
  }
}



void setup()
{
  
  Serial.begin(115200);  

  SerialBT.enableSSP();
  SerialBT.onConfirmRequest(BTConfirmRequestCallback);
  SerialBT.onAuthComplete(BTAuthCompleteCallback);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  pinMode(CHARGER, OUTPUT);                                                           // Laturin rele AC
  pinMode(HEAT_1, OUTPUT);
  pinMode(HEAT_2, OUTPUT);
  pinMode(IGNITION, OUTPUT);

  esp_task_wdt_init(WDT_TIMEOUT, true);  // enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);                           // add current thread to WDT watch


  ledcAttachPin(HEAT_1, 0); 
  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
  ledcSetup(0, 50, 8); //  Channel 0, Freq 50 Hz PWM, 8-bit resolution

  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_11db); //set reference voltage to internal

  // QuickPID
  Setpoint = 25;
  myPID.SetTunings(Kp, Ki, Kd); //apply PID gains
  myPID.SetMode(myPID.Control::automatic);   //turn the PID on
  
}


void loop()
{

   myPID.Compute();  // calculate the output value for the PID loop

  // Watchdog reset timer, so nothing will get stuck.
  // very important function to keep the program running.
  if(millis() - reset_timer >= 6000) 
    {
      reset_timer = millis();
      esp_task_wdt_reset();
    }
  
 
  // MITTAUSLOOPPI
  if(millis() - mittausmillit >= 1000)
    {
      mittausmillit = millis();

      akunjannite = batt.readVoltage();
      lampo = batt.readTemp();

      Serial.print("Lampo: ");
      Serial.println(lampo);

      SerialBT.print("Lampo: ");
      SerialBT.println(lampo);

      Serial.print("Jannite: ");
      SerialBT.print("Jannite: ");
      
      Serial.println(akunjannite);
      SerialBT.println((akunjannite));
    

      
    }

  // LÄMMITYKSEN TOTEUTUS
  if( lampo < 35  && lampo > -40 && !isnan(lampo) && lampo != 255 && lampo != 0)  // estetään lämmitys liian kuumana ja liian matalassa jännitteessä.
    {
    ledcWrite(0, Output); // PWM  0-255
    lammitys_tot = 1;
    }
  else
    {
    lammitys_tot = 0;
    }
    
   //  NOPEA LÄMPÖSUOJA: Jos käydään lämpörajoilla, katkaistaan laturin virta. 
  if( lampo < 2 || lampo >  45 )
    { 
      digitalWrite(CHARGER, LOW);
      LampoKatkaisu = 1;  // tee tästä varoitus! 
      delay(1000);
      Serial.println("Lampokatkaisu");
      laturi = 0;
    }
  


  // LATAUSLOOPPI
  if (millis() - lampoMillis >= 10000)      
    {
        Serial.println("Lataus");
        lampoMillis = millis(); 
        Serial.print("PID-Output: ");
        Serial.println(Output);

          
          if(lampo > 6 && lampo < 40) 
            {
              Serial.print("Latausloop: ");
              Serial.println(akunjannite);

              if(akunjannite < 80000)   // mV
                {
                  Serial.println("Battery: CHARGE");
                  digitalWrite(CHARGER, HIGH);  // Lataa
                  
                }
            
              if(akunjannite > 80000 && millis() - full_millis >= 60000)  // 1min
                {
                  full_millis = millis(); 
                  digitalWrite(CHARGER, LOW);  //sammuta lataus -> tavoite 4.0V kennojännite
                  Serial.println("Battery: ECO");
                }    

              if(akunjannite < 85000 && akku_boost == 1)
                {
                  digitalWrite(CHARGER, HIGH); 
                  Serial.println("Battery: Boost");              
                }           
            
            }
    Serial.print("Akkuboost: ");        
    Serial.println(akku_boost);
    Serial.print("Boostsensor: ");
    Serial.println(akkuboostsensor);
    }

  // Tarkistetaan syöttö PID -loopille, ettei sensori anna hassuja lukuja!
  if (lampo < 50 && lampo > -40 && lampo != -255 && lampo != 0 && lampo != 127)
    {
        Input = lampo;
    }

  
  if (confirmRequestPending)
  {
    if (Serial.available())
    {
      int dat = Serial.read();
      if (dat == 'Y' || dat == 'y')
      {
        SerialBT.confirmReply(true);
      }
      else
      {
        SerialBT.confirmReply(false);
      }
    }
  }
  else
  {
    if (Serial.available())
    {
      SerialBT.write(Serial.read());
    }
    if (SerialBT.available())
    {
      Serial.write(SerialBT.read());
    }
    delay(20);
  }

}