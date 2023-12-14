#include <Arduino.h>
#include <QuickPID.h>
//#include <math.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp_task_wdt.h>
#include <BluetoothSerial.h>
#include <iostream>
//#include <cstring>
//#include <sstream>
#include <string>
#include <vector>
#include <regex>
#include <EEPROM.h>

//#include <battery.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// MCU/Measurement related settings 
#define V_REF 1100
#define WDT_TIMEOUT 60    
#define EEPROM_SIZE 5  // This is 1-Byte   


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
float Setpoint;
float Input;
float Output;
float Kp = 8; 
float Ki = 0.001; 
float Kd = 0;  
float lammitys_tot;
float lampo;

int  temp_eeprom = 0;       //heater setpoint value for PID loop
int  nominal_eeprom = 0;    // Battery nominal string value
int  eco_eeprom = 0;        // calculated economy voltage
int  heatpoint_eeprom = 0;  // empty
int  resistance_eeprom = 0; // heater resistance value

///////////////////////////////////////////////////////////////////
/*

Todo:
 Functions: 

- Bluetooth pairing?
- Bluetooth inputs:
    - setpoint change & write on eeprom
    - resistance change & write on eeprom
    - PID - P value change & write on eeprom
    - NOMINAL_VOLTAGE change & write on eeprom
    - Heat2 & timeout & control
    - Charging on/off control from bluetooth
    - debug screen?

    SerialCommandStructureHotWords:
    
    Laturi=on/off ((??? turha?? ))
    eco=on/off

    lampo1=on/off
    lampo2=on/off

    heat2pwr=50
    heat2time=360

    setpoint=25
    res1=100
    res2=200
    pidP=10
    pidI=0.01
    NominalS=20
    Help



- Bluetooth output intervalls. 
    - 5min
    - 60min
    - 6h
    - 24h

Bluetooth output with multiple lines and all the data.




*/
///////////////////////////////////////////////////////////////////


class Battery {
private:
  int     _ChargerON;
  int     _Heat1ON;
  int     _Heat2ON;

  int     _setOutput;
  int     _getOutput;
  int     _heaterResistance;
  int     _resistance;

  int     _ecoModeStatus;
  int     _ecoPrecent;
  int     _nominalS;
  float   _ecoVolts;


  int       _setSetpoint;
  float     _temperature;
  float     _voltage;

  static const int MAX_TOKENS = 2;
  char* SerialCommand[MAX_TOKENS];


// String to Integer function
    int stringToInt(std::string s) {
    std::istringstream iss(s);
    int number;
    iss >> number;
    return number;
    }


////////////////////////////////////////////////////////////////////////////////////
//
//
// Battery Public functions
  public:


  // Control Logic for the Bluetooth Serial Command Interface 
  void handleCommand(std::string command, std::string value) {


    /////////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Most basic inputs needed for functionality: stemp, nominal, ecop, heatr
    //
    // temperature setpoint interface for user
    if(command == "stemp") {
       int arvo = stringToInt(value);

        Serial.print("SerialBT command input: ");
        Serial.print("SetTemp: ");
        Serial.println(arvo);

       if( arvo >= 0 && arvo <= 45 ) 
         {
         this->setHeatpoint(arvo);
         Serial.print("SetTemp: Set " );
         Serial.println(arvo);
         }
       else 
         {
         Serial.println("SetTemp: Invalid value");
         }
     }
    //
    // Nominal Battery String set interface for user
    if(command == "nominal")  {
        int arvo = stringToInt(value);

        Serial.print("SerialBT command input: ");
        Serial.print("Nominal: ");
        Serial.println(arvo);
        Serial.print("\n");

        if(arvo >= 7 && arvo <= 24) 
          {
          this->setNominalS(arvo);
          Serial.print("this -- > setNominalS is set to: " );
          Serial.println(arvo);
          }
       else 
         {
         Serial.println("Nominal: Invalid value");
         }
     }
    //
    // Battery ECO precentage set interface for user 
    if(command == "ecop") {
       int arvo = stringToInt(value);
       if(arvo <= 100 && arvo >= 50) 
         {
         setEcoPrecent(arvo);
         Serial.print("this --> EcoPrecent is Set: " );
         Serial.println(arvo);
         }
       else 
         {
         Serial.println("EcoPrecent: Invalid value");
         }
     }
    //
    // Heater Resistance set interface for user 0 ... 255
     if(command == "heater") {
       int arvo = stringToInt(value);
       if(arvo >= 0 && arvo <= 255) 
         {
          _resistance = arvo;
          setResistance(arvo);
          Serial.print("this --> Resistance is Set: " );
          Serial.println(arvo);
         }
       else 
         {
         Serial.println("Resistance: Invalid value");
         }
     }
    //
    ////////////////////////////////////////////////////////////////////////////////////////////7


    ////////////////////////////////////////////////////////////////////////////////////////////
    //
    // User controlled functions for everyday use. 
    //
    if(command == "laturi") {
       if(value == "on") 
         {
         Battery::setChargerStatus(1);
         Serial.println("Laturi on_HANDLECOMMAND");
         }
         else if(value == "off") 
         {
         Battery::setChargerStatus(0);
         Serial.println("Laturi off_HANDLECOMMAND");
         }
     }
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////////


// End of HandleCommand!  
  }   
//////////////////////////////////////////////////////////////          
    
    
  // Read User Set Charger status from private value
int getChargerStatus() {
    Serial.print("Laturi-internal_fetch\n");
    Serial.print(_ChargerON);
    return _ChargerON;

  }

  // Set Charger status to private value
void setChargerStatus(int status) {
    _ChargerON = status;
    Serial.print("Laturi-internal_set\n");
  }

  // function to tokenize serial read input.
    // https://stackoverflow.com/questions/9072320/split-string-into-string-array
void processSerialInput(std::string input) {
       // Convert the char input into a string
    std::string strInput = input;
    Serial.print("strInput: ");
    Serial.println(strInput.c_str());

    // Use std::regex and std::sregex_token_iterator to split the string
    std::regex re("\\s");
    std::sregex_token_iterator first{strInput.begin(), strInput.end(), re, -1}, last;
    std::vector<std::string> tokens = {first, last};

    if (!tokens.empty()) {
    tokens.back().erase(std::remove(tokens.back().begin(), tokens.back().end(), '\n'), tokens.back().end());
}

    Serial.print("token1: ");
    Serial.print(tokens[0].c_str());
    Serial.print("token2: ");
    Serial.print(tokens[1].c_str());
    Serial.print("\n");
    Serial.print(tokens.size());

    // Call handleCommand with the command and value
    if (tokens.size() <= 2) {

        handleCommand(tokens[0].c_str(), tokens[1].c_str());
        Serial.println("handlecommand_functioncall");
        } else {
            Serial.println("Invalid input");
        }
    }

  // Calculates from the set resistance how much power is consumed at this voltage level.
float HeatPowerResult() { 
    float result = 0;
    float voltage = Battery::getVoltage();

    // Equation is: P = (PWM * U^2) / R
    // PWM = quickpid output precentage

    voltage = voltage * voltage;
    float resistance = Battery::getResistance();
    result = voltage / resistance;
    Serial.print("Power: ");
    Serial.println(result);



    return result;
  }

// Get resistance of the heater element
int getResistance() {
  return _resistance;
  } 

// Set resistance of the heater element
int setResistance(int resistance) { 
  if(resistance >= 0 && resistance <= 255) {
    _resistance = resistance;

    EEPROM.write(4, resistance);
    EEPROM.commit();
    Serial.print("Resistance: EEPROM write");
    Serial.println(resistance);

    }
  else {
    Serial.println("Resistance: Invalid value");
    return 0;
    }
  return 1;
  }

  // Heater Channel 0 output Function
void Heater() {
    if( lampo > -30 && lampo < 42 ) {
      ledcWrite(0, Output); // PWM  0-255
    }
  }

  // User set temperature setpoint
void setHeatpoint(int setpoint) {
      if(setpoint > 0 && setpoint < 40) {
        _setSetpoint = setpoint;
        Setpoint = setpoint;

        if (temp_eeprom != setpoint) {
          EEPROM.write(0, setpoint);
          EEPROM.commit();
          Serial.print("Setpoint: EEPROM write");
          Serial.println(Setpoint);
        }
        }
      }
  
  // User get temperature setpoint
int getHeatpoint() {
    return _setSetpoint;
  }

  // Get Temperature from private value
float getTemp() {
    return _temperature;
  }

  // read temperature and set it to private value
float readTemp() { 
    OneWire Wire(TEMPs);
    DallasTemperature sensor1(&Wire);
    sensor1.begin();
    sensor1.setResolution(10);
    sensor1.requestTemperatures();

    float reading = sensor1.getTempCByIndex(0);
    if (reading > -40 && reading < 80) {                                      // add more filterin, if one is 10 % above the last, then mark it as error. -> reset?
      _temperature = reading;
    } 
    else {
      return 255; // fix! -> filter before output -> scaling problem.
    }
    return _temperature;
  Serial.println(_temperature);
  }

  // Get Voltage from private value
float getVoltage() {
    return _voltage;  // mvoltages to volts
  }

  // read voltage and set it to private value
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
        _voltage = MOVASum / 1000;
    } 
    return MOVASum / 1000;
  }

  // Get status if Eco mode is set or not aka. boost mode.
int getEcoMode() {
    return _ecoModeStatus;
    }
  
  // set status if Eco mode is set or not aka. boost mode.
void setEcoMode(int status) {
    _ecoModeStatus = status;
    }

  // get battery's string value, min: 7S and max: 24S (tehoretical)
int getNominalS() {
    return _nominalS;
    }
  
  // set battery's string value, min: 7S and max: 24S (tehoretical)
void setNominalS(int nominalS) {
    if(nominalS > 7 && nominalS < 24) {
       _nominalS = nominalS; 

          EEPROM.write(1, nominalS);
          EEPROM.commit();
          Serial.print("NominalS: EEPROM write");
          Serial.println(nominalS);

        }
      else { 
        Serial.println("NominalS: Invalid value");
        }
      }
    
// set Eco mode precentage, min: 50% and max: 100%
void setEcoPrecent(int ecoPrecent) {
    if(ecoPrecent >= 50 && ecoPrecent <= 100)
      { 
        _ecoPrecent = ecoPrecent; 

          EEPROM.write(2, ecoPrecent);
          EEPROM.commit();
          Serial.print("EcoPrecent: EEPROM write");
          Serial.println(ecoPrecent);
          

      float full = float(getNominalS()) * 4.2;
      float empty = float(getNominalS()) * 3.2;
      float VoltScale = full - empty;     // whats the voltscale in the used nominal battery level
      Serial.print("VoltScale: ");
      Serial.println(VoltScale);
      Serial.print("Full: ");
      Serial.println(full); 
      Serial.print("Empty: ");  
      Serial.println(empty);
      float Eprecent = float(ecoPrecent) / float(100);  // whats the precentage of the nominal battery level
      Eprecent = Eprecent * VoltScale;    // whats the voltscale in the used precentage
      float ecoVolt = Eprecent + empty;   // whats the eco voltage in the used nominal battery level
      
      Serial.print("EcoPrecent: ");
      Serial.println(Eprecent);  
      Serial.print("EcoVolts");
      Serial.println(ecoVolt);

      _ecoVolts = ecoVolt;
      }

        
      }
    
 // get calculated economy voltage based on batterys nominal voltage and eco mode precentage. 
float getEcoModeVoltage()
    {
      return _ecoVolts;
    }

// Class end
};

//
// End of Battery Class
//
//////////////////////////////////////////////////////////////////////////////


// kirjastot
BluetoothSerial SerialBT;
Battery batt;
QuickPID myPID(&Input, &Output, &Setpoint);

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
    boolean confirmRequestPending = true;

    // Handle received and sent messages
    String message = "";
    char incomingChar;
//
//////////////////////////////////////////////////////////////////////////////////

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
  EEPROM.begin(EEPROM_SIZE);

  //SerialBT.enableSSP();
  SerialBT.setPin("1234");

  //SerialBT.disableSSP();
  SerialBT.onConfirmRequest(BTConfirmRequestCallback);
  SerialBT.onAuthComplete(BTAuthCompleteCallback);
  SerialBT.begin("BattHeater Lassi"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  

  pinMode(CHARGER, OUTPUT);                                                           // Laturin rele AC
  pinMode(HEAT_1, OUTPUT);
  pinMode(HEAT_2, OUTPUT);
  pinMode(IGNITION, OUTPUT);

  esp_task_wdt_init(WDT_TIMEOUT, true);  // enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);                           // add current thread to WDT watch


  ledcAttachPin(HEAT_1, 0); 
  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
  ledcSetup(0, 250, 8); //  Channel 0, Freq 250 Hz PWM, 8-bit resolution

  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_11db); //set reference voltage to internal

  // QuickPID
  Setpoint = 0;
  myPID.SetTunings(Kp, Ki, Kd); //apply PID gains
  myPID.SetMode(myPID.Control::automatic);   //turn the PID on

/*
  , Kp, Ki, Kd,
               myPID.pMode::pOnError,
               myPID.dMode::dOnMeas,
               myPID.iAwMode::iAwClamp,
               myPID.Action::direct);
*/

  temp_eeprom = EEPROM.read(0);
  nominal_eeprom = EEPROM.read(1);
  eco_eeprom = EEPROM.read(2);
  heatpoint_eeprom = EEPROM.read(3);
  resistance_eeprom = EEPROM.read(4);
  
  //if (temp_eeprom != 0) {
    batt.setHeatpoint(temp_eeprom);
    Serial.print("Setpoint: EEPROM read");
    Serial.println(temp_eeprom);
  //}

  //if (nominal_eeprom != 0) {
    batt.setNominalS(nominal_eeprom);
    Serial.print("NominalS: EEPROM read");
    Serial.println(nominal_eeprom);
  //}

  //if (eco_eeprom != 0) {
    batt.setEcoPrecent(eco_eeprom);
    Serial.print("EcoPrecent: EEPROM read");
    Serial.println(eco_eeprom);
  //}

  //if (resistance_eeprom != 0) {
    batt.setResistance(resistance_eeprom);
    Serial.print("Resistance: EEPROM read");
    Serial.println(resistance_eeprom);
  //} 

}

void loop()
{
  myPID.Compute();  // calculate the output value for the PID loop
  //float MyPower = batt.HeatPowerResult() * Output; // calculate the power consumption of the heater element
  
  // Watchdog reset timer, so nothing will get stuck.
  // very important function to keep the program running.
  if(millis() - reset_timer >= 6000) 
    {
      reset_timer = millis();
      esp_task_wdt_reset();
    }

  // MITTAUSLOOPPI
  if(millis() - mittausmillit >= 4000)
    {
      mittausmillit = millis();
      akunjannite = batt.readVoltage();
      lampo = batt.readTemp();

      SerialBT.print("\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n");

      // YlinRivi
      SerialBT.print("    ");
      SerialBT.print("Akku: ");
      SerialBT.print(batt.getNominalS());
      SerialBT.print("S");
      SerialBT.print("   =   ");
      SerialBT.print(round(batt.getNominalS() * 3.6));
      SerialBT.print("\r\n\r\n");

      //ToiseksylinRivi
      SerialBT.print("           ");
      SerialBT.print("ECO-limit: ");
      SerialBT.print(round(batt.getEcoModeVoltage()));
      SerialBT.print("\r\n\r\n");

      //kolmanneksYlinrivi
      SerialBT.print("           ");
      SerialBT.print("Heater: ");
      SerialBT.print(" 50 W");
      SerialBT.print("     CutOFF: 39 V");
      SerialBT.print("\r\n\r\n");

      SerialBT.print("           ");
      SerialBT.print("Setpoint:  ");
      SerialBT.print(batt.getHeatpoint());
      SerialBT.print("      (min: +5 max: +30 C)");
      SerialBT.print("\r\n\r\n");



      // FirstLine
      SerialBT.print("      ----------------------------------------------------------------");
      SerialBT.print("\r\n\r\n");
      SerialBT.print("        ");
      SerialBT.print(lampo);
      SerialBT.print(" C");
      SerialBT.print("    ");
      SerialBT.print(Input);
      SerialBT.print(" C");
      SerialBT.print("    ");
      SerialBT.print(Setpoint);
      SerialBT.print(" C");
      SerialBT.print("    ");
      SerialBT.print(" ");
      SerialBT.print(" C");
      SerialBT.print("\r\n\r\n");

            // SecondLine

      float PowerNOW = batt.HeatPowerResult() * Output / 255;
      SerialBT.print("        ");
      SerialBT.print(PowerNOW);
      SerialBT.print(" W");
      SerialBT.print("     ");
      SerialBT.print("e");
      SerialBT.print(" W");
      SerialBT.print("     ");
      SerialBT.print(Output / 255);
      SerialBT.print(" %");
      SerialBT.print("     ");
      SerialBT.print(Output);
      SerialBT.print(" byte");
      SerialBT.print("\r\n\r\n");       
      SerialBT.print("      ----------------------------------------------------------------");
      SerialBT.print("\r\n\r\n");




      SerialBT.print("        ");
      SerialBT.print("Voltage ");
      SerialBT.print("       ");
      SerialBT.print("AUX Heater");
      SerialBT.print("           ");
      SerialBT.print("CHARGER ");
      SerialBT.print("     ");
      SerialBT.print("\r\n\r\n");


      SerialBT.print("        ");
      SerialBT.print(batt.getVoltage());
      SerialBT.print("            ");
      SerialBT.print("ON (15min)");
      SerialBT.print("             ");
      SerialBT.print(batt.getChargerStatus());
      SerialBT.print("            ");
      //firstEmptySpace
      SerialBT.print("\r\n\r\n\r\n");
    }

  // REWORK! Lämmitysluuppi
  if( lampo < 35  && lampo > -40 && !isnan(lampo) && lampo != 255 && lampo != 0)  
    {
    //myPID.Compute();  // calculate the output value for the PID loop
    ledcWrite(0, Output); // PWM  0-255 in float. 0-100%.
    lammitys_tot = 1;
    }
  else
    {
    lammitys_tot = 0;
    }
    
   //  NOPEA LÄMPÖSUOJA: Jos käydään lämpörajoilla, katkaistaan laturin virta. 
  if( lampo < 5 || lampo >  45 )
    { 
      digitalWrite(CHARGER, LOW);
      LampoKatkaisu = 1;  // tee tästä varoitus! 
      delay(1000);
      Serial.println("Lampokatkaisu");
      laturi = 0;
    }
  
  // LATAUSLOOPPI
  if (millis() - lampoMillis >= 4000)      
    {
        Serial.println("Lataus_10s-loop");
        lampoMillis = millis(); 

        Serial.print("Output arvo: ");
        Serial.println(Output); 

          
        if(lampo > 10 && lampo < 40) 
            {
              Serial.print("Latausloop: ");
              Serial.println(akunjannite);

              if(akunjannite < batt.getEcoModeVoltage() - 1)   // mV
                {
                  Serial.println("Battery: CHARGE");
                  digitalWrite(CHARGER, HIGH);  // Lataa
                  laturi = 1;
                  
                }


              if(akunjannite > batt.getEcoModeVoltage() && millis() - full_millis >= 60000)  // 1min
                {
                  full_millis = millis(); 
                  digitalWrite(CHARGER, LOW);  //sammuta lataus -> tavoite 4.0V kennojännite
                  Serial.println("Battery: ECO");
                  laturi = 0;
                }    

              if(akunjannite < 84 && akku_boost == 1)
                {
                  digitalWrite(CHARGER, HIGH); 
                  Serial.println("Battery: Boost");     
                  laturi = 1;           
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

    
    lammitys_tot = 1;
  
  


    /*
    Serial.print("  ");
    //Serial.print("Lämmitys-input: ");
    Serial.print(lampo);
    Serial.print("  "  );
    Serial.print(Input);
    */
    }

// reading and processing bluetooth input
  if (SerialBT.available())
    {
      std::string incomingSer = std::string(SerialBT.readStringUntil('\n').c_str());
          batt.processSerialInput(incomingSer);
          SerialBT.print("Message received: ");
          SerialBT.print(incomingSer.c_str());
          Serial.println(incomingChar);

    }

    





//
// End of Loop
//
}





