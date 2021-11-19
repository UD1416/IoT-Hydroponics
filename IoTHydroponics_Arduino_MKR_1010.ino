/*
 * file DFRobot_PH_EC.ino
 * @ https://github.com/DFRobot/DFRobot_EC
 *
 * This is the sample code for The Mixed use of two sensors: 
 *       1、Gravity: Analog pH Sensor / Meter Kit V2, SKU:SEN0161-V2
 *       2、Analog Electrical Conductivity Sensor / Meter Kit V2 (K=1.0), SKU: DFR0300.
 * In order to guarantee precision, a temperature sensor such as DS18B20 is needed, to execute automatic temperature compensation.
 * You can send commands in the serial monitor to execute the calibration.
 * Serial Commands:
 *
 *  PH Calibration：
 *   enterph -> enter the PH calibration mode
 *   calph   -> calibrate with the standard buffer solution, two buffer solutions(4.0 and 7.0) will be automaticlly recognized
 *   exitph  -> save the calibrated parameters and exit from PH calibration mode
 *
 *  EC Calibration：
 *   enterec -> enter the EC calibration mode
 *   calec   -> calibrate with the standard buffer solution, two buffer solutions(1413us/cm and 12.88ms/cm) will be automaticlly recognized
 *   exitec  -> save the calibrated parameters and exit from EC calibration mode
 *
 * Copyright   [DFRobot](http://www.dfrobot.com), 2018
 * Copyright   GNU Lesser General Public License
 *
 * version  V1.0
 * date  2018-04
 */

#define BLYNK_PRINT Serial

#define BLYNK_TEMPLATE_ID   "insert blynk template"

#include <DFRobot_PH.h>
#include <DFRobot_EC.h>
#include <DHT.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <BlynkSimpleWiFiNINA.h>
#include <FlashAsEEPROM.h>

char auth[] = "insert blynk auth token";

char ssid[] = "insert wifi ssid";
char pass[] = "insert wifi password";

#define PH_PIN A5
#define EC_PIN A3
#define TDS_PIN A4
#define LDR_PIN 13
#define DHTPIN  14

#define DHTTYPE DHT11

float  voltagePH,voltageEC,phValue,ecValue,temperature;

int TDSValue = 0;
float tdsValue = 0;
float VoltageTDS = 0;

int ldrval = 0;
int last_ldrval = 0;

float temp = 0;
float humi = 0; 

DHT dht(DHTPIN, DHTTYPE);

float h = dht.readHumidity();
float t = dht.readTemperature();

int count = 1;

BlynkTimer timer;

DFRobot_PH ph;
DFRobot_EC ec;

void setup()
{
    Serial.begin(9600);  
    ph.begin();
    ec.begin();
    dht.begin();

    Blynk.begin(auth, ssid, pass);

    timer.setInterval(1000L, myTimerEvent);
}

void myTimerEvent()
{
    char cmd[10];
    static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U){                            //time interval: 1s
        timepoint = millis();                 // read your temperature sensor to execute temperature compensation
        temperature = dht.readTemperature();
        voltagePH = analogRead(PH_PIN)/1024.0*5000;          // read the ph voltage
        phValue    = ph.readPH(voltagePH,temperature);       // convert voltage to pH with temperature compensation
        Serial.print("pH:");
        Serial.print(phValue,2);
        voltageEC = analogRead(EC_PIN)/1024.0*5000;
        ecValue    = ec.readEC(voltageEC,temperature);       // convert voltage to EC with temperature compensation
        Serial.print(", EC:");
        Serial.print(ecValue,2);
        Serial.print("ms/cm");

    TDSValue = analogRead(TDS_PIN);
    VoltageTDS = TDSValue*5/1024.0; //Convert analog reading to Voltage
    tdsValue=(133.42/VoltageTDS*VoltageTDS*VoltageTDS - 255.86*VoltageTDS*VoltageTDS + 857.39*VoltageTDS)*0.5; //Convert voltage value to TDS value
    Serial.print(", TDS Value:"); 
    Serial.print(tdsValue);
    Serial.print(" ppm");

    float h = dht.readHumidity();
    float t = dht.readTemperature(); // or dht.readTemperature(true) for Fahrenheit

    Serial.print(", Temperature:");
    Serial.print(t,2);
    Serial.print(", Humidity:");
    Serial.print(h,2);

    temp = dht.readTemperature();
    humi = dht.readHumidity();

    if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
    }

    ldrval = digitalRead(LDR_PIN);
    Serial.print(", LDR Value:");
    Serial.println(ldrval);

    if (ldrval != last_ldrval) {
        Serial.println("changed");
        Blynk.notify("Grow light status has changed!");
      }
    last_ldrval = ldrval;
    
    }
    if(readSerial(cmd)){
        strupr(cmd);
        if(strstr(cmd,"PH")){
            ph.calibration(voltagePH,temperature,cmd);       //PH calibration process by Serail CMD
        }
        if(strstr(cmd,"EC")){
            ec.calibration(voltageEC,temperature,cmd);       //EC calibration process by Serail CMD
        }
    }

    Blynk.virtualWrite(V1, temp);
    Blynk.virtualWrite(V2, humi);
    Blynk.virtualWrite(V3,ldrval);
    Blynk.virtualWrite(V4,phValue);
    Blynk.virtualWrite(V5,ecValue);
    Blynk.virtualWrite(V6,tdsValue);
}

int i = 0;
bool readSerial(char result[]){
    while(Serial.available() > 0){
        char inChar = Serial.read();
        if(inChar == '\n'){
             result[i] = '\0';
             Serial.flush();
             i=0;
             return true;
        }
        if(inChar != '\r'){
             result[i] = inChar;
             i++;
        }
        delay(1);
    }
    return false;
}


void loop()
{
  Blynk.run();
  timer.run();
}
