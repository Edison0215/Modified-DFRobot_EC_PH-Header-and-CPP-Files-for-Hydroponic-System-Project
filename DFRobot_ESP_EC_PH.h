/*
 * file DFRobot_ESP_EC.h * @ https://github.com/GreenPonik/DFRobot_ESP_EC_BY_GREENPONIK
 *
 * Arduino library for Gravity: Analog EC Sensor / Meter Kit V2, SKU: DFR0300
 * 
 * Based on the @ https://github.com/DFRobot/DFRobot_EC
 * Copyright   [DFRobot](http://www.dfrobot.com), 2018
 * Copyright   GNU Lesser General Public License
 *
 * ##################################################
 * ##################################################
 * ########## Fork on github by GreenPonik ##########
 * ############# ONLY ESP COMPATIBLE ################
 * ##################################################
 * ##################################################
 * 
 * version  V1.1.2
 * date  2019-06
 */

//INCLUDE LIGHT CONTROL
#ifndef _DFROBOT_ESP_EC_PH_H_
#define _DFROBOT_ESP_EC_PH_H_

#include "Arduino.h"

#define KVALUEADDR 10 //the start address of the K value stored in the EEPROM
#define RAWEC_1413_LOW 0.70
#define RAWEC_1413_HIGH 1.80
#define RAWEC_276_LOW 1.95
#define RAWEC_276_HIGH 3.2
#define RAWEC_1288_LOW 8
#define RAWEC_1288_HIGH 16.8

#define ReceivedBufferLength 10 //length of the Serial CMD buffer

#define PHVALUEADDR 0 //the start address of the pH calibration parameters stored in the EEPROM

/**
 * first you need to define the raw voltage for your circuit
 * the raw voltage for neutral pH 7.0 and acid pH 4.0 at 25 °c
 * for the actual circuit => ESP32 + ADC (ADS1115)
*/
/**
 * you may have to adapt voltage acid and neutral offset or directly the ranges
 * according to the signal sending by your own pH probe if you don't use the DFRobot pH sensor kit probe
 * same circuit with different probe return different voltage value for buffer4.0 or buffer 7.0
 */
#define PH_VOLTAGE_ACID_OFFSET 200
#define PH_VOLTAGE_NEUTRAL_OFFSET 200
#define PH_8_VOLTAGE 995  //linear culculation
#define PH_7_AT_25 1134   //laboratory measurement with isolation circuit, PH meter V2.0 and PH probe from DFRobot kit
#define PH_6_VOLTAGE 1250 //linear culculation
#define PH_5_VOLTAGE 1380 //linear culculation
#define PH_4_AT_25 1521   //laboratory measurement with isolation circuit, PH meter V2.0 and PH probe from DFRobot kit
#define PH_3_VOLTAGE 1700 //linear culculation

#define PH_VOLTAGE_NEUTRAL_LOW_LIMIT PH_8_VOLTAGE - PH_VOLTAGE_NEUTRAL_OFFSET
#define PH_VOLTAGE_NEUTRAL_HIGH_LIMIT PH_6_VOLTAGE
#define PH_VOLTAGE_ACID_LOW_LIMIT PH_5_VOLTAGE - PH_VOLTAGE_ACID_OFFSET
#define PH_VOLTAGE_ACID_HIGH_LIMIT PH_3_VOLTAGE

#define ReceivedBufferLength 10 //length of the Serial CMD buffer

class DFRobot_ESP_EC_PH
{
public:
    DFRobot_ESP_EC_PH();
    ~DFRobot_ESP_EC_PH();
    void ECcalibration(float voltage, float temperature, char *cmd); //calibration by Serial CMD
    void ECcalibration(float voltage, float temperature);
    float readEC(float voltage, float temperature); // voltage to EC value, with temperature compensation
    void PHcalibration(float voltage, float temperature, char *cmd); //calibration by Serial CMD
    void PHcalibration(float voltage, float temperature);
    void update();
    void nutrientpump();
    float readPH(float voltage, float temperature);   // voltage to pH value, with temperature compensation
    void begin(int ECEepromStartAddress = KVALUEADDR, int PHEepromStartAddress = PHVALUEADDR); //initialization
    // boolean isECCalibrated();    
    // boolean isPHCalibrated();
    int isCalibrated();
    int getOnTime();
    int getOffTime();
    bool ecphcontrol();
    int pumpgetOnTime();
    int pumpgetOffTime();
    bool ispumpSet();

private:
    float _ecvalue;
    float  _kvalue;
    float  _kvalueLow;
    float  _kvalueHigh;
    float  _ecvoltage;
    float  _temperature;
    float  _rawEC;
    boolean _eccalibrated;
    float _phValue;
    float _acidVoltage;
    float _neutralVoltage;
    float _phvoltage;
    boolean _phcalibrated;
    int onTime;
    int offTime;
    bool customBlink;
    int onmode = 0;
    int offmode = 0;

    int nonTime;
    int noffTime;
    bool ncustomBlink;
    int nonmode = 0;
    int noffmode = 0;



    char _cmdReceivedBuffer[ReceivedBufferLength]; //store the Serial CMD
    byte _cmdReceivedBufferIndex;

private:
    int _eceepromStartAddress;
    int _pheepromStartAddress;
    boolean cmdSerialDataAvailable();
    void Calibration(byte mode); // calibration process, wirte key parameters to EEPROM
    byte cmdParse(const char *cmd);
    byte cmdParse();
};

#endif
