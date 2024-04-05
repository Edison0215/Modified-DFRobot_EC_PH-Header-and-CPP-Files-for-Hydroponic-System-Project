/*
 * file DFRobot_ESP_EC.cpp * @ https://github.com/GreenPonik/DFRobot_ESP_EC_BY_GREENPONIK
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
#include "Arduino.h"
#include "DFRobot_ESP_EC_PH.h"
#include "EEPROM.h"

#define RES2 820.0
#define ECREF 200.0

int cal1 = 0; //detection flag for 1.413us/cm buffer solution -> detected (1), uncalibrated (0)
int cal2 = 0; //detection flag for 12.88us/cm buffer solution -> detected (1), uncalibrated (0)
int cal3 = 0; //detection flag for PH 7.00 buffer solution -> detected (1), uncalibrated (0)
int cal4 = 0; //detection flag for PH 4.00 buffer solution -> detected (1), uncalibrated (0)
int calmode = 0; //flag for PH (2), EC (1) and non-calibration mode (0)
DFRobot_ESP_EC_PH::DFRobot_ESP_EC_PH()
{
//----- EC Variables Declaration -----
    this->_ecvalue = 0.0;
    this->_kvalue = 1.0;
    this->_kvalueLow = 1.0;
    this->_kvalueHigh = 1.0;
    this->_cmdReceivedBufferIndex = 0;
    this->_ecvoltage = 0.0;
    this->_eccalibrated = false;

//----- PH Variables Declaration -----
    this->_phValue = 7.0;
    this->_acidVoltage = PH_4_AT_25;    //buffer solution 4.0 at 25C
    this->_neutralVoltage = PH_7_AT_25; //buffer solution 7.0 at 25C
    this->_phvoltage = PH_7_AT_25;
    this->_phcalibrated = false; // Initialize the calibration status as false 

//----- Others -----  
    this->_temperature = 25.0; 
    onTime = 0;
    offTime = 0;
    nonTime = 0;
    noffTime = 0;
    
    customBlink = false;
    ncustomBlink = false;
}

DFRobot_ESP_EC_PH::~DFRobot_ESP_EC_PH()
{
}

void DFRobot_ESP_EC_PH::begin(int ECEepromStartAddress, int PHEepromStartAddress)
{
//------ EC Sensor Initialization ------
    this->_eceepromStartAddress = ECEepromStartAddress;
    this->_pheepromStartAddress = PHEepromStartAddress;
    //check if calibration values (kvalueLow and kvalueHigh) are stored in eeprom
    this->_kvalueLow = EEPROM.readFloat(this->_eceepromStartAddress); //read the calibrated K value from EEPROM
    if (this->_kvalueLow == float() || isnan(this->_kvalueLow))
    {
        this->_kvalueLow = 1.0; // For new EEPROM, write default value( K = 1.0) to EEPROM
        EEPROM.writeFloat(this->_eceepromStartAddress, this->_kvalueLow);
        EEPROM.commit();
    }

    this->_kvalueHigh = EEPROM.readFloat(this->_eceepromStartAddress + (int)sizeof(float)); //read the calibrated K value from EEPROM
    if (this->_kvalueHigh == float() || isnan(this->_kvalueHigh))
    {
        this->_kvalueHigh = 1.0; // For new EEPROM, write default value( K = 1.0) to EEPROM
        EEPROM.writeFloat(this->_eceepromStartAddress + (int)sizeof(float), this->_kvalueHigh);
        EEPROM.commit();
    }
    this->_kvalue = this->_kvalueLow; // set default K value: K = kvalueLow

//------- PH Sensor Initialization ------
    //check if calibration values (neutral and acid) are stored in eeprom
    this->_neutralVoltage = EEPROM.readFloat(this->_pheepromStartAddress); //load the neutral (pH = 7.0) voltage of the pH board from the EEPROM
    if (this->_neutralVoltage == float() || isnan(this->_neutralVoltage) || isinf(this->_neutralVoltage))
    {
        this->_neutralVoltage = PH_7_AT_25; // new EEPROM, write typical voltage
        EEPROM.writeFloat(this->_pheepromStartAddress, this->_neutralVoltage);
        EEPROM.commit();
    }

    this->_acidVoltage = EEPROM.readFloat(this->_pheepromStartAddress + (int)sizeof(float)); //load the acid (pH = 4.0) voltage of the pH board from the EEPROM
    if (this->_acidVoltage == float() || isnan(this->_acidVoltage) || isinf(this->_acidVoltage))
    {
        this->_acidVoltage = PH_4_AT_25; // new EEPROM, write typical voltage
        EEPROM.writeFloat(this->_pheepromStartAddress + (int)sizeof(float), this->_acidVoltage);
        EEPROM.commit();
    }

}

float DFRobot_ESP_EC_PH::readEC(float voltage, float temperature)
{
    float value = 0, valueTemp = 0;
    this->_rawEC = 1000 * voltage / RES2 / ECREF;
    //Serial.print(F(">>>rawEC: "));
    //Serial.println(this->_rawEC, 4);
    valueTemp = this->_rawEC * this->_kvalue;
    //automatic shift process
    //First Range:(0,2); Second Range:(2,20)
    if (valueTemp > 2.5)
    {
        this->_kvalue = this->_kvalueHigh;
    }
    else if (valueTemp < 2.0)
    {
        this->_kvalue = this->_kvalueLow;
    }

    value = this->_rawEC * this->_kvalue;                  //calculate the EC value after automatic shift
    value = value / (1.0 + 0.0185 * (temperature - 25.0)); //temperature compensation
    this->_ecvalue = value;                                //store the EC value for Serial CMD calibration
    //Serial.print(F(", ecValue: "));
    //Serial.print(this->_ecvalue, 4);
    //Serial.println(F("<<<"));
    return this->_ecvalue;
}

float DFRobot_ESP_EC_PH::readPH(float voltage, float temperature)
{
    // Serial.print("[readPH]... _neutraVoltage:");
    // Serial.print(this->_neutralVoltage);
    // Serial.print(", _acidVoltage:");
    // Serial.print(this->_acidVoltage);
    float slope = (7.0 - 4.0) / ((this->_neutralVoltage - PH_7_AT_25) / 3.0 - (this->_acidVoltage - PH_7_AT_25) / 3.0); // two point: (_neutralVoltage,7.0),(_acidVoltage,4.0)
    float intercept = 7.0 - slope * (this->_neutralVoltage - PH_7_AT_25) / 3.0;
    // Serial.print(", slope:");
    // Serial.print(slope);
    // Serial.print(", intercept:");
    // Serial.println(intercept);
    this->_phValue = slope * (voltage - PH_7_AT_25) / 3.0 + intercept; //y = k*x + b
    //Serial.print(F(">>>phValue "));
    //Serial.print(this->_phValue, 4);
    //Serial.println(F("<<<"));
    return this->_phValue;
}

void DFRobot_ESP_EC_PH::ECcalibration(float voltage, float temperature, char *cmd)
{
    this->_ecvoltage = voltage;
    this->_temperature = temperature;
    strupr(cmd);
    Calibration(cmdParse(cmd)); // if received Serial CMD from the serial monitor, enter into the calibration mode
}

void DFRobot_ESP_EC_PH::ECcalibration(float voltage, float temperature)
{
    this->_ecvoltage = voltage;
    this->_temperature = temperature;
    if (cmdSerialDataAvailable() > 0)
    {
        Calibration(cmdParse()); // if received Serial CMD from the serial monitor, enter into the calibration mode
    }
}

void DFRobot_ESP_EC_PH::PHcalibration(float voltage, float temperature, char *cmd)
{
    this->_phvoltage = voltage;
    this->_temperature = temperature;
    strupr(cmd);
    Calibration(cmdParse(cmd)); // if received Serial CMD from the serial monitor, enter into the calibration mode
}

void DFRobot_ESP_EC_PH::PHcalibration(float voltage, float temperature)
{
    this->_phvoltage = voltage;
    this->_temperature = temperature;
    if (cmdSerialDataAvailable() > 0)
    {
        Calibration(cmdParse()); // if received Serial CMD from the serial monitor, enter into the calibration mode
    }
}

void DFRobot_ESP_EC_PH::update()
{
    if (cmdSerialDataAvailable() > 0)
    {
        Calibration(cmdParse()); // if received Serial CMD from the serial monitor, enter into the calibration mode
    }
}

void DFRobot_ESP_EC_PH::nutrientpump()
{
    if (cmdSerialDataAvailable() > 0)
    {
        Calibration(cmdParse()); // if received Serial CMD from the serial monitor, enter into the calibration mode
    }
}

boolean DFRobot_ESP_EC_PH::cmdSerialDataAvailable()
{
    char cmdReceivedChar;
    static unsigned long cmdReceivedTimeOut = millis();
    while (Serial.available() > 0)
    {
        if (millis() - cmdReceivedTimeOut > 500U)
        {
            this->_cmdReceivedBufferIndex = 0;
            memset(this->_cmdReceivedBuffer, 0, (ReceivedBufferLength));
        }
        cmdReceivedTimeOut = millis();
        cmdReceivedChar = Serial.read();
        if (cmdReceivedChar == '\n' || this->_cmdReceivedBufferIndex == ReceivedBufferLength - 1)
        {
            this->_cmdReceivedBufferIndex = 0;
            strupr(this->_cmdReceivedBuffer);
            return true;
        }
        else
        {
            this->_cmdReceivedBuffer[this->_cmdReceivedBufferIndex] = cmdReceivedChar;
            this->_cmdReceivedBufferIndex++;
        }
    }
    return false;
}

byte DFRobot_ESP_EC_PH::cmdParse(const char *cmd)
{
    byte modeIndex = 0;
    if (strstr(cmd, "ENTEREC") != NULL)
    {
        modeIndex = 1;
    }
    else if (strstr(cmd, "EXITEC") != NULL)
    {
        modeIndex = 3;
    }
    else if (strstr(cmd, "CALEC") != NULL)
    {    
        modeIndex = 2;
    }
    else if (strstr(cmd, "ENTERPH") != NULL)
    {
        modeIndex = 4;
    }
    else if (strstr(cmd, "EXITPH") != NULL)
    {
        modeIndex = 6;
    }
    else if (strstr(cmd, "CALPH") != NULL)
    {
        modeIndex = 5;
    }
    else if (strstr(cmd, "ECPHDOWN") != NULL)
    {
        modeIndex = 7;
    }
    else if (strstr(cmd, "ECPHUP") != NULL)
    {
        modeIndex = 8;
    }
    return modeIndex;
}

byte DFRobot_ESP_EC_PH::cmdParse()
{
    byte modeIndex = 0;
    if (strstr(this->_cmdReceivedBuffer, "ENTEREC") != NULL)
    {
        modeIndex = 1;
    }
    else if (strstr(this->_cmdReceivedBuffer, "EXITEC") != NULL)
    {
        modeIndex = 3;
    }
    else if (strstr(this->_cmdReceivedBuffer, "CALEC") != NULL)
    {
        modeIndex = 2;
    }
    else if (strstr(this->_cmdReceivedBuffer, "ENTERPH") != NULL)
    {
        modeIndex = 4;
    }
    else if (strstr(this->_cmdReceivedBuffer, "EXITPH") != NULL)
    {
        modeIndex = 6;
    }
    else if (strstr(this->_cmdReceivedBuffer, "CALPH") != NULL)
    {
        modeIndex = 5;
    }
    else if (strstr(this->_cmdReceivedBuffer, "ECPHDOWN") != NULL)
    {
        modeIndex = 7;
    }
    else if (strstr(this->_cmdReceivedBuffer, "ECPHUP") != NULL)
    {
        modeIndex = 8;
    }
    return modeIndex;
}

void DFRobot_ESP_EC_PH::Calibration(byte mode)
{
    char *receivedBufferPtr;
    static boolean ecCalibrationFinish = 0;
    static boolean ecenterCalibrationFlag = 0;
    static boolean phCalibrationFinish = 0;
    static boolean phenterCalibrationFlag = 0;
    static boolean lightonset = 0;
    static boolean lightonsetfinish = 0;
    static boolean lightoffset = 0;
    static boolean lightoffsetfinish = 0;
    static boolean pumponset = 0;
    static boolean pumponsetfinish = 0;
    static boolean pumpoffset = 0;
    static boolean pumpoffsetfinish = 0;

    static float compECsolution;
    float KValueTemp;
    static boolean phcalibrated, eccalibrated;
    int parsedOffTime;
    int parsedOnTime;
    int nparsedOnTime;
    int nparsedOffTime;
    switch (mode)
    {
    case 0:
        if (ecenterCalibrationFlag || phenterCalibrationFlag)
        {
            Serial.println(F(">>>Command Error<<<"));
            //this->_eccalibrated = false; // Calibration failed, set _calibrated to false
            this->_phcalibrated = false; // Calibration failed, set _calibrated to false
            ecenterCalibrationFlag = 0;
            phenterCalibrationFlag = 0;
            calmode = 0; //uncalibration mode
        }
        break;

    case 1://ENTEREC prompt
        ecenterCalibrationFlag = 1;
        ecCalibrationFinish = 0;
        calmode = 1; //PH calibration mode triggered
        if (!phenterCalibrationFlag || !lightoffset || !lightonset || !pumponset || !pumpoffset){ //when the next input prompt is not "ENTERPH" or "ONLIGHT" or "OFFLIGHT"
        Serial.println();
        Serial.println(F(">>>Enter EC Calibration Mode<<<"));
        Serial.println(F(">>>Please put the probe into the 1413us/cm or 2.76ms/cm or 12.88ms/cm buffer solution<<<"));
        Serial.println(F(">>>Only need two point for calibration one low (1413us/com) and one high(2.76ms/cm or 12.88ms/cm)<<<"));
        Serial.println();
        this->_eccalibrated = false; // EC calibration failed
        }
        else{ //when the next input prompt is "ENTERPH" or "ONLIGHT" or "OFFLIGHT"
            calmode = 0;
            phenterCalibrationFlag = 0;
            phCalibrationFinish = 0;
            ecCalibrationFinish = 0;
            ecenterCalibrationFlag = 0;
            pumponset = 0;
            pumponsetfinish = 0;
            pumpoffsetfinish = 0;
            pumpoffset = 0;
            lightonset = 0;
            lightonsetfinish = 0;
            lightoffset = 0;
            lightoffsetfinish = 0;
            Serial.println(F(">>>Multiple calibration command detected.<<<"));
        }

        break;

    case 2://CALEC prompt
        if (ecenterCalibrationFlag && calmode == 1) //after "ENTEREC" is prompted
        {
            if ((this->_rawEC > RAWEC_1413_LOW) && (this->_rawEC < RAWEC_1413_HIGH))
            {
                Serial.print(F(">>>Buffer 1.413ms/cm<<<"));                            //recognize 1.413us/cm buffer solution
                compECsolution = 1.413 * (1.0 + 0.0185 * (this->_temperature - 25.0)); //temperature compensation
                Serial.print(F(">>>compECsolution: "));
                Serial.print(compECsolution);
                Serial.println(F("<<<"));
                cal1 = 1; //1.413us/cm buffer detection flag triggered
            }
            else if ((this->_rawEC > RAWEC_276_LOW) && (this->_rawEC < RAWEC_276_HIGH))
            {
                Serial.print(F(">>>Buffer 2.76ms/cm<<<"));                            //recognize 2.76ms/cm buffer solution
                compECsolution = 2.76 * (1.0 + 0.0185 * (this->_temperature - 25.0)); //temperature compensation
                Serial.print(F(">>>compECsolution: "));
                Serial.print(compECsolution);
                Serial.println(F("<<<"));
                cal2 = 1; //2.76ms/cm buffer detection flag triggered
            }
            else if ((this->_rawEC > RAWEC_1288_LOW) && (this->_rawEC < RAWEC_1288_HIGH))
            {
                Serial.print(F(">>>Buffer 12.88ms/cm<<<"));                            //recognize 12.88ms/cm buffer solution
                compECsolution = 12.88 * (1.0 + 0.0185 * (this->_temperature - 25.0)); //temperature compensation
                Serial.print(F(">>>compECsolution: "));
                Serial.print(compECsolution);
                Serial.println(F("<<<"));
                cal2 = 1; //12.88ms/cm buffer detection flag triggered
            }
            else
            {
                Serial.print(F(">>>Buffer Solution Error Try Again<<<   "));
                ecCalibrationFinish = 0;
                cal1 = 0; //deactivate buffer detection flag
                cal2 = 0; //deactivate buffer detection flag
                //user can prompt "CALEC" to retry EC calibration since ecenterCalibrationFlag is still HIGH
            }
            Serial.println();
            Serial.print(F(">>>KValueTemp calculation formule: "));
            Serial.print(F("RES2"));
            Serial.print(F(" * "));
            Serial.print(F("ECREF"));
            Serial.print(F(" * "));
            Serial.print(F("compECsolution"));
            Serial.print(F(" / 1000.0 / "));
            Serial.print(F("voltage"));
            Serial.println(F("<<<"));
            Serial.print(F(">>>KValueTemp calculation: "));
            Serial.print(RES2);
            Serial.print(F(" * "));
            Serial.print(ECREF);
            Serial.print(F(" * "));
            Serial.print(compECsolution);
            Serial.print(F(" / 1000.0 / "));
            Serial.print(this->_ecvoltage);
            Serial.println(F("<<<"));
            KValueTemp = RES2 * ECREF * compECsolution / 1000.0 / this->_ecvoltage; //calibrate the k value
            Serial.println();
            Serial.print(F(">>>KValueTemp: "));
            Serial.print(KValueTemp);
            Serial.println(F("<<<"));
            if ((KValueTemp > 0.5) && (KValueTemp < 2.0))
            {
                Serial.println();
                Serial.print(F(">>>Successful,K:"));
                Serial.print(KValueTemp);
                Serial.println(F(", Send EXITEC to Save and Exit<<<"));
                if ((this->_rawEC > RAWEC_1413_LOW) && (this->_rawEC < RAWEC_1413_HIGH))
                {
                    this->_kvalueLow = KValueTemp;
                    Serial.print(">>>kvalueHigh: ");
                    Serial.print(this->_kvalueLow);
                    Serial.println(F("<<<"));
                }
                else if ((this->_rawEC > RAWEC_276_LOW) && (this->_rawEC < RAWEC_276_HIGH))
                {
                    this->_kvalueHigh = KValueTemp;
                    Serial.print(">>>kvalueHigh: ");
                    Serial.print(this->_kvalueHigh);
                    Serial.println(F("<<<"));
                }
                else if ((this->_rawEC > RAWEC_1288_LOW) && (this->_rawEC < RAWEC_1288_HIGH))
                {
                    this->_kvalueHigh = KValueTemp;
                    Serial.print(">>>kvalueHigh: ");
                    Serial.print(this->_kvalueHigh);
                    Serial.println(F("<<<"));
                }
                ecCalibrationFinish = 1;
            }
            else
            {
                Serial.println();
                Serial.println(F(">>>KValueTemp out of range 0.5-2.0<<<"));
                Serial.print(">>>KValueTemp: ");
                Serial.print(KValueTemp, 4);
                Serial.println("<<<");
                Serial.println(F(">>>Failed,Try Again<<<"));
                Serial.println();
                ecCalibrationFinish = 0;
                this->_eccalibrated = false; //Failed EC calibration
            }
        }
        else {
            Serial.println(">>Wrong CAL command detected.<<<");
            calmode = 0;
        }
        break;
    case 3://"EXITEC" prompt
        if (ecenterCalibrationFlag && calmode == 1)
        {
            Serial.println();
            if (ecCalibrationFinish)
            {
                if ((this->_rawEC > RAWEC_1413_LOW) && (this->_rawEC < RAWEC_1413_HIGH))
                {
                    EEPROM.writeFloat(this->_eceepromStartAddress, this->_kvalueLow);
                    EEPROM.commit();
                }
                else if ((this->_rawEC > RAWEC_276_LOW) && (this->_rawEC < RAWEC_276_HIGH))
                {
                    EEPROM.writeFloat(this->_eceepromStartAddress + (int)sizeof(float), this->_kvalueHigh);
                    EEPROM.commit();
                }
                else if ((this->_rawEC > RAWEC_1288_LOW) && (this->_rawEC < RAWEC_1288_HIGH))
                {
                    EEPROM.writeFloat(this->_eceepromStartAddress + (int)sizeof(float), this->_kvalueHigh);
                    EEPROM.commit();
                }
                Serial.print(F(">>>Calibration Successful"));
            }
            else
            {
                Serial.print(F(">>>Calibration Failed"));
                //this->_eccalibrated = false; // Calibration is successful, set _calibrated to true
            }

            Serial.println(F(",Exit EC Calibration Mode<<<"));
            Serial.println();
            ecCalibrationFinish = 0;
            ecenterCalibrationFlag = 0;
            if (cal1 == 1 and cal2 ==1){ //2 different buffer solution has been detected and calibrated
            this->_eccalibrated = true; //Successful EC calibration
            cal1 =0; //deactivate buffer detection flag 
            cal2=0; //detect buffer solution flag
            calmode = 0; //back to uncalibrated mode 

            }
            else { //only one or no buffer solution has been detected or calibrated
                cal1 = 0; //deactivate buffer detection flag
                cal2 = 0; //deactivate buffer detection flag
                calmode =0; //back to uncalibrated mode
                this->_eccalibrated = false; // Failed EC calibration
            }
        }
        else {
            Serial.println(">>>Wrong EXIT command detected.<<<");
            calmode = 0;
        }
        break;

    case 4: //"ENTERPH" prompt
        phenterCalibrationFlag = 1;
        phCalibrationFinish = 0;
        calmode = 2; //PH calibration mode
        if (!ecenterCalibrationFlag || !lightoffset || !lightonset || !pumponset || !pumpoffset){ //when "ENTEREC" is not prompted
        Serial.println();
        Serial.println(F(">>>Enter PH Calibration Mode<<<"));
        Serial.println(F(">>>Please put the probe into the 4.0 or 7.0 standard buffer solution<<<"));
        Serial.println();
        this->_phcalibrated = false; // Calibration failed, set _calibrated to false
        }
        else { //when "ENTEREC" is prompted
            calmode = 0;
            phenterCalibrationFlag = 0;
            phCalibrationFinish = 0;
            ecCalibrationFinish = 0;
            ecenterCalibrationFlag = 0;
            pumponset = 0;
            pumponsetfinish = 0;
            pumpoffsetfinish = 0;
            pumpoffset = 0;
            lightonset = 0;
            lightonsetfinish = 0;
            lightoffset = 0;
            lightoffsetfinish = 0;
            Serial.println(F(">>>Multiple command detected.<<<"));
        }
        break;

    case 5: //"CALPH" prompt
        if (phenterCalibrationFlag && calmode == 2)
        {
            // buffer solution:7.0
            // 7795 to 1250
            if ((this->_phvoltage > PH_VOLTAGE_NEUTRAL_LOW_LIMIT) && (this->_phvoltage < PH_VOLTAGE_NEUTRAL_HIGH_LIMIT))
            {
                Serial.println();
                Serial.print(F(">>>Buffer Solution:7.0"));
                this->_neutralVoltage = this->_phvoltage;
                Serial.println(F(",Send EXITPH to Save and Exit<<<"));
                Serial.println();
                phCalibrationFinish = 1;
                cal3 = 1; //buffer solution 1 detected
            }
            //buffer solution:4.0
            //1180 to 1700
            else if ((this->_phvoltage > PH_VOLTAGE_ACID_LOW_LIMIT) && (this->_phvoltage < PH_VOLTAGE_ACID_HIGH_LIMIT))
            {
                Serial.println();
                Serial.print(F(">>>Buffer Solution:4.0"));
                this->_acidVoltage = this->_phvoltage;
                Serial.println(F(",Send EXITPH to Save and Exit<<<"));
                Serial.println();
                phCalibrationFinish = 1;
                cal4 = 1; //buffer solution 2 detected
            }
            else
            {
                Serial.println();
                Serial.print(F(">>>Buffer Solution Error Try Again<<<"));
                Serial.println(); // not buffer solution or faulty operation
                phCalibrationFinish = 0;
                //user can prompt "CALPH" to retry PH calibration since phenterCalibrationFlag is still HIGH
            }
        }
        else {
            Serial.println(">>Wrong CAL command detected.<<<");
            calmode = 0;
        }
        break;

    case 6: //EXITPH prompt
        if (phenterCalibrationFlag && calmode == 2)
        {
            Serial.println();
            if (phCalibrationFinish)
            {
                // buffer solution:7.0
                // 7795 to 1250
                if ((this->_phvoltage > PH_VOLTAGE_NEUTRAL_LOW_LIMIT) && (this->_phvoltage < PH_VOLTAGE_NEUTRAL_HIGH_LIMIT))
                {
                    EEPROM.writeFloat(this->_pheepromStartAddress, this->_neutralVoltage);
                    EEPROM.commit();
                }
                //buffer solution:4.0
                //1180 to 1700
                else if ((this->_phvoltage > PH_VOLTAGE_ACID_LOW_LIMIT) && (this->_phvoltage < PH_VOLTAGE_ACID_HIGH_LIMIT))
                {
                    EEPROM.writeFloat(this->_pheepromStartAddress + (int)sizeof(float), this->_acidVoltage);
                    EEPROM.commit();
                }
                Serial.print(F(">>>Calibration Successful"));
            }
            else
            {
                Serial.print(F(">>>Calibration Failed"));
                this->_phcalibrated = false; // Failed PH calibration
                //phcalibrated = 0;
            }
            Serial.println(F(",Exit PH Calibration Mode<<<"));
            Serial.println();
            phCalibrationFinish = 0;
            phenterCalibrationFlag = 0;
            if (cal3 == 1 and cal4 ==1){ //when two buffer solutions are detected and calibrated
                this->_phcalibrated = true; // Calibration is successful
                calmode = 0;
                cal3=0;
                cal4=0;
            }
            else {//if only one or no buffer solution is detected
                cal3 = 0; //buffer detection flag reset
                cal4 = 0; //buffer detection flag reset
                calmode =0; //back to uncalibrated mode
                this->_phcalibrated = false; // Failed PH calibration
            }
            
        }
        else {
            Serial.println(">>>Wrong EXIT command detected.<<<");
            calmode = 0;
        }
        break;
    
    case 7:
        customBlink = true;
        break;
    case 8:
        customBlink = false;
        break;
    case 9:
    break;

    case 10:
        pumponset = 1;
        pumponsetfinish = 0;
        calmode = 4;
        if ((phCalibrationFinish == 0 && phenterCalibrationFlag == 0) && (ecCalibrationFinish == 0 && ecenterCalibrationFlag == 0) && (lightonset == 0 && lightoffset == 0)){
        Serial.println(">>>NUTRIENT PUMP: Enter the on time (in milliseconds)<<<");
        while (!Serial.available()) {}
        nparsedOnTime = Serial.parseInt();
        if (nparsedOnTime != 0){
            if (nparsedOnTime>= 0 || nparsedOnTime == 0){
                
                Serial.print(">>>NUTRIENT PUMP: Nutrient pump on duration of ");
                Serial.print(nparsedOnTime);
                Serial.println(" is set successfully.<<<");
                nonTime = nparsedOnTime;
                nonmode = 1;
            }
            else{
                Serial.println(">>>NUTRIENT PUMP: Invalid input. Pump on duration setup exited.<<<");
                nonmode = 0;
            }
        }
        else{
            Serial.println(">>>NUTRIENT PUMP: Invalid input. Pump on duration setup exited.<<<");
            nonmode = 0;
            calmode = 0;
        }
        ncustomBlink = false;
        }  
        else { //when "ENTEREC" is prompted
            calmode = 0; //back to uncalibrated mode
            phenterCalibrationFlag = 0;
            phCalibrationFinish = 0;
            ecCalibrationFinish = 0;
            ecenterCalibrationFlag = 0;
            pumponset = 0;
            pumponsetfinish = 0;
            pumpoffsetfinish = 0;
            pumpoffset = 0;
            lightonset = 0;
            lightonsetfinish = 0;
            lightoffset = 0;
            lightoffsetfinish = 0;
            Serial.println(F(">>>Multiple command detected.<<<"));
        } 
        break;

    case 11:
        pumpoffset = 1;
        pumpoffsetfinish = 0;
        calmode = 4;
        if ((phCalibrationFinish == 0 && phenterCalibrationFlag == 0) && (ecCalibrationFinish == 0 && ecenterCalibrationFlag == 0) && (lightonset == 0 && lightoffset == 0)){
            Serial.println("NUTRIENT PUMP: Enter the off time (in milliseconds): ");
            while (!Serial.available()) {}
            nparsedOffTime = Serial.parseInt();
            if (nparsedOffTime != 0){
                if (nparsedOffTime>= 0){
                    Serial.print(">>>NUTRIENT PUMP: Nutrient pump off duration of ");
                    Serial.print(nparsedOffTime);
                    Serial.println(" is set successfully.<<<");
                    noffTime = nparsedOffTime;
                    noffmode = 1;
                }
                else{
                    Serial.println(">>>NUTRIENT PUMP: Invalid input. Nutrient pump off duration setup exited without data saved.<<<");
                    noffmode = 0;
                }
            }
            else{
                Serial.println(">>>NUTRIENT PUMP: Invalid input. Nutrient pump off duration setup exited without data saved.<<<");
                noffmode = 0;
            }
            ncustomBlink = false;
            }
            else { //when "ENTEREC" is prompted
            calmode = 0; //back to uncalibrated mode
            phenterCalibrationFlag = 0;
            phCalibrationFinish = 0;
            ecCalibrationFinish = 0;
            ecenterCalibrationFlag = 0;
            pumponset = 0;
            pumponsetfinish = 0;
            pumpoffsetfinish = 0;
            pumpoffset = 0;
            lightonset = 0;
            lightonsetfinish = 0;
            lightoffset = 0;
            lightoffsetfinish = 0;
            Serial.println(F(">>>Multiple command detected.<<<"));
            } 
        break;
    
    case 12:
      if (nonmode == 1 and noffmode ==1 && calmode == 4){
        Serial.println(">>>Nutrient pump on and off duration are set successfully.<<<");
        Serial.println(">>>Nutrient pump setup exited successfully.<<<");
        nonmode = 0;
        noffmode = 0;
        calmode = 0;
        pumponset = 0;
        pumponsetfinish = 1;
        pumpoffset = 0;
        pumpoffsetfinish = 1;
        ncustomBlink = true;
      }
      else {
        if (calmode == 4){
        Serial.println(">>>Nutrient pump setup exited unsuccessfully. Reset to zero.<<<");
        ncustomBlink = false;
        nonmode = 0;
        noffmode = 0;
        calmode = 0;
        pumponset = 0;
        pumponsetfinish = 1;
        pumpoffset = 0;
        pumpoffsetfinish = 1;
        }
        else {
            Serial.println(">>>Wrong EXIT command detected.<<<");
            calmode = 0;
        }
      }
        break;
    }   
}

int DFRobot_ESP_EC_PH::isCalibrated()
{
    int calibrated;
    if (this->_phcalibrated && this->_eccalibrated)
    {
        calibrated = 0; //ph and ec are calibrated
    } 
    else if (!this->_phcalibrated && this->_eccalibrated)
    {
        calibrated = 1; //ph not calibrated, ec calibrated
    }
    else if (this->_phcalibrated && !this->_eccalibrated)
    {
        calibrated = 2; //ph calibrated, ec not calibrated
    }
    else {
        calibrated = 3; //ph and ec not calibrated
    }
    return calibrated; // Return the calibration status
}

// boolean DFRobot_ESP_EC_PH::isPHCalibrated()
// {
//     return _phcalibrated; // Return the calibration status
// }

// //use case!!

int DFRobot_ESP_EC_PH::getOnTime() {
  return onTime;
}

int DFRobot_ESP_EC_PH::getOffTime() {
  return offTime;
}

bool DFRobot_ESP_EC_PH::ecphcontrol() {
  return customBlink;
}

int DFRobot_ESP_EC_PH::pumpgetOnTime() {
  return nonTime;
}

int DFRobot_ESP_EC_PH::pumpgetOffTime() {
  return noffTime;
}

bool DFRobot_ESP_EC_PH::ispumpSet() {
  return ncustomBlink;
}