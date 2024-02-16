// Nikunj Parasar 
// Created: 10/20/2023
// GAUCHO RACING CAN NODES SOFTWARE (FLEXCAN_T4)
// This file contains the CAN nodes for the GR24 EV

// //////////////////////////////////////////////////////////////////////ADJUST UNITS LATER AS NEEDED
// https://docs.google.com/spreadsheets/d/1XfJhhAQoDnuSuwluNitPsDWtuQu-bP-VbEGPmSo5ujA/edit#gid=1132363474
#ifndef NODES
#define NODES

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SPI.h>
#include <stdio.h>
#include <stdlib.h>

#define USE_CAN_PRIMARY
// #define USE_CAN_DATA  


#if defined(USE_CAN_PRIMARY) && !defined(USE_CAN_DATA)
    #define CAN_PRIMARY_BUS CAN1
    #define CAN_DATA_BUS CAN2  // Default to CAN2 if CAN_PRIMARY_BUS is selected
#elif !defined(USE_CAN_PRIMARY) && defined(USE_CAN_DATA)
    #define CAN_PRIMARY_BUS CAN2
    #define CAN_DATA_BUS CAN2
#else
    #error "Please define either USE_CAN_PRIMARY or USE_CAN_DATA"
#endif


/*
 _ _______ _     _ _______ ______ _______ _______ ______  
| (_______|_)   (_|_______|_____ (_______|_______|_____ \ 
| |_     _ _     _ _____   _____) )  _    _____   _____) )
| | |   | | |   | |  ___) |  __  /  | |  |  ___) |  __  / 
| | |   | |\ \ / /| |_____| |  \ \  | |  | |_____| |  \ \ 
|_|_|   |_| \___/ |_______)_|   |_| |_|  |_______)_|   |_|
                                                          
*/

struct Inverter {
    byte data[5][8]={{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}; 
    
    /*
    example data packets from CAN bus:
    ---id ------------ byte ---------------
           | 0              | 1             | 2             | 3             | 4             | 5             | 6             | 7         |
    ----------------------------------------------------------------------------------------------------------------------------------
    0x2016 |                		       ERPM	                            |            Duty Cycle 	    |         Input Voltage	    |
    ----------------------------------------------------------------------------------------------------------------------------------
    0x2116 |             AC Current		    |          DC Current	        |	                        RESERVED			            |
    ----------------------------------------------------------------------------------------------------------------------------------
    0x2216 |         Controller Temp		|           Motor Temp		    |    Faults	    |         RESERVED		                    |
    ----------------------------------------------------------------------------------------------------------------------------------
    0x2316 |                         FOC Id		                          	|                       	FOC Iq			                |
    ----------------------------------------------------------------------------------------------------------------------------------
    0x2416 |  Throttle      |	Brake	    |   Digital IO	|  Drive Enable	|     Flags     |	Flags	    | RESERVED	    |CAN Version
    -----------------------------------------------------------------------------------------------------------------------------------------
    */



    unsigned long ID = 0;
    FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> Can1;
    CAN_message_t msg;
    unsigned long receiveTime = 0;

    Inverter(unsigned long id, FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> &can) : ID(id){
        can = Can1;
    }

    void receive(unsigned long id, byte buf[]){
        if(id >= 0x2016 && id <= 0x2416){
            byte digit2 = (id >> 4) & 0xF; // 0 for 0x2016, 1 for 0x2116, 2 for 0x2216, 3 for 0x2316, 4 for 0x2416
            receiveTime = millis();
            for(int i = 0; i < 8; i++) data[digit2][i] = buf[i];                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
            return;
        }
        else{
            Serial.print(id, HEX);
            Serial.println(" is not data from inverter");
        }
    }

    void send(long OutId, long data, int dataLength){    //Sends 8 bytes with that Id and that data shifted left all the way
        byte stuff[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        for(int i=0; i<dataLength; i++)
            stuff[i] = (data >> ((dataLength-i-1)*8));
        for (int i = 0; i < 8; i++)
            msg.buf[i] = stuff[i];
        msg.id = ((long)OutId << 8) + ID;
        msg.flags.extended=true;
        Can1.write(msg);
    }
    unsigned long getID() {return ID;}

    long getERPM() const {return(((long)data[0][0] << 24) + ((long)data[0][1] << 16) + ((long)data[0][2] << 8) + data[0][3]);} //rpm/pole pairs
    float getDuty() const {return((((long)data[0][4] << 8) + data[0][5])/10);} //i think [0,100]. Related to top speed
    int getVoltIn() const {return(((long)data[0][6] << 8) + data[0][7]);}
    float getACCurrent() const {return((((long)data[1][0] << 8) + data[1][1])/10);}
    float getDCCurrent() const {return(((long)(data[1][2] << 8) + data[1][3])/10);}
    float getInvTemp() const {return((((long)data[2][0] << 8) + data[2][1])/10);} //Deg C
    float getMotorTemp() const {return((((long)data[2][2] << 8) + data[2][3])/10);} //Deg C
    byte getFaults() const {return data[2][4];}
    float getCurrentD() const {return((((long)data[3][0] << 24) + ((long)data[3][1] << 16) + ((long)data[3][2] << 8) + data[3][3])/100);}  //FOC current (don't need)
    float getCurrentQ() const {return((((long)data[3][4] << 24) + ((long)data[3][5] << 16) + ((long)data[3][6] << 8) + data[3][7])/100);}  //FOC current (don't need)
    byte getThrottleIn() const {return data[4][0];}  //Received throttle signal by the invertor
    byte getBrakeIn() const {return data[4][1];}  //Received brake signal by the invertor
    bool getD1() const {return ((data[4][2] & 0x80) == 0x80);}  //Digital input read
    bool getD2() const {return ((data[4][2] & 0x40) == 0x40);}  //Digital input read
    bool getD3() const {return ((data[4][2] & 0x20) == 0x20);}  //Digital input read
    bool getD4() const {return ((data[4][2] & 0x10) == 0x10);}  //Digital input read
    bool getDO1() const {return ((data[4][2] & 0x08) == 0x08);}  //Digital output write
    bool getDO2() const {return ((data[4][2] & 0x04) == 0x04);}  //Digital output write
    bool getDO3() const {return ((data[4][2] & 0x02) == 0x02);}  //Digital output write
    bool getDO4() const {return ((data[4][2] & 0x01) == 0x01);}  //Digital output write
    bool getDriveEnable() const {return ((data[4][3] & 0x01) == 0x01);} //These are setting that can be changed (prob don't need these)
    bool getCapTempLim() const {return ((data[4][4] & 0x80) == 0x80);}//         ^
    bool getDCCurrentLim() const {return ((data[4][4] & 0x40) == 0x40);}//       ^
    bool getDriveEnableLim() const {return ((data[4][4] & 0x20) == 0x20);}//     ^
    bool getIgbtAccelTempLim() const {return ((data[4][4] & 0x10) == 0x10);}//   ^
    bool getIgbtTempLim() const {return ((data[4][4] & 0x08) == 0x08);}//        ^
    bool getVoltInLim() const {return ((data[4][4] & 0x04) == 0x04);}//          ^
    bool getMotorAccelTempLim() const {return ((data[4][4] & 0x02) == 0x02);}//  ^
    bool getMotorTempLim() const {return ((data[4][4] & 0x01) == 0x01);}//       ^
    bool getRPMMinLimit() const {return ((data[4][5] & 0x80) == 0x80);}//        ^
    bool getRPMMaxLimit() const {return ((data[4][5] & 0x40) == 0x40);}//        ^
    bool getPowerLimit() const {return ((data[4][5] & 0x20) == 0x20);}//    

    void setCurrent(float in) {send(0x1A, (long)(in*10), 2);}//            ^
    void setBrakeCurrent(float in) {send(0x1B, (long)(in*10), 2);}//       ^
    void setERPM(long in) {send(0x1C, (long)in, 4);}//                     ^
    void setPosition(float in) {send(0x1D, (long)in, 2);}//                ^
    void setRCurrent(float in) {send(0x1E, (long)(in*10), 2);}//           ^
    void setRBrakeCurrent(float in) {send(0x1F, (long)(in*10), 2);}//      ^
    void setMaxCurrent(float in) {send(0x20, (long)(in*10), 2);}//         ^
    void setMaxBrakeCurrent(float in) {send(0x21, (long)(in*10), 2);}//    ^
    void setMaxDCCurrent(float in) {send(0x22, (long)(in*10), 2);}//       ^
    void setMaxDCBrakeCurrent(float in) {send(0x23, (long)(in*10), 2);}//  ^
    void setDriveEnable(byte in) {send(0x24, (long)in, 1);} //Enable/disable motor
    unsigned long getAge() const {return(millis() - receiveTime);} //time since last data packet
};


/*
 _______ _______ _     _ 
(_______|_______|_)   (_)
 _____   _       _     _ 
|  ___) | |     | |   | |
| |_____| |_____| |___| |
|_______)\______)\_____/ 
                         
*/
struct VDM{    
    byte data[17][8] = {0x00};
    byte dataOut[8] = {0x00};
    unsigned long ID = 0;
    FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> Can1;
    CAN_message_t msg;
    unsigned long receiveTime = 0;

    int getCANHash(int id){
        switch(id){
            case 0x64:
                return 0;
            case 0x65:
                return 1;
            case 0x66:
                return 2;
            case 0xCA:
                return 3;
            case 0xFA:
                return 4;
            case 0x116:
                return 5;
            case 0x216:
                return 6;
            case 0x316:
                return 7;
            case 0x416:
                return 8;
            case 0x516:
                return 9;
            case 0x616:
                return 10;
            case 0x716:
                return 11;
            case 0x816:
                return 12;
            case 0x916:
                return 13;
            case 0xA16:
                return 14;
            case 0xB16:
                return 15;
            case 0xC16:
                return 16;
            default:
                return -1;
        };
    }

    VDM(unsigned long id, FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> &can) : ID(id){
        can = Can1;
    }

    void receive(unsigned long id, byte buf[]){
            if(getCANHash(id) != -1){
                int row = getCANHash(id);
                receiveTime = millis();
                for(int i = 0; i < 8; i++) data[row][i] = buf[i];
                return;
            }
            else{
                Serial.print(id, HEX);
                Serial.println(" is not data from VDM");
            }
    }
    
    unsigned long getID() const {return ID;}
    unsigned long getAge() const {return(millis() - receiveTime);} //time since last data packet

    byte pedalPingRequest() const {return data[3][0];}
    byte getVCU_STATE() const {return data[4][0];}




    //IDK what the other recieving stuff is but its ther in the spreadsheet if needed to be implemented later


};



/*
_  _  _ _     _ _______ _______ _        ______ 
(_)(_)(_|_)   (_|_______|_______|_)      / _____)
 _  _  _ _______ _____   _____   _      ( (____  
| || || |  ___  |  ___) |  ___) | |      \____ \ 
| || || | |   | | |_____| |_____| |_____ _____) )
 \_____/|_|   |_|_______)_______)_______|______/ 
                                                 
*/

// Wheel type
// INITIALIZE WHEEL WITH WHEELTYPE AND I HANDLE THE LOCATION WITHIN THE CONSTRUCTOR
enum HubSensorArray{
    WHEEL_FR, //ids 0x10F00 - 0x10F04
    WHEEL_FL, //ids 0x10F08 - 0x10F0C
    WHEEL_RR, //ids 0x10F10 - 0x10F14
    WHEEL_RL  //ids 0x10F18 - 0x10F1C
};

struct Wheel {
    byte data[5][8] = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  
                {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
                {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  
                {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  
                {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}; 
    HubSensorArray location;

    unsigned long ID = 0;
    FlexCAN_T4<CAN_DATA_BUS, RX_SIZE_256, TX_SIZE_16> Can2;
    CAN_message_t msg;
    unsigned long receiveTime = 0;
    int id_range[2];
    const char* loc_cstr;
    Wheel(unsigned long id, FlexCAN_T4<CAN_DATA_BUS, RX_SIZE_256, TX_SIZE_16> &can, HubSensorArray loc) : ID(id), location(loc){
        can = Can2; //set reference
        switch(location){
            case WHEEL_FR:
                id_range[0] = 0x10F00;
                id_range[1] = 0x10F04;
                loc_cstr = "FR WHEEL HUB";
                break;
            case WHEEL_FL:
                id_range[0] = 0x10F08;
                id_range[1] = 0x10F0C;
                loc_cstr = "FL WHEEL HUB";
                break;
            case WHEEL_RR:
                id_range[0] = 0x10F10;
                id_range[1] = 0x10F14;
                loc_cstr = "RR WHEEL HUB";
                break;
            case WHEEL_RL:
                id_range[0] = 0x10F18;
                id_range[1] = 0x10F1C;
                loc_cstr = "RL WHEEL HUB";
                break;
            default:
                break;
        }
    }
    

    void recieve(unsigned long id, byte buf[]){
        if(id >= id_range[0] && id <= id_range[1]){
            // extract row dpending on id and wheel location
            byte row = (id - id_range[0]);
            receiveTime = millis();
            for(int i = 0; i < 8; i++) data[row][i] = buf[i];
            return;
        }
        else{
            Serial.print(id, HEX);
            Serial.print(" is not data from ");
            Serial.println(loc_cstr);
        }
    }
    
    unsigned long getID() {return ID;}


    float getSuspensionTravel() const {return data[0][0];}
    float getWheelSpeed() const {return((long)data[0][1] << 8) + data[0][2];}
    float getTirePressure() const {return data[0][3];}
    float getIMUAccelX() const {return ((long)data[1][0] << 8) + data[1][1];}
    float getIMUAccelY() const {return ((long)data[1][2] << 8) + data[1][3];}
    float getIMUAccelZ() const {return ((long)data[1][4] << 8) + data[1][5];}
    float getIMUGyroX() const {return ((long)data[2][0] << 8) + data[2][1];}
    float getIMUGyroY() const {return ((long)data[2][2] << 8) + data[2][3];}
    float getIMUGyroZ() const {return ((long)data[2][4] << 8) + data[2][5];}
    byte getBraketemp1() const {return data[3][0];}
    byte getBraketemp2() const {return data[3][1];}
    byte getBraketemp3() const {return data[3][2];}
    byte getBraketemp4() const {return data[3][3];}
    byte getBraketemp5() const {return data[3][4];}
    byte getBraketemp6() const {return data[3][5];}
    byte getBraketemp7() const {return data[3][6];}
    byte getBraketemp8() const {return data[3][7];}
    byte getTireTemp1() const {return data[4][0];}
    byte getTireTemp2() const {return data[4][1];}
    byte getTireTemp3() const {return data[4][2];}
    byte getTireTemp4() const {return data[4][3];}
    byte getTireTemp5() const {return data[4][4];}
    byte getTireTemp6() const {return data[4][5];}
    byte getTireTemp7() const {return data[4][6];}
    byte getTireTemp8() const {return data[4][7];}
    float getAvgBrakeTemp() const {return (getBraketemp1() + getBraketemp2() + getBraketemp3() + getBraketemp4() + getBraketemp5() + getBraketemp6() + getBraketemp7() + getBraketemp8())/8;}
    float getAvgTireTemp() const {return (getTireTemp1() + getTireTemp2() + getTireTemp3() + getTireTemp4() + getTireTemp5() + getTireTemp6() + getTireTemp7() + getTireTemp8())/8;}
    unsigned long getAge() const {return(millis() - receiveTime);} //time since last data packet
    
};



/*

 _____ _______ _     _
   |   |  |  | |     |
 __|__ |  |  | |_____|
*/

struct Central_IMU {
    byte data[3][8] = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  //Accel
                {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //Gyro
                {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}; //Mag

    unsigned long ID = 0;
    FlexCAN_T4<CAN_DATA_BUS, RX_SIZE_256, TX_SIZE_16> Can2;
    CAN_message_t msg;
    unsigned long receiveTime = 0;

    Central_IMU(unsigned long id, FlexCAN_T4<CAN_DATA_BUS, RX_SIZE_256, TX_SIZE_16> &can) : ID(id){
        can = Can2;
    }

    void receive(unsigned long id, byte buf[]){
        if(id >= 0x10F20 && id <= 0x1022){
            byte digit2 = (id - 0x10F20); 
            receiveTime = millis();
            for(int i = 0; i < 8; i++) data[digit2][i] = buf[i];                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
            return;
        }
        else{
            Serial.print(id, HEX);
            Serial.println(" is not data from GPS");
        }
    }

    float getAccelX() const {return ((long)data[0][0] << 8) + data[0][1];}
    float getAccelY() const {return ((long)data[0][2] << 8) + data[0][3];}
    float getAccelZ() const {return ((long)data[0][4] << 8) + data[0][5];}
    float getGyroX() const {return ((long)data[1][0] << 8) + data[1][1];}
    float getGyroY() const {return ((long)data[1][2] << 8) + data[1][3];}
    float geti() const {return ((long)data[1][4] << 8) + data[1][5];}
    float getMagX() const {return ((long)data[2][0] << 8) + data[2][1];}  
    float getMagY() const {return ((long)data[2][2] << 8) + data[2][3];}
    float getMagZ() const {return ((long)data[2][4] << 8) + data[2][5];}
    
    unsigned long getAge() const {return(millis() - receiveTime);} //time since last data packet

    

};


/*
_______ ______   ______ 
(_______|_____ \ / _____)
 _   ___ _____) | (____  
| | (_  |  ____/ \____ \ 
| |___) | |      _____) )
 \_____/|_|     (______/ 
                         
*/
struct GPS {
    byte data[4][8] = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  //Latitude
                {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //Longitude
                {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //Other
                {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}; //Other 2
    unsigned long ID = 0;
    FlexCAN_T4<CAN_DATA_BUS, RX_SIZE_256, TX_SIZE_16> Can2;
    CAN_message_t msg;
    unsigned long receiveTime = 0;

    GPS(unsigned long id, FlexCAN_T4<CAN_DATA_BUS, RX_SIZE_256, TX_SIZE_16> &can) : ID(id){
        can = Can2;
    }

    void receive(unsigned long id, byte buf[]){
        if(id >= 0x10F23 && id <= 0x10F6){
            byte digit2 = (id - 0x10F23); 
            receiveTime = millis();
            for(int i = 0; i < 8; i++) data[digit2][i] = buf[i];                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
            return;
        }
        else{
            Serial.print(id, HEX);
            Serial.println(" is not data from GPS");
        }
    }

    float getLatitude() const {return ((long)data[0][0] << 24) + ((long)data[0][1] << 16) + ((long)data[0][2] << 8) + data[0][3];}
    float getHighPrecisionLatitude() const {return ((long)data[0][4] << 24) + ((long)data[0][5] << 16) + ((long)data[0][6] << 8) + data[0][7];}
    float getLongitude() const {return ((long)data[1][0] << 24) + ((long)data[1][1] << 16) + ((long)data[1][2] << 8) + data[1][3];}
    float getHighPrecisionLongitude() const {return ((long)data[1][4] << 24) + ((long)data[1][5] << 16) + ((long)data[1][6] << 8) + data[1][7];}
    unsigned long getAge() const {return(millis() - receiveTime);} //time since last data packet

    //rest of the data is still undecided.

};

/*
 ______ _______ ______  _______ _        ______ 
(_____ (_______|______)(_______|_)      / _____)
 _____) )____   _     _ _______ _      ( (____  
|  ____/  ___) | |   | |  ___  | |      \____ \ 
| |    | |_____| |__/ /| |   | | |_____ _____) )
|_|    |_______)_____/ |_|   |_|_______|______/ 
*/
struct Pedals{
    byte data[2][8] = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  
                {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}; 
    byte dataOut[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned long ID = 0;
    FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> Can1;
    CAN_message_t msg;
    unsigned long receiveTime = 0;

    Pedals(unsigned long id, FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> &can) : ID(id){
        can = Can1;
    }

    void receive(unsigned long id, byte buf[]){
        if(id == 0xC8 || id == 0xC9){
            receiveTime = millis();
            for(int i = 0; i < 8; i++) data[0][i] = buf[i];
            return;
        }
        else{
            Serial.print(id, HEX);
            Serial.println(" is not data from Pedals");
        }
    }

    unsigned long getID() {return ID;}

    float getThrottle() const {
        return 0; // TODO: Do
    }
    float getAPPS1() const {return ((long)data[0][0] << 8) + data[0][1];}
    float getAPPS2() const {return ((long)data[0][2] << 8) + data[0][3];}
    float getBrakePressureF() const {return ((long)data[0][4] << 8) + data[0][5];}
    float getBrakePressureR() const {return ((long)data[0][6] << 8) + data[0][7];}
    byte getPedalsPingResponse() const {return data[1][0];}
    void pedalPingRequest(byte anything) {dataOut[0] = anything; send();} //ping request
    
    void send(){
        for(int i = 0; i < 8; i++) msg.buf[i] = dataOut[i];
        msg.id = ID;
        msg.flags.extended=true;
        Can1.write(msg);
    }
    void reset(){
        for(int i = 0; i < 8; i++) dataOut[i] = 0x00;
        send();
    }

    unsigned long getAge() const {return(millis() - receiveTime);} //time since last data packet

};


/*
 _______ _______ _     _ 
(_______|_______|_)   (_)
 _______ _       _     _ 
|  ___  | |     | |   | |
| |   | | |_____| |___| |
|_|   |_|\______)\_____/ 

*/


struct ACU {
    //condensed cell data and bunch of other stuff
    byte data[40][8] = {0x00}; //40 ids
    byte dataOut[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint32_t ID = 0;
    FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> Can1;
    CAN_message_t msg;
    uint32_t receiveTime = 0;
    int range_cell_data[2] = {0x99, 0xBC};
    
    ACU(unsigned long id, FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> &can): ID(id){
        can = Can1;
    }

    void receive(unsigned long id, byte buf[]){
        if(id >= 0x96 && id <= 0xC4){
            uint32_t row = id - 0x96;
            receiveTime = millis();
            for(int i = 0; i < 8; i++) data[row][i] = buf[i];
        }
        else if(id == 0xC7) {
            receiveTime = millis();
            for(int i = 0; i < 8; i++) data[39][i] = buf[i];
            return;
        }
        else{
            Serial.print(id, HEX);
            Serial.println(" is not data from ACU");
        }
    }

    unsigned long getID() {return ID;}


    //voltages and temps for specific cells (range 0 to 144)
    float getCellVoltage_n(int cell_n) const {
        if(cell_n < 0 || cell_n > 144) Serial.println("Battery Cell number out of range [0,144]");
        int col = (cell_n%4)*2; //even bytes
        int row = (cell_n/4) + 3; //check GR24 CAN datasheet
        return data[row][col];
    }
    float getCellTemp_n(int cell_n) const {
        if(cell_n < 0 || cell_n > 144) Serial.println("Battery Cell number out of range [0,144]");
        int col = (cell_n%4)*2 + 1; //odd bytes
        int row = (cell_n/4) + 3; //check GR24 CAN datasheet
        return data[row][col];
    }

    //ACU General
    float getAccumulatorVoltage() const {return ((long)data[0][0] << 8) + data[0][1];}
    float getAccumulatorCurrent() const {return ((long)data[0][2] << 8) + data[0][3];}
    float getMaxCellTemp() const {return ((long)data[0][4] << 8) + data[0][5];}
    byte getSOC() const {return data[0][6];}//state of charge
    byte getACUGeneralErrors() const {return data[0][7];}

    byte getFan1Speed() const {return data[1][0];}
    byte getFan2Speed() const {return data[1][1];}
    byte getFan3Speed() const {return data[1][2];}
    byte getFan4Speed() const {return data[1][3];}
    byte getPumpSpeed() const {return data[1][4];}
    float getWaterTemp() const {return ((long)data[1][5] << 8) + data[1][6];}
    byte getPowertrainCoolingErrors() const {return data[1][7];}
    byte getACUPingResponse() const {return data[39][0];}
    unsigned long getAge() const {return(millis() - receiveTime);} //time since last data packet


    void setFan1Speed(byte in) {dataOut[0] = in; send();}
    void setFan2Speed(byte in) {dataOut[1] = in; send();}
    void setFan3Speed(byte in) {dataOut[2] = in; send();}
    void setFan4Speed(byte in) {dataOut[3] = in; send();}
    void setPumpSpeed(byte in) {dataOut[4] = in; send();}

    void playSound(byte soundCode) {dataOut[0] = soundCode; send();} //play sound
    void pingRequest(byte anything) {dataOut[0] = anything; send();} //ping request
    // void requestCellData(cell, data type, periodic?) //  IDK what this supposed to do
    

    void send(){
        for(int i = 0; i < 8; i++) msg.buf[i] = dataOut[i];
        msg.id = ID;
        msg.flags.extended=true;
        Can1.write(msg);
    }

    void reset(){
        for(int i = 0; i < 8; i++) dataOut[i] = 0x00;
        send();
    }


};


/*
 ______  _______ _______ 
(____  \(_______|_______)
 ____)  )_       _  _  _ 
|  __  (| |     | ||_|| |
| |__)  ) |_____| |   | |
|______/ \______)_|   |_|

*/
struct BCM {
    byte data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte dataOut[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    unsigned long ID = 0;
    FlexCAN_T4<CAN_DATA_BUS, RX_SIZE_256, TX_SIZE_16> Can2;
    CAN_message_t msg;
    unsigned long receiveTime = 0;
    BCM(unsigned long id, FlexCAN_T4<CAN_DATA_BUS, RX_SIZE_256, TX_SIZE_16> &can) : ID(id){
        can = Can2;
    }

    void receive(unsigned long id, byte buf[]){
        if(id == 0x12000){
            receiveTime = millis();
            for(int i = 0; i < 8; i++) data[i] = buf[i];
            return;
        }
        else{
            Serial.print(id, HEX);
            Serial.println(" is not data from BCM");
        }
    }

    byte getCloudStatus() const {return data[0];}
    unsigned long getAge() const {return(millis() - receiveTime);} //time since last data packet

};

/*
 ______  _______  ______ _     _ 
(______)(_______)/ _____|_)   (_)
 _     _ _______( (____  _______ 
| |   | |  ___  |\____ \|  ___  |
| |__/ /| |   | |_____) ) |   | |
|_____/ |_|   |_(______/|_|   |_|
                                 
*/
struct Dash {
    byte data[3][8] = {0x00};
    byte dataOut[8] = {0x00};
    unsigned long ID = 0;
    FlexCAN_T4<CAN_DATA_BUS, RX_SIZE_256, TX_SIZE_16> Can2;
    CAN_message_t msg;
    unsigned long receiveTime = 0;

    Dash(unsigned long id, FlexCAN_T4<CAN_DATA_BUS, RX_SIZE_256, TX_SIZE_16> &can) : ID(id){
        can = Can2;
    }

    void receive(unsigned long id, byte buf[]){
        if(id >= 0x11001 && id <= 0x11002){
            byte digit2 = (id - 0x11001); // 0 for 0x11001, 1 for 0x11002
            receiveTime = millis();
            for(int i = 0; i < 8; i++) data[digit2][i] = buf[i];                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
            return;
        }
        else{
            Serial.print(id, HEX);
            Serial.println(" is not data from Dash");
        }
    }

    float getAccelX() const {return ((long)data[0][0] << 8) + data[0][1];}
    float getAccelY() const {return ((long)data[0][2] << 8) + data[0][3];}
    float getAccelZ() const {return ((long)data[0][4] << 8) + data[0][5];}
    float getGyroX() const {return ((long)data[1][0] << 8) + data[1][1];}
    float getGyroY() const {return ((long)data[1][2] << 8) + data[1][3];}
    float getGyroZ() const {return ((long)data[1][4] << 8) + data[1][5];}

    unsigned long getAge() const {return(millis() - receiveTime);} //time since last data packet

    void send(){
        for(int i = 0; i < 8; i++) msg.buf[i] = dataOut[i];
        msg.id = ID;
        msg.flags.extended=true;
        Can2.write(msg);
    }
    void reset(){
        for(int i = 0; i < 8; i++) dataOut[i] = 0x00;
        send();
    }




};

/*
 _______     _______ _______ _______ _______ ______  
(_______)   (_______|_______|_______|_______|_____ \ 
 _____ _____ _  _  _ _____      _    _____   _____) )
|  ___|_____) ||_|| |  ___)    | |  |  ___) |  __  / 
| |_____    | |   | | |_____   | |  | |_____| |  \ \ 
|_______)   |_|   |_|_______)  |_|  |_______)_|   |_|
                                                     
*/

struct Energy_Meter {
    byte data[8] = {0x00};
    unsigned long ID = 0;
    FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> Can1;
    CAN_message_t msg;
    unsigned long receiveTime = 0;

    Energy_Meter(unsigned long id, FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> &can) : ID(id){
        can = Can1;
    }

    void receive(unsigned long id, byte buf[]){
        if(id == 100){
            receiveTime = millis();
            for(int i = 0; i < 8; i++) data[i] = buf[i];
            return;
        }
        else{
            Serial.print(id, HEX);
            Serial.println(" is not data from Energy Meter");
        }
    }

    float getCurrent() const {return ((long)data[0] << 24) + ((long)data[1] << 16) + ((long)data[2] << 8) + data[3];}
    float getVoltage() const {return ((long)data[4] << 24) + ((long)data[5] << 16) + ((long)data[6] << 8) + data[7];}
    unsigned long getAge() const {return(millis() - receiveTime);} //time since last data packet
    
};




// DASH PANEL



#endif
