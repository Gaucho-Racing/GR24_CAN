#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SPI.h>

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





struct Inverter {
    byte data[5][8]={{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}; 
    
    /*
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

    long getERPM() {return(((long)data[0][0] << 24) + ((long)data[0][1] << 16) + ((long)data[0][2] << 8) + data[0][3]);} //rpm/pole pairs
    float getDuty() {return((((long)data[0][4] << 8) + data[0][5])/10);} //i think [0,100]. Related to top speed
    int getVoltIn() {return(((long)data[0][6] << 8) + data[0][7]);}
    float getACCurrent() {return((((long)data[1][0] << 8) + data[1][1])/10);}
    float getDCCurrent() {return(((long)(data[1][2] << 8) + data[1][3])/10);}
    float getInvTemp() {return((((long)data[2][0] << 8) + data[2][1])/10);} //Deg C
    float getMotorTemp() {return((((long)data[2][2] << 8) + data[2][3])/10);} //Deg C
    byte getFaults() {return data[2][4];}
    float getCurrentD() {return((((long)data[3][0] << 24) + ((long)data[3][1] << 16) + ((long)data[3][2] << 8) + data[3][3])/100);}  //FOC current (don't need)
    float getCurrentQ() {return((((long)data[3][4] << 24) + ((long)data[3][5] << 16) + ((long)data[3][6] << 8) + data[3][7])/100);}  //FOC current (don't need)
    byte getThrottleIn() {return data[4][0];}  //Received throttle signal by the invertor
    byte getBrakeIn() {return data[4][1];}  //Received brake signal by the invertor
    bool getD1() {return ((data[4][2] & 0x80) == 0x80);}  //Digital input read
    bool getD2() {return ((data[4][2] & 0x40) == 0x40);}  //Digital input read
    bool getD3() {return ((data[4][2] & 0x20) == 0x20);}  //Digital input read
    bool getD4() {return ((data[4][2] & 0x10) == 0x10);}  //Digital input read
    bool getDO1() {return ((data[4][2] & 0x08) == 0x08);}  //Digital output write
    bool getDO2() {return ((data[4][2] & 0x04) == 0x04);}  //Digital output write
    bool getDO3() {return ((data[4][2] & 0x02) == 0x02);}  //Digital output write
    bool getDO4() {return ((data[4][2] & 0x01) == 0x01);}  //Digital output write
    bool getDriveEnable() {return ((data[4][3] & 0x01) == 0x01);} //These are setting that can be changed (prob don't need these)
    bool getCapTempLim() {return ((data[4][4] & 0x80) == 0x80);}//         ^
    bool getDCCurrentLim() {return ((data[4][4] & 0x40) == 0x40);}//       ^
    bool getDriveEnableLim() {return ((data[4][4] & 0x20) == 0x20);}//     ^
    bool getIgbtAccelTempLim() {return ((data[4][4] & 0x10) == 0x10);}//   ^
    bool getIgbtTempLim() {return ((data[4][4] & 0x08) == 0x08);}//        ^
    bool getVoltInLim() {return ((data[4][4] & 0x04) == 0x04);}//          ^
    bool getMotorAccelTempLim() {return ((data[4][4] & 0x02) == 0x02);}//  ^
    bool getMotorTempLim() {return ((data[4][4] & 0x01) == 0x01);}//       ^
    bool getRPMMinLimit() {return ((data[4][5] & 0x80) == 0x80);}//        ^
    bool getRPMMaxLimit() {return ((data[4][5] & 0x40) == 0x40);}//        ^
    bool getPowerLimit() {return ((data[4][5] & 0x20) == 0x20);}//    



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
    unsigned long getAge(){return(millis() - receiveTime);} //time since last data packet
};

struct VDM {

};


struct BCM {

};

struct ACU {

};

struct Dash {

};


struct Sensors {
    // REFERENCE GR24 CAN BUS DATA SHEET
   
    unsigned long ID = 0;
    FlexCAN_T4<CAN_DATA_BUS, RX_SIZE_256, TX_SIZE_16> Can1;
    CAN_message_t msg;
    unsigned long receiveTime = 0;

    Sensors(unsigned long id, FlexCAN_T4<CAN_DATA_BUS, RX_SIZE_256, TX_SIZE_16> &can) : ID(id){
        can = Can1;
    }

    void receive(unsigned long id, byte buf[]){
        // idfk 
        // ... polymorphism using c++ or some shit
        // or just not
    }

     struct Wheel_FR {
         byte data[5][8] = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}; 
    };
    struct Wheel_FL {
        byte data[5][8] = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}; 
    };
    
    struct Wheel_RR {
        byte data[5][8] = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}; 
    };

    struct Wheel_RL {
        byte data[5][8] = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}; 
    };
    
    struct Central_IMU {
        byte data[3][8] = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  //Accel
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //Gyro
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}; //Mag

    };

    struct GPS {
        byte data[4][8] = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  //Latitude
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //Longitude
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //Other
                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}; //Other 2
    };

};

struct Energy_Meter {

};





