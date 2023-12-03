#include "FlexCAN_T4.h"
#include "Nodes.h"
#include "icanflex.h"

unsigned long lastRecieveTime=millis();

unsigned long stopwatch=0;
bool canSend = false;


iCANflex::iCANflex(){


}

bool iCANflex::begin() {   //Coordinate the magic CAN pixies to dance together 
    Serial.begin(115200);
    // hasSD=SD.begin(BUILTIN_SDCARD);
    //  if(hasSD){
    //     String filename = "test0.csv";
    //     int filenum = 0;
    //     char a[500];
    //     filename.toCharArray(a,500);
    //     while(SD.exists(a)){
    //         filenum++;
    //         filename = "test"+(String)filenum+".csv";
    //         filename.toCharArray(a,500);
    //     }
    //     logFile = SD.open(a, O_CREAT | O_WRITE);
    //     stuffz.hasSD2 = hasSD;
    //     stuffz.logFile = logFile;
    // }
    can_primary.begin();
    can_primary.setBaudRate(1000000);
    msg.flags.extended = 1;
    can_data.begin();
    can_data.setBaudRate(1000000);
    return true;
}
bool iCANflex::readData(){    //Read data from the inverter or the BMS
    // lastRecieveTime = millis();
    // if(!can_primary.read(msg)){ // fix this line I beg you
    // //Serial.println("no");
    //     return 0;
    // }
    // if(!msg.flags.extended){
    //     Serial.println("WHAT IS THIS SHIT???? I WANT AN EXTENDED FRAME!!!! COME ON DUDE");
    //     return false;
    // }
    // // stuffz.CAN_CSV(msg);
    // if((msg.id & 0xFF) == DTI.ID){
    //     DTI.receive(msg.id, msg.buf);
    // }
    // else if(msg.id == BMS.ID){
    //     BMS.receive(msg.id, msg.buf);
    //     Serial.print("BMS "); Serial.println(msg.id);
    // }
    // else if(msg.id == fans.ID){
    //     fans.receive(msg.id, msg.buf);
    // }
    // else if(msg.id == pedals.ID){
    //     pedals.receive(msg.id, msg.buf);
    // }
    // else if(msg.id == 105){
    //     Serial.println(micros()-stopwatch);
    // }
    // else{
    //     Serial.print("Unknown CAN Id "); Serial.println(msg.id);
    // }
    // return true;
}

bool iCANflex::readData(INT32U *tim){//Read function that gives the time since the last read
    *tim = millis() - lastRecieveTime;
    return readData();
}
void iCANflex::rawData(INT32U *id, INT8U *buffer){*id=msg.id; buffer=msg.buf;}  //Gives back the raw Id and data array





void iCANflex::send(long OutId, long data, int dataLength){    //Sends 8 bytes with that Id and that data shifted left all the way
    byte stuff[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    for(int i=0; i<dataLength; i++)
        stuff[i] = (data >> (i*8));
    for (int i = 0; i < 8; i++)
        msg.buf[i] = stuff[i];
    msg.id = OutId;
    if(canSend){can_primary.write(msg);} //por favor mi amor
    else{Serial.println("YOU TRIED TO SEND????? - note this will prob appear when recieving from the bms actually nevermind ill fix that now");}
}


void iCANflex::send(long OutId, byte stuff[]){    //Sends 8 bytes with that Id and that data shifted left all the way
    for (int i = 0; i < 8; i++)
        msg.buf[i] = stuff[i];
    msg.id = OutId;
    if(canSend){can_primary.write(msg);} //por favor mi amor
    else{Serial.println("YOU TRIED TO SEND????? - note this will prob appear when recieving from the bms actually nevermind ill fix that now");}
}

void iCANflex::sendDashError(byte error){
    msg.id = 2021;
    msg.buf[0] = error;
    msg.buf[1]=0;msg.buf[2]=0;msg.buf[3]=0;msg.buf[4]=0;msg.buf[5]=0;msg.buf[6]=0;msg.buf[7]=0;
    can_primary.write(msg);
}
// void I_no_can_speak_flex::ping(byte id){
//     msg.id = id;
//     stopwatch=micros();
//     can1.write(msg);
// }
void iCANflex::ping(byte id){
    msg.id = id;
    stopwatch=micros();
    can_data.write(msg);
}