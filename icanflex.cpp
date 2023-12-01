#include "FlexCAN_T4.h"




bool I_no_can_speak_flex::begin(){   //Coordinate the magic CAN pixies to dance together 
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
    can1.begin();
    can1.setBaudRate(1000000);
    msg.flags.extended = 1;
    can2.begin();
    can2.setBaudRate(1000000);
    return true;
}
