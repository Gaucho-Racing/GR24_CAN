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




struct VDM {

};

struct Sensors {

};

struct Energy_Meter {

};

struct Inverter {

};

struct BCM {

};

struct ACU {

};

struct Dash {

};





