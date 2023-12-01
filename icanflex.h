#include "Nodes.h"
#include "FlexCAN_T4.h"



#define USE_CAN_PRIMARY

#define INT8U byte
#define INT32U unsigned long

#ifndef I_no_can_speak_flex_
#define I_no_can_speak_flex_

#define Sender true
#define Reciever false

#if defined(USE_CAN_PRIMARY) && !defined(USE_CAN_DATA)
    #define CAN_PRIMARY_BUS CAN1
    #define CAN_DATA_BUS CAN2  // Default to CAN2 if CAN_PRIMARY_BUS is selected
#elif !defined(USE_CAN_PRIMARY) && defined(USE_CAN_DATA)
    #define CAN_PRIMARY_BUS CAN2
    #define CAN_DATA_BUS CAN2
#else
    #error "Please define either USE_CAN_PRIMARY or USE_CAN_DATA"
#endif



class iCANflex{
    public:

    iCANflex();
    FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> can_primary;
    FlexCAN_T4<CAN_DATA_BUS, RX_SIZE_256, TX_SIZE_16> can_data;


    Inverter DTI = Inverter(22, can_primary);
    VDM ECU = VDM(23, can_primary);
    Wheel WFL = Wheel(24, can_data, WHEEL_FL);
    Wheel WFR = Wheel(25, can_data, WHEEL_FR);
    Wheel WRL = Wheel(26, can_data, WHEEL_RL);
    Wheel WRR = Wheel(27, can_data, WHEEL_RR);
    GPS GPS1 = GPS(28, can_data);
    Pedals PEDALS = Pedals(29, can_primary);
    ACU ACU1 = ACU(30, can_primary);
    BCM BCM1 = BCM(31, can_data);
    Dash DASHBOARD = Dash(32, can_data);
    Energy_Meter ENERGY_METER = Energy_Meter(33, can_primary);














};







#endif 
