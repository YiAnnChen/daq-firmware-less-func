#ifndef can_addr_def_h
#define can_addr_def_h

typedef enum {
  CA_INV_LEFT_VOLTAGE = 0x10,
  CA_INV_RIGHT_VOLTAGE = 0x11,
  CA_INV_LEFT_DATA1 = 0x12,
  CA_INV_LEFT_DATA2 = 0x13,
  CA_INV_LEFT_DATA3 = 0x14,
  CA_INV_RIGHT_DATA1 = 0x15,
  CA_INV_RIGHT_DATA2 = 0x16,
  CA_INV_RIGHT_DATA3 = 0x17,

  CA_IMD_Request = 0x22,
  CA_IMD_Response = 0x23,
  CA_FAULT_SIGNAL = 0x30,
  CA_IMD_Info_General = 0x37,
  CA_IMD_Info_IsolationDetail = 0x38,
  CA_IMD_Info_Voltage = 0x39,
  CA_IMD_Info_IT = 0x3A,

  CA_BMS_DATA1 = 0x40,
  CA_BMS_DATA2 = 0x41,
  CA_LVBMS_DATA1 = 0x45,
  CA_LVBMS_DATA2 = 0x46,
  
  CA_DASHBOARD_BTN = 0x50,
  CA_DAQ_EN = 0x70,
  CA_DAQ_DATA = 0x71,
  CA_RGB_EN = 0x90,
} can_addr_def_e;

/* CAN data index for Fault Signal */
#define CA_BSPD_FAULT_IDX 0
#define CA_IMD_FAULT_IDX 1
#define CA_BMS_FAULT_IDX 2
/* CAN data index for BSPD Signal */
#define CA_BOTH_FAULT 6
#define CA_APPS_FAULT 5
#define CA_BSER_FAULT 4
#define CA_BSEL_FAULT 3


/* CAN data index for Dashboard Button */
#define CA_U1_TRIG 0
#define CA_U2_TRIG 1
#define CA_U3_TRIG 2

/* CAN data index for Dashboard Info */
#define CA_U1_LED 0
#define CA_U2_LED 1
#define CA_U3_LED 2


/* CAN data index for DAQ Enable */
#define CA_DAQ_EN_IDX 7

/* CAN data index for DAQ Data */
#define CA_DAQ_PRE_TEMP 7
#define CA_DAQ_POST_TEMP 6
#define CA_DAQ_PRESSURE_H 5
#define CA_DAQ_PRESSURE_L 4
#define CA_DAQ_FLOW_RATE_H 3
#define CA_DAQ_FLOW_RATE_L 2
#define CA_DAQ_FAN_PWM 1
#define CA_DAQ_PUMP_PWM 0

/* CAN data index for RGB Enable */
#define CA_RGB_EN_IDX 7
#define CA_RGB_INTENSITY 6

#endif
