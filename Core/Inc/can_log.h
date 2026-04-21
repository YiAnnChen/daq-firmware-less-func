#ifndef CAN_LOG_H
#define CAN_LOG_H

#include <stdint.h>
#include <string.h>

#include "SEGGER_RTT.h"
#include "can_addr_def.h"

#define CL_COUNT 23

static struct {
  uint32_t id;
  const char *label;
  uint8_t rx;
  uint8_t data[8];
} _cl[] = {
    {CA_INV_LEFT_VOLTAGE,         "INV_LV "},
    {CA_INV_RIGHT_VOLTAGE,        "INV_RV "},
    {CA_INV_LEFT_DATA1,           "INV_L1 "},
    {CA_INV_LEFT_DATA2,           "INV_L2 "},
    {CA_INV_LEFT_DATA3,           "INV_L3 "},
    {CA_INV_RIGHT_DATA1,          "INV_R1 "},
    {CA_INV_RIGHT_DATA2,          "INV_R2 "},
    {CA_INV_RIGHT_DATA3,          "INV_R3 "},
    {CA_IMD_Request,              "IMD_REQ"},
    {CA_IMD_Response,             "IMD_RSP"},
    {CA_FAULT_SIGNAL,             "FAULT  "},
    {CA_IMD_Info_General,         "IMD_GEN"},
    {CA_IMD_Info_IsolationDetail, "IMD_ISO"},
    {CA_IMD_Info_Voltage,         "IMD_VLT"},
    {CA_IMD_Info_IT,              "IMD_IT "},
    {CA_BMS_DATA1,                "BMS1   "},
    {CA_BMS_DATA2,                "BMS2   "},
    {CA_LVBMS_DATA1,              "LVBMS1 "},
    {CA_LVBMS_DATA2,              "LVBMS2 "},
    {CA_DASHBOARD_BTN,            "BTN    "},
    {CA_DAQ_EN,                   "DAQ_EN "},
    {CA_DAQ_DATA,                 "DAQ    "},
    {CA_RGB_EN,                   "RGB    "},
};

static void _cl_print(uint32_t std_id, const uint8_t *data) {
  switch (std_id) {
    case CA_FAULT_SIGNAL:
      SEGGER_RTT_printf(0, "[CAN 0x%02x] FAULT | BSPD:%s IMD:%s BMS:%s\r\n",
                         std_id,
                         (data[CA_BSPD_FAULT_IDX] == 0xFF) ? "ERR" : "OK",
                         (data[CA_IMD_FAULT_IDX] == 0xFF) ? "ERR" : "OK",
                         (data[CA_BMS_FAULT_IDX] == 0xFF) ? "ERR" : "OK");
      break;
    case CA_DASHBOARD_BTN:
      SEGGER_RTT_printf(0, "[CAN 0x%02x] BTN   | RTD:%s RGB:%s REC:%s\r\n",
                         std_id,
                         (data[CA_U1_TRIG] == 0xFF) ? "ON " : "OFF",
                         (data[CA_U2_TRIG] == 0xFF) ? "ON " : "OFF",
                         (data[CA_U3_TRIG] == 0xFF) ? "ON " : "OFF");
      break;
    case CA_DAQ_DATA: {
      uint16_t pressure =
          ((uint16_t)data[CA_DAQ_PRESSURE_H] << 8) | data[CA_DAQ_PRESSURE_L];
      uint16_t flow_rate =
          ((uint16_t)data[CA_DAQ_FLOW_RATE_H] << 8) | data[CA_DAQ_FLOW_RATE_L];
      SEGGER_RTT_printf(
          0,
          "[CAN 0x%02x] DAQ   | PreT:%d PostT:%d Pres:%d Flow:%d Fan:%d "
          "Pump:%d\r\n",
          std_id, data[CA_DAQ_PRE_TEMP], data[CA_DAQ_POST_TEMP], pressure,
          flow_rate, data[CA_DAQ_FAN_PWM], data[CA_DAQ_PUMP_PWM]);
      break;
    }
    case CA_DAQ_EN:
      SEGGER_RTT_printf(0, "[CAN 0x%02x] DAQ_EN| EN:%s\r\n", std_id,
                         (data[CA_DAQ_EN_IDX] == 0xFF) ? "ON " : "OFF");
      break;
    case CA_RGB_EN:
      SEGGER_RTT_printf(0,
                         "[CAN 0x%02x] RGB   | EN:%s Intensity:%d\r\n", std_id,
                         (data[CA_RGB_EN_IDX] == 0xFF) ? "ON " : "OFF",
                         data[CA_RGB_INTENSITY]);
      break;
    default:
      SEGGER_RTT_printf(0, "[CAN 0x%02x] %02x %02x %02x %02x %02x %02x %02x %02x\r\n",
                         std_id, data[0], data[1], data[2], data[3],
                         data[4], data[5], data[6], data[7]);
      break;
  }
}

/**
 * @brief Store latest CAN data into state table.
 *        Call this inside HAL_CAN_RxFifo0MsgPendingCallback.
 */
static void CAN_Log_Update(uint32_t std_id, const uint8_t *data) {
  for (int i = 0; i < CL_COUNT; i++) {
    if (_cl[i].id == std_id) {
      memcpy(_cl[i].data, data, 8);
      _cl[i].rx = 1;
      return;
    }
  }
}

/**
 * @brief Print all CAN signals at once, then reset all states.
 *        Received signals print formatted data, others print "N/A".
 */
static void CAN_Log_PrintAll(void) {
  SEGGER_RTT_printf(0, "===== CAN Signal Dump =====\r\n");
  for (int i = 0; i < CL_COUNT; i++) {
    if (_cl[i].rx) {
      _cl_print(_cl[i].id, _cl[i].data);
      _cl[i].rx = 0;
    } else {
      SEGGER_RTT_printf(0, "[CAN 0x%02x] %s | N/A\r\n",
                         _cl[i].id, _cl[i].label);
    }
  }
  SEGGER_RTT_printf(0, "===========================\r\n");
}

#endif  // CAN_LOG_H
