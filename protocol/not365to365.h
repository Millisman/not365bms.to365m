/* Shell console for battery management based on bq769x Ic
 * Copyright (c) 2022 Sergey Kostanoy (https://arduino.uno)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <stdint.h>
#include "utils/cpp.h"
#include <string.h>
#include <avr/eeprom.h>
#include "devices/bq769x0.h"
#include "mcu/timer.h"
#include <avr/pgmspace.h>
#include "mcu/pin.h"

namespace protocol {

#define PACKET_SIZE 128

enum M36ProtoDirect :uint8_t {
    to_Batt     = 0x22,
    to_Master   = 0x25
};

enum M36MagicChar :uint8_t {
    magic_First    = 0x55,
    magic_Second   = 0xAA
};

enum M36StatusBits {
    b_Ready         = 0,
    b_Charging      = 6,
    b_UnderVoltage  = 8,
    b_OverVoltage   = 9,
    b_OverHeat      = 10,
};

enum M36Regs :uint8_t {
    Reg_Serial      = 0x10, // BMS s/n
    Reg_Version     = 0x17, // BMS Ver
    Reg_Capacity    = 0x18, // BMS Design capacity
    Reg_Date        = 0x20, // BMS Factory date Year 20** (MSB) 7b| Month 4b| Day (LSB) 5b
    Reg_Status      = 0x30, // BMS Flags
    Reg_CapLeft     = 0x31, // How much capacity is left
    Reg_Percentage  = 0x32, // Charge percentage
    Reg_Current     = 0x33, // Current (gained x100, signed), '-' sign Load, '+' sign for charging
    Reg_Voltage     = 0x34, // Voltage, full pack (gained x10)
    Reg_Temp        = 0x35, // Temp sensors (Temp1 (MSB), Temp2 (LSB) with offset +20)
    Reg_PackHealth  = 0x3B, // Battery health cannot be lower than 60%
    Reg_Cell_0      = 0x40, // Cell 1 voltage, mV
    Reg_Cell_1      = 0x41, // Cell 2 voltage, mV
    Reg_Cell_2      = 0x42, // Cell 3 voltage, mV
    Reg_Cell_3      = 0x43, // Cell 4 voltage, mV
    Reg_Cell_4      = 0x44, // Cell 5 voltage, mV
    Reg_Cell_5      = 0x45, // Cell 6 voltage, mV
    Reg_Cell_6      = 0x46, // Cell 7 voltage, mV
    Reg_Cell_7      = 0x47, // Cell 8 voltage, mV
    Reg_Cell_8      = 0x48, // Cell 9 voltage, mV
    Reg_Cell_9      = 0x49  // Cell 10 voltage, mV
};

enum M36Cells :uint8_t {
    Cell_0,
    Cell_1,
    Cell_2,
    Cell_3,
    Cell_4,
    Cell_5,
    Cell_6,
    Cell_7,
    Cell_8,
    Cell_9
};

// #define MAX_NUMBER_OF_CELLSINPUT_COUNT MAX_NUMBER_OF_CELLS
#define OUTPUT_COUNT 10

class Protocol {
    bool     m_Error;
    uint16_t m_360_p[PACKET_SIZE];
    uint16_t m_crc16;
    uint16_t m_Undervolt_lvl;
    uint16_t m_Overvolt_lvl;    
    uint16_t m_Min_Cell_Voltage;
    uint16_t m_Max_Cell_Voltage;
public:
    void set_Cell_Voltage(M36Cells cell, uint16_t voltage);
    void two8to16(uint8_t rega, uint8_t a, uint8_t b);
    Protocol(uint16_t _ver = 0x116, uint16_t _cap = 0x19C8, uint16_t _date = 0x2A2A);
    void update();
    uint16_t send_byte(uint8_t c);
    void set_status_bit(M36StatusBits bit, bool state);
    void set_pack_voltage(uint16_t pV);
protected:
    uint16_t looking;
    uint16_t send_360_p(uint8_t pos, uint16_t sz);
    void protocol_send(const uint8_t cmd);
};

    
enum PrintParam {
    Conf_Allow_Charging,
    Conf_Allow_Discharging,
    //Conf_BQ_dbg,
    Conf_RT_bits,
    Conf_RS_uOhm,
    Conf_RT_Beta,
    Conf_Cell_CapaNom_mV,
    Conf_Cell_CapaFull_mV,
    Conf_Batt_CapaNom_mAsec,
    Conf_CurrentThresholdIdle_mA,
    Conf_Cell_TempCharge_min,
    Conf_Cell_TempCharge_max,
    Conf_Cell_TempDischarge_min,
    Conf_Cell_TempDischarge_max,
    Conf_BalancingInCharge,
    Conf_BalancingEnable,
    Conf_BalancingCellMin_mV,
    Conf_BalancingCellMaxDifference_mV,
    Conf_BalancingIdleTimeMin_s,
    Conf_Cell_OCD_mA,
    Conf_Cell_OCD_ms,
    Conf_Cell_SCD_mA,
    Conf_Cell_SCD_us,
    Conf_Cell_ODP_mA,
    Conf_Cell_ODP_ms,
    Conf_Cell_OVP_mV,
    Conf_Cell_OVP_sec,
    Conf_Cell_UVP_mV,
    Conf_Cell_UVP_sec,
    Conf_adcCellsOffset,
    Conf_ts,
    Conf_CRC8,
    FIRST = Conf_Allow_Charging,
    LAST = Conf_CRC8
};

class Console {
    devices::bq769_conf  bq769x_conf;
    devices::bq769_data  bq769x_data;
    devices::bq769_stats bq769x_stats;
    devices::bq769x0     bq;
    Protocol             proto;
    
    uint32_t m_lastUpdate;
    uint32_t m_oldMillis = 0;
    uint32_t m_millisOverflows;
    uint16_t m_BatCycles_prev;
    uint16_t m_ChargedTimes_prev;
    uint8_t shd;
public:
    Console();
    bool update(mcu::Pin job, const bool force);
    void begin();
    bool Recv();
    void command_format_EEMEM();
    void command_bootloader();
    void stats_load();
    void stats_save();

private:
    void conf_begin_protect();
    void conf_default();
    uint16_t arr_connected[MAX_NUMBER_OF_CELLS];
    uint16_t arr_out[OUTPUT_COUNT];
    uint8_t cell_count = 0;
    void sorting();
};

uint8_t gencrc8(uint8_t *data, uint16_t len);

}
