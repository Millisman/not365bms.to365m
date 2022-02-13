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
#include <avr/pgmspace.h>
#include "mcu/pin.h"
#include "bms365.hpp"

namespace protocol {


    
enum PrintParam {
    Conf_Allow_Charging,
    Conf_Allow_Discharging,
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

// virtual cells count
#define OUTPUT_COUNT 10

class Console {
    devices::bq769_conf  bq769x_conf;
    devices::bq769_data  bq769x_data;
    devices::bq769_stats bq769x_stats;
    devices::bq769x0     bq;
    
    Protocol_365 proto;
    
    uint32_t m_lastUpdate;
    uint32_t m_oldMillis = 0;
    uint32_t m_millisOverflows;
    uint16_t m_BatCycles_prev;
    uint16_t m_ChargedTimes_prev;
    uint8_t shd;
public:
    Console();
    bool update(const bool force);
    void begin();
//     bool Recv();
    void command_format_EEMEM();
    void command_bootloader();
    void stats_load();
    void stats_save();
    void debug_print();
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
