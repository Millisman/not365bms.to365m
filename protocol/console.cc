/*
 * Emulation protocol for battery management based on bq769x Ic
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

#include "console.h"
#include <stdlib.h>
#include "mcu/watchdog.h"
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include "GyverLibs/GyverCore_uart.h"

extern uint32_t moment; // from timer2



namespace protocol {
    
Console::Console():
    bq(bq769x_conf, bq769x_data, bq769x_stats),
    m_lastUpdate(0),
    m_oldMillis(0),
    m_millisOverflows(0)
{
    conf_default();
    m_BatCycles_prev    = bq769x_stats.batCycles_;
    m_ChargedTimes_prev = bq769x_stats.chargedTimes_;
    shd = 255;
}

void Console::conf_begin_protect() {
    bq.setShortCircuitProtection(           bq769x_conf.Cell_SCD_mA, bq769x_conf.Cell_SCD_us);
    bq.setOvercurrentChargeProtection(      bq769x_conf.Cell_OCD_mA, bq769x_conf.Cell_OCD_ms);
    bq.setOvercurrentDischargeProtection(   bq769x_conf.Cell_ODP_mA, bq769x_conf.Cell_ODP_ms);
    bq.setCellUndervoltageProtection(       bq769x_conf.Cell_UVP_mV, bq769x_conf.Cell_UVP_sec);
    bq.setCellOvervoltageProtection(        bq769x_conf.Cell_OVP_mV, bq769x_conf.Cell_OVP_sec);
}

void Console::begin() {
    bq.begin();
    bq.update();
    bq.resetSOC(100);
    bq.enableCharging();
    conf_begin_protect();
    
    if (bq769x_conf.Allow_Discharging) {
        bq.enableDischarging();
    } else {
        bq.disableDischarging();
    }
    
}

void Console::conf_default() {
    bq769x_conf.Allow_Charging    = true;
    bq769x_conf.Allow_Discharging = true;
    bq769x_conf.RT_bits    = BQ769X0_THERMISTORS;
    bq769x_conf.RS_uOhm    = 1000; // Shunt, 1mOhm
    bq769x_conf.RT_Beta[0] = 3435; // for Semitec 103AT-5 thermistor
#ifdef IC_BQ76930
    bq76940_conf.RT_Beta[1] = 3435;
#endif
#ifdef IC_BQ76940
    bq769x_conf.RT_Beta[1] = 3435;
    bq769x_conf.RT_Beta[2] = 3435;
#endif
    // Capacity calc
    bq769x_conf.Cell_CapaNom_mV         = 3600;     // mV, nominal voltage for single cell
    bq769x_conf.Cell_CapaFull_mV        = 4180;     // mV, full voltage for single cell
    bq769x_conf.Batt_CapaNom_mAsec      = 360000;   // mA*sec, nominal capacity of battery pack, max. 580 Ah possible @ 3.7V

    bq769x_conf.CurrentThresholdIdle_mA = 100;  // 30 Current (mA)

    // TODO Temperature for any sensors
    bq769x_conf.Cell_TempCharge_min     =    0; // Temperature limits (Cx10)
    bq769x_conf.Cell_TempCharge_max     =  500; // Temperature limits (Cx10)
    bq769x_conf.Cell_TempDischarge_min  = -200; // Temperature limits (Cx10)
    bq769x_conf.Cell_TempDischarge_max  =  650; // Temperature limits (Cx10)
    
    
    bq769x_conf.BalancingInCharge       = true; // false
    bq769x_conf.BalancingEnable         = true; // false
    bq769x_conf.BalancingCellMin_mV     = 3600; // Cell voltage (mV)
    bq769x_conf.BalancingCellMaxDifference_mV   = 10;   // 20 
    bq769x_conf.BalancingIdleTimeMin_s          = 1800;
   
    // checkUser Cell overcurrent charge protection
    bq769x_conf.Cell_OCD_mA    = 5500;
    bq769x_conf.Cell_OCD_ms    = 3000;
    
    // PROTECT1 Cell short circuit protection
    bq769x_conf.Cell_SCD_mA    = 80000;
    bq769x_conf.Cell_SCD_us    = 200;
    
    // PROTECT2 Cell overcurrent discharge protection
    bq769x_conf.Cell_ODP_mA    = 40000;
    bq769x_conf.Cell_ODP_ms    = 2000;

    // PROTECT3 Cell voltage protection limits
    bq769x_conf.Cell_OVP_mV    = 4200;
    bq769x_conf.Cell_OVP_sec   = 2;
    // min
    bq769x_conf.Cell_UVP_mV    = 2850;
    bq769x_conf.Cell_UVP_sec   = 2;

    memset(bq769x_conf.adcCellsOffset_, 0, sizeof(bq769x_conf.adcCellsOffset_));
    
}

devices::bq769_stats EEMEM In_EEPROM_stats;

void Console::stats_load() {
    //LOG_P(PSTR("Stats load "));
    eeprom_read_block(&bq769x_stats, &In_EEPROM_stats, sizeof(bq769x_stats));
    if (bq769x_stats.crc8 != gencrc8((uint8_t*)&bq769x_stats, sizeof(bq769x_stats)-1)) {
        memset(&bq769x_stats, 0, sizeof(bq769x_stats));
        stats_save();
    }
}

void Console::stats_save() {
    bq769x_stats.ts = moment;
    bq769x_stats.crc8 = gencrc8((uint8_t*)&bq769x_stats, sizeof(bq769x_stats)-1);
    eeprom_write_block(&bq769x_stats, &In_EEPROM_stats, sizeof(bq769x_stats));
}

static volatile uint8_t xxx = 0;

// return true if system active
bool Console::update(const bool force) {
    bool result = false;
    
    while (uart.available()) {
        proto.put_char(uart.read());
        result = true;
    }
    proto.update();
    
    bq769x_data.alertInterruptFlag_ = force;
    
    if(moment - m_lastUpdate >= 250) { // 250
        uint8_t error = bq.update();
        if (error) { result = true; }
        m_lastUpdate = moment;
        sorting();
        proto.set_Health(99); // TODO
        proto.set_Voltage_Pack(bq769x_data.batVoltage_/10);
        for (uint8_t i = 0; i < OUTPUT_COUNT; ++i) {
            proto.set_Cell_Voltage(i, arr_out[i]);
        }
        proto.set_Voltage_Pack(bq769x_data.batVoltage_/10);
        proto.set_Temp(bq769x_stats.temperatures_[0]/10, bq769x_stats.temperatures_[1]/10);
        uint16_t m_MinCV = 4200;
        for (uint8_t x = Reg_Cell_0; x <= Reg_Cell_9; ++x) {
            if (arr_out[x] < m_MinCV) { m_MinCV = arr_out[x]; }
        }
//         uint16_t m_MaxCV = 0;
//         for (uint8_t x = Reg_Cell_0; x <= Reg_Cell_9; ++x) {
//             if (arr_out[x] < m_MaxCV) { m_MaxCV = arr_out[x]; }
//         }
//         proto.set_Status_bit(b_UnderVoltage, (m_MinCV <= 2900));
//         proto.set_Status_bit(b_OverVoltage, (m_MaxCV >= 4200));
        proto.set_Status_bit(b_OverVoltage,  !!(error & STAT_OV));
        proto.set_Status_bit(b_UnderVoltage, !!(error & STAT_UV));
        
        if (!!(error & STAT_UV)) { --shd; }
        if (!!(error & STAT_OV)) { --shd; }
        if (shd == 0) { bq.shutdown(); }
        //if(error & STAT_SCD) { cout << PGM << PSTR("Short Circuit Protection!\r\n"); }
        //if(error & STAT_OCD) { cout << PGM << PSTR("Overcurrent Charge Protection!\r\n"); }
        proto.set_Current(bq769x_data.batCurrent_/10);
        proto.set_Percentage((uint8_t)((float)(m_MinCV - 3100) * 0.091F));

//         if (bq769x_stats.batCycles_ != m_BatCycles_prev) {
//             m_BatCycles_prev    = bq769x_stats.batCycles_;
//             stats_save();
//         }
//         if (bq769x_stats.chargedTimes_ != m_ChargedTimes_prev) {
//             m_ChargedTimes_prev = bq769x_stats.chargedTimes_;
//             stats_save();
//         }

        if(m_oldMillis > moment) { m_millisOverflows++; }
        m_oldMillis = moment;
    }
    return result;
}


typedef void (*do_reboot_t)(void);
const do_reboot_t do_reboot = (do_reboot_t)((FLASHEND - 511) >> 1);

void Console::command_bootloader() {
    mcu::Watchdog::disable();
    cli();
    TCCR0A = 0;
    TCCR1A = 0;
    TCCR2A = 0;
    MCUSR = 0;
    do_reboot();
}

uint8_t gencrc8(uint8_t *data, uint16_t len) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc = devices::_crc8_ccitt_update(crc, data[i]);
    }
    return crc;
}

void Console::command_format_EEMEM() {
    for (int i = 0 ; i < E2END + 1 ; i++) {
        eeprom_write_byte((uint8_t*)i, 0xff);
        uart.write('.');
        mcu::Watchdog::reset();
    }
}
// sorting little..biG
void sort_numbers_top(uint16_t number[], uint8_t count) {
    uint16_t temp;
    for (uint8_t j = 0; j < count; ++j) {
        for (uint8_t k = j + 1; k < count; ++k) {
            if (number[j] > number[k]) {
                temp = number[j];
                number[j] = number[k];
                number[k] = temp;
            }
        }
    }
}

void Console::sorting() {
    for (uint8_t i = 0; i < MAX_NUMBER_OF_CELLS; ++i) { arr_connected[i] = 0; }
    for (uint8_t i = 0; i < OUTPUT_COUNT; ++i) { arr_out[i] = 0; }
    cell_count = 0;
    for (uint8_t i = 0; i <  MAX_NUMBER_OF_CELLS; ++i) {
        if (bq769x_stats.cellVoltages_[i] > 256) {
            arr_connected[cell_count++] = bq769x_stats.cellVoltages_[i];
        }
    }
    
    sort_numbers_top(arr_connected, cell_count);

    if (cell_count > OUTPUT_COUNT) {
        int8_t b = OUTPUT_COUNT - 1;
        for (uint8_t a = 0; a < OUTPUT_COUNT/2; ++a) {
            arr_out[a] = arr_connected[a];
            arr_out[b--] = arr_connected[cell_count-a-1];
        }
    } else if (OUTPUT_COUNT > cell_count) {
        uint16_t summ = 0;
        for (uint8_t i = 0; i < cell_count; ++i) { summ += arr_connected[i]; }
        summ /= cell_count;
        for (uint8_t i = 0; i < OUTPUT_COUNT; ++i) { arr_out[i] = summ; }
        for (uint8_t i = 0; i < cell_count/2; ++i) { arr_out[i] = arr_connected[i]; }
        uint8_t x = OUTPUT_COUNT - 1;
        for (uint8_t i = cell_count-1; i > cell_count/2; --i) {
            arr_out[x--] = arr_connected[i];
        }
    } else {
        for (uint8_t i = 0; i < cell_count; ++i) { arr_out[i] = arr_connected[i]; }
    }
    

    
}


void Console::debug_print() {
    
    uart.print(F("not365 emulation app, Copyright (c) 2022 Sergey Kostanoy, https://arduino.uno\r\n"));
    uart.print(F("WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND!\r\n"));
    uart.print(F("Version 0.1 Alpha. USING IT IS YOUR RISK!\r\n"));
    
    uart.print(F("\r\nBAT Temp(C): "));
    uart.print(bq.getTemperatureDegC(0));
    uart.print(' ');
    uart.print(bq.getTemperatureDegC(1));
    uart.print(' ');
    uart.print(bq.getTemperatureDegC(2));
    
    uart.print(F("\r\nBAT Voltage: "));
    uart.print(bq769x_data.batVoltage_);
    uart.print(F(" mV ("));
    uart.print(bq769x_data.batVoltage_raw_);
    uart.print(F(" raw)\r\nCurrent: "));
    uart.print(bq769x_data.batCurrent_);
    uart.print(F(" mA ("));
    uart.print(bq769x_data.batCurrent_raw_);
    uart.print(F(" raw)\r\n"));
    uart.print(F("SOC: "));
    uart.print(bq.getSOC());
    uart.print(F(" Balancing status: "));
    uart.print(bq769x_data.balancingStatus_);
    uart.print(F("\r\nCell voltages:\r\n"));
    
    for(uint8_t x = 0; x < MAX_NUMBER_OF_CELLS; x++) {
        uint8_t y = bq769x_stats.cellIdMap_[x];
        uart.print(bq769x_stats.cellVoltages_[y]);
        uart.print(F(" mV (")); 
        uart.print(bq769x_data.cellVoltages_raw_[y]);
        uart.print(F(" raw)\t"));
        if ((x+1) % 3 == 0) uart.print(F("\r\n"));
    }
    
    uart.print(F("\r\nCell mV: Min: "));
    uart.print(bq.getMinCellVoltage());
    uart.print(F(" | Avg: "));
    uart.print(bq.getAvgCellVoltage());
    uart.print(F(" | Max: "));
    uart.print(bq.getMaxCellVoltage());
    uart.print(F(" | Delta: "));
    uart.print(bq.getMaxCellVoltage() - bq.getMinCellVoltage());
    uart.print(F("\r\nXREADY errors: "));
    uart.print(bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_XREADY]);
    uart.print(F("\r\n ALERT errors: "));
    uart.print(bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_ALERT]);
    uart.print(F("\r\n   UVP errors: "));
    uart.print(bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_UVP]);
    uart.print(F("\r\n   OVP errors: "));
    uart.print(bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_OVP]);
    uart.print(F("\r\n   SCD errors: "));
    uart.print(bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_SCD]);
    uart.print(F("\r\n   OCD errors: "));
    uart.print(bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_OCD]);
    uart.print(F("\r\nDISCHG TEMP errors: "));
    uart.print(bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_USER_DISCHG_TEMP]);
    uart.print(F("\r\n   CHG TEMP errors: "));
    uart.print(bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_USER_CHG_TEMP]);
    uart.print(F("\r\n    CHG OCD errors: "));
    uart.print(bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_USER_CHG_OCD]);
    uart.print(F("\r\n"));
    bq.printRegisters();
}


}
