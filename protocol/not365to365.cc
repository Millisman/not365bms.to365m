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

#include "not365to365.h"
#include "console_strings.h"
#include <stdlib.h>
#include "mcu/watchdog.h"
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <stdio.h>


#define BUFFER_SIZE 136
#define UART_RX_BUFFER_SIZE BUFFER_SIZE
#define UART_TX_BUFFER_SIZE BUFFER_SIZE

#define LOG(...)   fprintf(cout, __VA_ARGS__)
#define LOG_P(...) fprintf_P(cout, __VA_ARGS__)

namespace {

static volatile uint8_t UART_TX_BUFFER[UART_TX_BUFFER_SIZE];
static volatile uint8_t UART_TX_BUFFER_HEAD;
static volatile uint8_t UART_TX_BUFFER_TAIL;

ISR(USART_TX_vect) {
    if (UART_TX_BUFFER_HEAD != UART_TX_BUFFER_TAIL) {
        uint8_t c = UART_TX_BUFFER[UART_TX_BUFFER_TAIL];
        if (++UART_TX_BUFFER_TAIL >= UART_TX_BUFFER_SIZE) UART_TX_BUFFER_TAIL = 0;  // хвост двигаем
        UDR0 = c;
    }
}

static volatile uint8_t     UART_RX_BUF[UART_RX_BUFFER_SIZE];
static volatile uint16_t    UART_RX_BUF_HEAD;
static volatile uint16_t    UART_RX_BUF_TAIL;
static volatile bool        UART_RX_BUF_FULL;

ISR(USART_RX_vect) {
    if (bit_is_set(UCSR0A, UPE0)) {
        UDR0; // parity error! read
    } else {
        uint8_t c = UDR0;
        if (!UART_RX_BUF_FULL) {
            UART_RX_BUF[UART_RX_BUF_HEAD] = c;
            UART_RX_BUF_HEAD++;
            if (UART_RX_BUF_HEAD >= BUFFER_SIZE) { UART_RX_BUF_HEAD -= BUFFER_SIZE; }
            if (UART_RX_BUF_HEAD == UART_RX_BUF_TAIL) { UART_RX_BUF_FULL = true; }
        }
    }
}

int uart_write(char data, FILE*) {
    // ------------- direct write (isr off) ------------
    //     while (!(UCSR0A & (1<<UDRE0)));
    //     UDR0 = c;
    uint8_t i = (UART_TX_BUFFER_HEAD + 1 >= UART_TX_BUFFER_SIZE) ? 0 : UART_TX_BUFFER_HEAD + 1;
    while ( (i + 1) == UART_TX_BUFFER_TAIL);
    if (i != UART_TX_BUFFER_TAIL) {
        UART_TX_BUFFER[UART_TX_BUFFER_HEAD] = data;
        UART_TX_BUFFER_HEAD = i;
    }
    while (!(UCSR0A & (1<<UDRE0)));
    if (UART_TX_BUFFER_HEAD != UART_TX_BUFFER_TAIL) {
        uint8_t c = UART_TX_BUFFER[UART_TX_BUFFER_TAIL];
        if (++UART_TX_BUFFER_TAIL >= UART_TX_BUFFER_SIZE) UART_TX_BUFFER_TAIL = 0;  // хвост двигаем
        UDR0 = c;
    }

    return data;
}

static FILE *cout = fdevopen(uart_write, NULL);

void uart0_begin() {
    UART_RX_BUF_HEAD = 0;
    UART_RX_BUF_TAIL = 0;
    UART_RX_BUF_FULL = false;
    UART_TX_BUFFER_HEAD = 0;
    UART_TX_BUFFER_TAIL = 0;
    
    UBRR0H = ((F_CPU / 4 / 115200 - 1) / 2) >> 8;
    UBRR0L = ((F_CPU / 4 / 115200 - 1) / 2) & 0xFF;
    UCSR0A = (1 << U2X0);
    UCSR0B = ((1<<TXEN0) | (1<<RXEN0) | (1<<RXCIE0)   | (1<<TXCIE0));
    UCSR0C = ((1<<UCSZ01) | (1<<UCSZ00));
    sei();
    LOG_P(PSTR("not365\r\n"));
}

uint8_t RxB_Get(uint16_t i) {
    uint16_t idx = UART_RX_BUF_TAIL + i;
    if (idx >= UART_RX_BUFFER_SIZE) { idx -= UART_RX_BUFFER_SIZE; };
    return UART_RX_BUF[(uint8_t)(idx)];
}

uint8_t RxB_Next(uint16_t i) {
    UART_RX_BUF_TAIL += i;
    if (UART_RX_BUF_TAIL >= UART_RX_BUFFER_SIZE) UART_RX_BUF_TAIL -= UART_RX_BUFFER_SIZE;
    UART_RX_BUF_FULL = false;
    return UART_RX_BUF_TAIL;
}

} // namespace

namespace protocol {

void Protocol::set_Cell_Voltage(M36Cells cell, uint16_t voltage) {
    if (m_Min_Cell_Voltage > voltage) m_Min_Cell_Voltage = voltage;
    if (m_Max_Cell_Voltage < voltage) m_Max_Cell_Voltage = voltage;
    switch (cell) {
        case Cell_0: m_360_p[Reg_Cell_0] = voltage; break;
        case Cell_1: m_360_p[Reg_Cell_1] = voltage; break;
        case Cell_2: m_360_p[Reg_Cell_2] = voltage; break;
        case Cell_3: m_360_p[Reg_Cell_3] = voltage; break;
        case Cell_4: m_360_p[Reg_Cell_4] = voltage; break;
        case Cell_5: m_360_p[Reg_Cell_5] = voltage; break;
        case Cell_6: m_360_p[Reg_Cell_6] = voltage; break;
        case Cell_7: m_360_p[Reg_Cell_7] = voltage; break;
        case Cell_8: m_360_p[Reg_Cell_8] = voltage; break;
        case Cell_9: m_360_p[Reg_Cell_9] = voltage; break;
    }
}

void Protocol::two8to16(uint8_t rega, uint8_t a, uint8_t b) {
    m_360_p[rega] = ((uint16_t)a | ((uint16_t)b) << 8);
}

Protocol::Protocol(uint16_t _ver, uint16_t _cap, uint16_t _date) {
    for (uint8_t x = 0; x < PACKET_SIZE; ++x) { m_360_p[x] = 0; }
    m_360_p[Reg_Version]    = _ver;
    m_360_p[Reg_Capacity]   = _cap;
    m_360_p[Reg_Date]  = _date; two8to16(Reg_Serial + 0, '3', 'J'); two8to16(Reg_Serial + 1, '_', 'N');
    two8to16(Reg_Serial + 2, 'O', 'T'); two8to16(Reg_Serial + 3, '3', '6'); two8to16(Reg_Serial + 4, '5', '_');
    two8to16(Reg_Serial + 5, 'B', 'M'); two8to16(Reg_Serial + 6, 'S', '1');
    m_360_p[Reg_Cell_0] = m_360_p[Reg_Cell_1] = m_360_p[Reg_Cell_2] = m_360_p[Reg_Cell_3] = m_360_p[Reg_Cell_4] =
    m_360_p[Reg_Cell_5] = m_360_p[Reg_Cell_6] = m_360_p[Reg_Cell_7] = m_360_p[Reg_Cell_8] = m_360_p[Reg_Cell_9] = 3000;
    m_360_p[Reg_Status] = 1;
    two8to16(Reg_Temp, (26 + 20), (27 + 20)); // 0 == -20C
    m_360_p[Reg_PackHealth] = 100;
    m_Undervolt_lvl = 2800;
    m_Overvolt_lvl  = 4210;
// // //     m_Batt_Voltage  = 420;
    m_Min_Cell_Voltage = m_Overvolt_lvl;
    m_Max_Cell_Voltage = m_Undervolt_lvl;
    m_Error = false;
    uart0_begin();
}

void Protocol::set_pack_voltage(uint16_t pV) {
    m_360_p[Reg_Voltage] = pV;
}

uint16_t Protocol::send_byte(uint8_t c) {
    uart_write(c, NULL);
    return (uint16_t)c;
}

uint16_t Protocol::send_360_p(uint8_t pos, uint16_t sz) {
    m_crc16 = 0;
    for (uint16_t x = pos; x < (pos + (sz/2)); x++) {
        m_crc16 += send_byte((uint8_t)((m_360_p[x & (PACKET_SIZE-1)]) & 0xFF));
        m_crc16 += send_byte((uint8_t)(((m_360_p[x & (PACKET_SIZE-1)]) >> 8) & 0xFF));
    }
    if (!!(sz & 0x01)) { m_crc16 += send_byte((uint8_t)((m_360_p[(uint8_t)((pos + 1 + (sz/2)) & (PACKET_SIZE-1))]) & 0xFF)); }
    return m_crc16;
}

void Protocol::protocol_send(const uint8_t cmd) {
    m_crc16 = 0;
    switch (cmd) {
        case 0x01: {
            send_byte(M36MagicChar::magic_First); send_byte(M36MagicChar::magic_Second);
            m_crc16 += send_byte(RxB_Get(6) + 2); // length
            m_crc16 += send_byte(M36ProtoDirect::to_Master); // addr
            m_crc16 += send_byte(cmd); // mode
            m_crc16 += send_byte(RxB_Get(5)); // offset
            m_crc16 += send_360_p(RxB_Get(5), RxB_Get(6));
            m_crc16 ^= 0xFFFF;
            send_byte((uint8_t)((m_crc16) & 0xFF));
            send_byte((uint8_t)((m_crc16 >> 8) & 0xFF));
            
            m_Min_Cell_Voltage = m_Overvolt_lvl;
            m_Max_Cell_Voltage = m_Undervolt_lvl;
        }
        break;
        case 0x03: break;
        case 0x07: break;
        case 0x08: break;
        case 0x09: break;
        case 0x0a: break;
        default: break;
    }
}

void Protocol::update() {
    
    // if (BatteryCurrent() > (int16_t)idle_currentThres) {
    //     m_360_p[Reg_Status] |= (1 << b_Charging);
    // } else {
    //     m_360_p[Reg_Status] &= ~(1 << b_Charging);
    // }
    
    // if (Temperature() > (temp_maxDischargeC - 3) * 10) {
    //     m_360_p[Reg_Status] |= (1 << b_OverHeat); // overheat
    // } else {
    //     m_360_p[Reg_Status] &= ~(1 << b_OverHeat);
    // }
    
    if (m_Min_Cell_Voltage <= m_Undervolt_lvl) {
        m_360_p[Reg_Status] |= (1 << b_UnderVoltage);
    } else {
        m_360_p[Reg_Status] &= ~(1 << b_UnderVoltage);
    }
    
    if (m_Max_Cell_Voltage >= m_Overvolt_lvl) {
        m_360_p[Reg_Status] |= (1 << b_OverVoltage);
    } else {
        m_360_p[Reg_Status] &= ~(1 << b_OverVoltage);
    }
    
    if (m_Error)
        m_360_p[Reg_Status] &= ~(1 << b_Ready);
    else
        m_360_p[Reg_Status] |= (1 << b_Ready);
    

    uint8_t percent = 0;
    if (m_Min_Cell_Voltage > 3000) { // bad method;
        percent = (uint8_t)((float)(m_Min_Cell_Voltage - 3000) * 0.092F);
    }
    if (percent > 100) percent = 100;
    
    m_360_p[Reg_Percentage] = percent;
    m_360_p[Reg_CapLeft] = (uint16_t)(((uint32_t)m_360_p[Reg_Percentage] * m_360_p[Reg_Capacity])/100);     
    
    cli();
    int16_t rxc = UART_RX_BUF_HEAD - UART_RX_BUF_TAIL;
    sei();
    
    if (rxc < 0) { rxc += BUFFER_SIZE; };
    if (UART_RX_BUF_FULL && rxc == 0) { rxc = BUFFER_SIZE; }
    m_crc16 = 0;
    looking = 2;
    if ((looking + 6) <= rxc) {
        if (RxB_Get(0) != M36MagicChar::magic_First && RxB_Get(1) != M36MagicChar::magic_Second) {
            RxB_Next(1);
        } else {
            if ((RxB_Get(2) + 6) < BUFFER_SIZE) looking = RxB_Get(2); else looking = 2;
            if ((looking + 6) <= rxc) {
                m_crc16 = 0;
                for (uint16_t x = looking + 3; x > 1; x--) { m_crc16 += RxB_Get(x); }
                m_crc16 = m_crc16^0xFFFF;
                if (RxB_Get(looking + 4) ==  (m_crc16 & 0xFF) &&
                    RxB_Get(looking + 5) == ((m_crc16 >> 8) & 0xFF) &&
                    RxB_Get(3) == M36ProtoDirect::to_Batt)
                {    
                    protocol_send(RxB_Get(4));
                    RxB_Next(looking + 6);
                } else RxB_Next(rxc);
            }
        }
    }
}


void Protocol::set_status_bit(M36StatusBits bit, bool state) {
    if (state) m_360_p[Reg_Status] |= (1 << bit);
    else m_360_p[Reg_Status] &= ~(1 << bit);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

bool Console::Recv() {
    bool result = false;
    proto.update();
    return result;
}
    
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
    bq.printRegisters();
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
    LOG_P(PSTR("Stats load "));
    eeprom_read_block(&bq769x_stats, &In_EEPROM_stats, sizeof(bq769x_stats));
    if (bq769x_stats.crc8 != gencrc8((uint8_t*)&bq769x_stats, sizeof(bq769x_stats)-1)) {
        memset(&bq769x_stats, 0, sizeof(bq769x_stats));
        stats_save();
    }
}

void Console::stats_save() {
    bq769x_stats.ts = mcu::Timer::millis();
    bq769x_stats.crc8 = gencrc8((uint8_t*)&bq769x_stats, sizeof(bq769x_stats)-1);
    eeprom_write_block(&bq769x_stats, &In_EEPROM_stats, sizeof(bq769x_stats));
}

bool Console::update(mcu::Pin job, const bool force) {
    bool result = force;
    bq769x_data.alertInterruptFlag_ = force;
    
    if(mcu::Timer::millis() - m_lastUpdate >= 250) { // 250
        m_lastUpdate = mcu::Timer::millis();
        // if (BatteryCurrent() > (int16_t)idle_currentThres) {
        //     m_360_p[Reg_Status] |= (1 << b_Charging);
        // } else {
        //     m_360_p[Reg_Status] &= ~(1 << b_Charging);
        // }
        
        // if (Temperature() > (temp_maxDischargeC - 3) * 10) {
        //     m_360_p[Reg_Status] |= (1 << b_OverHeat); // overheat
        // } else {
        //     m_360_p[Reg_Status] &= ~(1 << b_OverHeat);
        // }
        //         proto.set_status_bit(b_OverHeat, true);

        result = false;
        job = 1;
        uint8_t error = bq.update();

        sorting();
        proto.set_pack_voltage(bq769x_data.batVoltage_/100);
        proto.set_status_bit(b_OverVoltage,  !!(error & STAT_OV));
        proto.set_status_bit(b_UnderVoltage, !!(error & STAT_UV));
        //if(error & STAT_SCD) { cout << PGM << PSTR("Short Circuit Protection!\r\n"); }
        //if(error & STAT_OCD) { cout << PGM << PSTR("Overcurrent Charge Protection!\r\n"); }
        
        if (bq769x_stats.batCycles_ != m_BatCycles_prev) {
            m_BatCycles_prev    = bq769x_stats.batCycles_;
            stats_save();
        }
        if (bq769x_stats.chargedTimes_ != m_ChargedTimes_prev) {
            m_ChargedTimes_prev = bq769x_stats.chargedTimes_;
            stats_save();
        }
        
        proto.update();
        
        // uint16_t bigDelta = bq.getMaxCellVoltage() - bq.getMinCellVoltage();
        // if(bigDelta > 100) cout << PGM << PSTR("Difference too big!\r\n");
        if(m_oldMillis > mcu::Timer::millis())
            m_millisOverflows++;
        m_oldMillis = mcu::Timer::millis();
        job = 0;
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
        proto.send_byte('.');
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
        if (bq769x_stats.cellVoltages_[i] > 512) {
            arr_connected[cell_count++] = bq769x_stats.cellIdMap_[i];
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
    
    for (uint8_t i = 0; i < OUTPUT_COUNT; ++i) { proto.set_Cell_Voltage((M36Cells)i, arr_out[i]); }
    
}





}
