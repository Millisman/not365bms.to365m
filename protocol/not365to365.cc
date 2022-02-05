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

#include "not365to365.h"
#include "console_strings.h"
#include <stdlib.h>
#include "mcu/watchdog.h"
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

using stream::Flags::PGM;

#define BUFFER_SIZE 136
#define UART_RX_BUFF_SIZE BUFFER_SIZE
#define UART_TX_BUFF_SIZE BUFFER_SIZE

namespace {

static volatile uint8_t UART_TX_BUFF_HEAD;
static volatile uint8_t UART_TX_BUFF_TAIL;
static volatile uint8_t UART_TX_BUFF[UART_TX_BUFF_SIZE];

ISR(USART0_TX_vect) {
    if (UART_TX_BUFF_HEAD != UART_TX_BUFF_TAIL) {
        uint8_t c = UART_TX_BUFF[UART_TX_BUFF_TAIL];
        if (++UART_TX_BUFF_TAIL >= UART_TX_BUFF_SIZE) UART_TX_BUFF_TAIL = 0;  // хвост двигаем
        UDR0 = c;
    }
}

static volatile uint8_t     UART_RX_BUF[UART_RX_BUFF_SIZE];
static volatile uint16_t    UART_RX_BUF_HEAD;
static volatile uint16_t    UART_RX_BUF_TAIL;
static volatile bool        UART_RX_BUF_FULL;

ISR(USART1_RX_vect) {
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

void uart0_begin() {
    UART_RX_BUF_HEAD = 0;
    UART_RX_BUF_TAIL = 0;
    UART_RX_BUF_FULL = false;
    UART_TX_BUFF_HEAD = 0;
    UART_TX_BUFF_TAIL = 0;
    UBRR0H = ((F_CPU / 4 / 57600 - 1) / 2) >> 8;
    UBRR0L = ((F_CPU / 4 / 57600 - 1) / 2) & 0xFF;
    UCSR0A = (1 << U2X0);
    UCSR0B = ((1<<TXEN0) | (1<<RXEN0) | (1<<RXCIE0) | (1<<TXCIE0));
    UCSR0C = ((1<<UCSZ01) | (1<<UCSZ00));
    sei();
}


void uart_write(const char c) {
    // ------------- direct write ------------
    // while (!(UCSR1A & (1<<UDRE1)));
    // UDR1 = data;
    // ------------- buffer write ------------
    uint8_t i = (uint16_t)(UART_TX_BUFF_HEAD + 1 >= UART_TX_BUFF_SIZE) ? 0 : UART_TX_BUFF_HEAD + 1;
    while ( (i + 1) == UART_TX_BUFF_TAIL);
    if (i != UART_TX_BUFF_TAIL) {
        UART_TX_BUFF[UART_TX_BUFF_HEAD] = c;
        UART_TX_BUFF_HEAD = i;
    }
    
    // ---------- start Transmission ---------
    while (!(UCSR0A & (1<<UDRE0)));
    if (UART_TX_BUFF_HEAD != UART_TX_BUFF_TAIL) {
        uint8_t c = UART_TX_BUFF[UART_TX_BUFF_TAIL];
        if (++UART_TX_BUFF_TAIL >= UART_TX_BUFF_SIZE) UART_TX_BUFF_TAIL = 0; 
        UDR0 = c;
    }
}

uint8_t RxB_Get(uint16_t i) {
    uint16_t idx = UART_RX_BUF_TAIL + i;
    if (idx >= UART_RX_BUFF_SIZE) { idx -= UART_RX_BUFF_SIZE; };
    return UART_RX_BUF[(uint8_t)(idx)];
}

uint8_t RxB_Next(uint16_t i) {
    UART_RX_BUF_TAIL += i;
    if (UART_RX_BUF_TAIL >= UART_RX_BUFF_SIZE) UART_RX_BUF_TAIL -= UART_RX_BUFF_SIZE;
    UART_RX_BUF_FULL = 0;
    return UART_RX_BUF_TAIL;
}
    
} // namespace

namespace protocol {

void Protocol::set_Cell_Voltage(M36Cells cell, uint16_t voltage) {
    if (m_Min_Cell_Voltage > voltage) m_Min_Cell_Voltage = voltage;
    if (m_Max_Cell_Voltage < voltage) m_Max_Cell_Voltage = voltage;
    switch (cell) {
        case Cell_0: m_360_p[Reg_Cell_0] = voltage; break;
        case Cell_1: m_360_p[Reg_Cell_0] = voltage; break;
        case Cell_2: m_360_p[Reg_Cell_0] = voltage; break;
        case Cell_3: m_360_p[Reg_Cell_0] = voltage; break;
        case Cell_4: m_360_p[Reg_Cell_0] = voltage; break;
        case Cell_5: m_360_p[Reg_Cell_0] = voltage; break;
        case Cell_6: m_360_p[Reg_Cell_0] = voltage; break;
        case Cell_7: m_360_p[Reg_Cell_0] = voltage; break;
        case Cell_8: m_360_p[Reg_Cell_0] = voltage; break;
        case Cell_9: m_360_p[Reg_Cell_0] = voltage; break;
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
    m_Batt_Voltage  = 420;
    m_Min_Cell_Voltage = m_Overvolt_lvl;
    m_Max_Cell_Voltage = m_Undervolt_lvl;
    m_Error = false;
    uart0_begin();
}

uint16_t Protocol::send_byte(uint8_t c) {
    uart_write(c);
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
    
    m_360_p[Reg_Voltage] = m_Batt_Voltage;
    
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
    if (UART_RX_BUF_FULL == 1 && rxc == 0) { rxc = BUFFER_SIZE; }
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


bool Console::Recv() {
    bool result = false;
    proto.update();
    return result;
}




    
Console::Console():
    ser(mcu::Usart::get()),
    cout(ser),
    bq(bq769x_conf, bq769x_data, bq769x_stats),
    m_lastUpdate(0),
    m_oldMillis(0),
    m_millisOverflows(0)
{
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
//     debug_print();
//     print_all_conf();
//     print_all_stats();
}

void Console::conf_default() {
    bq769x_conf.BQ_dbg            = false;
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

void Console::print_conf(const PrintParam c) {
    switch (c) {
        case Conf_Allow_Charging:
            cout << PGM << STR_cmd_Allow_Charging << '=' <<
            bq769x_conf.Allow_Charging;
            cout << PGM << STR_cmd_Allow_Charging_HELP;
            break;
            break;
        case Conf_Allow_Discharging:
            cout << PGM << STR_cmd_Allow_Discharging << '=' <<
            bq769x_conf.Allow_Discharging;
            cout << PGM << STR_cmd_Allow_Discharging_HELP;
            break;
            break;
        case Conf_BQ_dbg:
            cout << PGM << STR_cmd_BQ_dbg << '=' <<
            bq769x_conf.BQ_dbg;
            cout << PGM << STR_cmd_BQ_dbg_HELP;
            break;
        case Conf_RT_bits:
            cout << PGM << STR_cmd_RT_bits << '=';
            if (bq769x_conf.RT_bits & (1 << 0)) cout << '1'; else cout << '0'; cout << ' ';
            if (bq769x_conf.RT_bits & (1 << 1)) cout << '1'; else cout << '0'; cout << ' ';
            if (bq769x_conf.RT_bits & (1 << 2)) cout << '1'; else cout << '0';
            cout << PGM << STR_cmd_RT_bits_HELP;
            break;
            
        case Conf_RS_uOhm:
            cout << PGM << STR_cmd_RS_uOhm << '=' <<
            bq769x_conf.RS_uOhm;
            cout << PGM << STR_cmd_RS_uOhm_HELP;
            break;
            
        case Conf_RT_Beta:
            cout << PGM << STR_cmd_RT_Beta << '=' <<
            bq769x_conf.RT_Beta[0] << ' ' <<
            bq769x_conf.RT_Beta[1] << ' ' <<
            bq769x_conf.RT_Beta[2];
            cout << PGM << STR_cmd_RT_Beta_HELP;
            break;
            
        case Conf_Cell_CapaNom_mV:
            cout << PGM << STR_cmd_Cell_CapaNom_mV << '=' <<
            bq769x_conf.Cell_CapaNom_mV;
            cout << PGM << STR_cmd_Cell_CapaNom_mV_HELP;
            break;
            
        case Conf_Cell_CapaFull_mV:
            cout << PGM << STR_cmd_Cell_CapaFull_mV << '=' <<
            bq769x_conf.Cell_CapaFull_mV;
            cout << PGM << STR_cmd_Cell_CapaFull_mV_HELP;
            break;
            
        case Conf_Batt_CapaNom_mAsec:
            cout << PGM << STR_cmd_Batt_CapaNom_mAsec << '=' <<
            bq769x_conf.Batt_CapaNom_mAsec/ 60 * 60;
            cout << PGM << STR_cmd_Batt_CapaNom_mAsec_HELP;
            break;
            
        case Conf_CurrentThresholdIdle_mA:
            cout << PGM << STR_cmd_CurrentThresholdIdle_mA << '=' <<
            bq769x_conf.CurrentThresholdIdle_mA;
            cout << PGM << STR_cmd_CurrentThresholdIdle_mA_HELP;
            break;
            
        case Conf_Cell_TempCharge_min:
            cout << PGM << STR_cmd_Cell_TempCharge_min << '=' <<
            bq769x_conf.Cell_TempCharge_min;
            cout << PGM << STR_cmd_Cell_TempCharge_min_HELP;
            break;
            
        case Conf_Cell_TempCharge_max:
            cout << PGM << STR_cmd_Cell_TempCharge_max << '=' <<
            bq769x_conf.Cell_TempCharge_max;
            cout << PGM << STR_cmd_Cell_TempCharge_max_HELP;
            break;
            
        case Conf_Cell_TempDischarge_min:
            cout << PGM << STR_cmd_Cell_TempDischarge_min << '=' <<
            bq769x_conf.Cell_TempDischarge_min;
            cout << PGM << STR_cmd_Cell_TempDischarge_min_HELP;
            break;
            
        case Conf_Cell_TempDischarge_max:
            cout << PGM << STR_cmd_Cell_TempDischarge_max << '=' <<
            bq769x_conf.Cell_TempDischarge_max;
            cout << PGM << STR_cmd_Cell_TempDischarge_max_HELP;
            break;
            
        case Conf_BalancingInCharge:
            cout << PGM << STR_cmd_BalancingInCharge << '=' <<
            bq769x_conf.BalancingInCharge;
            cout << PGM << STR_cmd_BalancingInCharge_HELP;
            break;
            
        case Conf_BalancingEnable:
            cout << PGM << STR_cmd_BalancingEnable << '=' <<
            bq769x_conf.BalancingEnable;
            cout << PGM << STR_cmd_BalancingEnable_HELP;
            break;
            
        case Conf_BalancingCellMin_mV:
            cout << PGM << STR_cmd_BalancingCellMin_mV << '=' <<
            bq769x_conf.BalancingCellMin_mV;
            cout << PGM << STR_cmd_BalancingCellMin_mV_HELP;
            break;
            
        case Conf_BalancingCellMaxDifference_mV:
            cout << PGM << STR_cmd_BalancingCellMaxDifference_mV << '=' <<
            bq769x_conf.BalancingCellMaxDifference_mV;
            cout << PGM << STR_cmd_BalancingCellMaxDifference_mV_HELP;
            break;
            
        case Conf_BalancingIdleTimeMin_s:
            cout << PGM << STR_cmd_BalancingIdleTimeMin_s << '=' <<
            bq769x_conf.BalancingIdleTimeMin_s;
            cout << PGM << STR_cmd_BalancingIdleTimeMin_s_HELP;
            break;
            
        case Conf_Cell_OCD_mA:
            cout << PGM << STR_cmd_Cell_OCD_mA << '=' <<
            bq769x_conf.Cell_OCD_mA;
            cout << PGM << STR_cmd_Cell_OCD_mA_HELP;
            break;
            
        case Conf_Cell_OCD_ms:
            cout << PGM << STR_cmd_Cell_OCD_ms << '=' <<
            bq769x_conf.Cell_OCD_ms;
            cout << PGM << STR_cmd_Cell_OCD_ms_HELP;
            break;
            
        case Conf_Cell_SCD_mA:
            cout << PGM << STR_cmd_Cell_SCD_mA << '=' <<
            bq769x_conf.Cell_SCD_mA;
            cout << PGM << STR_cmd_Cell_SCD_mA_HELP;
            break;
            
        case Conf_Cell_SCD_us:
            cout << PGM << STR_cmd_Cell_SCD_us << '=' <<
            bq769x_conf.Cell_SCD_us;
            cout << PGM << STR_cmd_Cell_SCD_us_HELP;
            break;
            
        case Conf_Cell_ODP_mA:
            cout << PGM << STR_cmd_Cell_ODP_mA << '=' <<
            bq769x_conf.Cell_ODP_mA;
            cout << PGM << STR_cmd_Cell_ODP_mA_HELP;
            break;
            
        case Conf_Cell_ODP_ms:
            cout << PGM << STR_cmd_Cell_ODP_ms << '=' <<
            bq769x_conf.Cell_ODP_ms;
            cout << PGM << STR_cmd_Cell_ODP_ms_HELP;
            break;
            
        case Conf_Cell_OVP_mV:
            cout << PGM << STR_cmd_Cell_OVP_mV << '=' <<
            bq769x_conf.Cell_OVP_mV;
            cout << PGM << STR_cmd_Cell_OVP_mV_HELP;
            break;
            
        case Conf_Cell_OVP_sec:
            cout << PGM << STR_cmd_Cell_OVP_sec << '=' <<
            bq769x_conf.Cell_OVP_sec;
            cout << PGM << STR_cmd_Cell_OVP_sec_HELP;
            break;
            
        case Conf_Cell_UVP_mV:
            cout << PGM << STR_cmd_Cell_UVP_mV << '=' <<
            bq769x_conf.Cell_UVP_mV;
            cout << PGM << STR_cmd_Cell_UVP_mV_HELP;
            break;
            
        case Conf_Cell_UVP_sec:
            cout << PGM << STR_cmd_Cell_UVP_sec << '=' <<
            bq769x_conf.Cell_UVP_sec;
            cout << PGM << STR_cmd_Cell_UVP_sec_HELP;
            break;
            
        case Conf_adcCellsOffset:
            cout << "TODO";
            break;
            
        case Conf_ts:
            cout << "TS: " << bq769x_conf.ts;
            break;
            
        case Conf_CRC8:
            cout << "CRC8: " << bq769x_conf.crc8;
            break;
        default: {
            cout << '?';
        }
    }
}

void Console::print_all_conf() {
    for (uint8_t i = FIRST ; i <= LAST; i++) {
        print_conf((PrintParam)i);
        cout << EOL;
    }
}

char const STR_TS[] PROGMEM = " timestamp = ";

void Console::print_all_stats() {
    cout
        << PGM << PSTR("ADC Gain=") << bq769x_stats.adcGain_
        << PGM << PSTR(" Offset=")  << bq769x_stats.adcOffset_
        << PGM << PSTR("\r\nBAT Cycles=") << bq769x_stats.batCycles_
        << PGM << PSTR(" Charged times=")  << bq769x_stats.chargedTimes_
        << PGM << PSTR("\r\nLook Cell mVmin=") << bq769x_stats.idCellMinVoltage_
        << PGM << PSTR(" mVmax=")  << bq769x_stats.idCellMaxVoltage_
        << PGM << PSTR("\r\nTimestamp idle=") << bq769x_stats.idleTimestamp_
        << PGM << PSTR(" charge=")  << bq769x_stats.chargeTimestamp_
        << PGM << PSTR(" saved in EEPROM=")  << bq769x_stats.ts;
        
    cout
        << PGM << PSTR("\r\nErrors counter:")
        << PGM << PSTR("\r\nXREADY = ") << bq769x_stats.errorCounter_[devices::ERROR_XREADY] << PGM << STR_TS << bq769x_stats.errorTimestamps_[devices::ERROR_XREADY]
        << PGM << PSTR("\r\n ALERT = ") << bq769x_stats.errorCounter_[devices::ERROR_ALERT] << PGM << STR_TS << bq769x_stats.errorTimestamps_[devices::ERROR_ALERT]
        << PGM << PSTR("\r\n   UVP = ") << bq769x_stats.errorCounter_[devices::ERROR_UVP] << PGM << STR_TS << bq769x_stats.errorTimestamps_[devices::ERROR_UVP]
        << PGM << PSTR("\r\n   OVP = ") << bq769x_stats.errorCounter_[devices::ERROR_OVP] << PGM << STR_TS << bq769x_stats.errorTimestamps_[devices::ERROR_OVP]
        << PGM << PSTR("\r\n   SCD = ") << bq769x_stats.errorCounter_[devices::ERROR_SCD] << PGM << STR_TS << bq769x_stats.errorTimestamps_[devices::ERROR_SCD]
        << PGM << PSTR("\r\n   OCD = ") << bq769x_stats.errorCounter_[devices::ERROR_OCD] << PGM << STR_TS << bq769x_stats.errorTimestamps_[devices::ERROR_OCD]
        << PGM << PSTR("\r\n     USR_SWITCH = ") << bq769x_stats.errorCounter_[devices::ERROR_USER_SWITCH] << PGM << STR_TS << bq769x_stats.errorTimestamps_[devices::ERROR_USER_SWITCH]
        << PGM << PSTR("\r\nUSR_DISCHG_TEMP = ") << bq769x_stats.errorCounter_[devices::ERROR_USER_DISCHG_TEMP] << PGM << STR_TS << bq769x_stats.errorTimestamps_[devices::ERROR_USER_DISCHG_TEMP]
        << PGM << PSTR("\r\n   USR_CHG_TEMP = ") << bq769x_stats.errorCounter_[devices::ERROR_USER_CHG_TEMP] << PGM << STR_TS << bq769x_stats.errorTimestamps_[devices::ERROR_USER_CHG_TEMP]
        << PGM << PSTR("\r\n    USR_CHG_OCD = ") << bq769x_stats.errorCounter_[devices::ERROR_USER_CHG_OCD] << PGM << STR_TS << bq769x_stats.errorTimestamps_[devices::ERROR_USER_CHG_OCD];

    cout << PGM << PSTR("\r\nCell ID Map:\r\n");
    for (uint8_t i = 0; i < MAX_NUMBER_OF_CELLS; i++) {
        cout << i << " = " << bq769x_stats.cellIdMap_[i] << '\t';
        if ((i+1) % 3 == 0) cout << EOL;
    }
    cout << PGM << PSTR("Cell Voltages:\r\n");
    for (uint8_t i = 0; i < MAX_NUMBER_OF_CELLS; i++) {
        cout << i << " = " << bq769x_stats.cellVoltages_[i] << " mV\t";
        if ((i+1) % 3 == 0) cout << EOL;
    }
    cout << PGM << PSTR("Temperatures x10C: ");
    for (uint8_t i = 0; i < MAX_NUMBER_OF_THERMISTORS; i++) {
        cout << i << " = " << bq769x_stats.temperatures_[i] << ' ';
    }
}


devices::bq769_stats EEMEM In_EEPROM_stats;

void Console::stats_load() {
    cout << PGM << PSTR("Stats load ");
    eeprom_read_block(&bq769x_stats, &In_EEPROM_stats, sizeof(bq769x_stats));
    if (bq769x_stats.crc8 != gencrc8((uint8_t*)&bq769x_stats, sizeof(bq769x_stats)-1)) {
        cout << PGM << PSTR("bad crc, restore zero");
        memset(&bq769x_stats, 0, sizeof(bq769x_stats));
        stats_save();
    } else cout << "OK";
    cout << EOL;
}

void Console::stats_save() {
    bq769x_stats.ts = mcu::Timer::millis();
    bq769x_stats.crc8 = gencrc8((uint8_t*)&bq769x_stats, sizeof(bq769x_stats)-1);
    eeprom_write_block(&bq769x_stats, &In_EEPROM_stats, sizeof(bq769x_stats));
}

bool Console::update(mcu::Pin job, const bool force) {
    bool result = force;
    bq769x_data.alertInterruptFlag_ = force;
    uint32_t now = mcu::Timer::millis();
    if(now - m_lastUpdate >= 250) { // 250
//     if(force || (uint32_t)(now - m_lastUpdate) >= 250) { // 250
        result = false;
        job = 1;
        uint8_t error = bq.update(); // should be called at least every 250 ms
        m_lastUpdate = now;
        if(error & STAT_OV)  { cout << PGM << PSTR("Overvoltage!\r\n"); }
        if(error & STAT_UV)  {
            cout << PGM << PSTR("Undervoltage!\r\n");
            shd--;
            if (shd == 0) bq.shutdown();
        }
        if(error & STAT_SCD) { cout << PGM << PSTR("Short Circuit Protection!\r\n"); }
        if(error & STAT_OCD) { cout << PGM << PSTR("Overcurrent Charge Protection!\r\n"); }
        if (bq769x_stats.batCycles_ != m_BatCycles_prev) {
            m_BatCycles_prev    = bq769x_stats.batCycles_;
            stats_save();
        }
        if (bq769x_stats.chargedTimes_ != m_ChargedTimes_prev) {
            m_ChargedTimes_prev = bq769x_stats.chargedTimes_;
            stats_save();
        }
        uint16_t bigDelta = bq.getMaxCellVoltage() - bq.getMinCellVoltage();
        if(bigDelta > 100) cout << PGM << PSTR("Difference too big!\r\n");
        if(m_oldMillis > now)
            m_millisOverflows++;
        m_oldMillis = now;
        job = 0;
    }
    return result;
}

void Console::debug_print() {
    uint32_t uptime = m_millisOverflows * (0xffffffffLL / 1000UL);
    uptime += mcu::Timer::millis() / 1000;

    cout
        << PGM << PSTR("BMS uptime: ") << uptime
        << PGM << PSTR(" BAT Temp: ") // TODO macro for x20 x30 Ic
        << bq.getTemperatureDegC(0) << ' '
        << bq.getTemperatureDegC(1) << ' '
        << bq.getTemperatureDegC(2)
        << EOL;

    cout 
        << PGM << PSTR("BAT Voltage: ")     << bq769x_data.batVoltage_
        << PGM << PSTR(" mV (")             << bq769x_data.batVoltage_raw_
        << PGM << PSTR(" raw), current: ")  << bq769x_data.batCurrent_
        << PGM << PSTR(" mA (")             << bq769x_data.batCurrent_raw_
        << PGM << PSTR(" raw)\r\n")
        << PGM << PSTR("SOC: ") << bq.getSOC()
        << PGM << PSTR(" Balancing status: ") << bq769x_data.balancingStatus_
        << PGM << PSTR("\r\nCell voltages:\r\n");
    
    for(uint8_t x = 0; x < MAX_NUMBER_OF_CELLS; x++) {
        uint8_t y = bq769x_stats.cellIdMap_[x];
        cout << bq769x_stats.cellVoltages_[y] << PGM << PSTR(" mV (") << bq769x_data.cellVoltages_raw_[y] << " raw)\t";
        if ((x+1) % 3 == 0) cout << EOL;
    }
    
    cout
        << PGM << PSTR("\r\nCell mV: Min: ")       << bq.getMinCellVoltage()
        << PGM << PSTR(" | Avg: ")                 << bq.getAvgCellVoltage()
        << PGM << PSTR(" | Max: ")                 << bq.getMaxCellVoltage()
        << PGM << PSTR(" | Delta: ")               << bq.getMaxCellVoltage() - bq.getMinCellVoltage()
        << PGM << PSTR("\r\nXREADY errors: ")      << bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_XREADY]
        << PGM << PSTR("\r\n ALERT errors: ")      << bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_ALERT]
        << PGM << PSTR("\r\n   UVP errors: ")      << bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_UVP]
        << PGM << PSTR("\r\n   OVP errors: ")      << bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_OVP]
        << PGM << PSTR("\r\n   SCD errors: ")      << bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_SCD]
        << PGM << PSTR("\r\n   OCD errors: ")      << bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_OCD]
        << PGM << PSTR("\r\nDISCHG TEMP errors: ") << bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_USER_DISCHG_TEMP]
        << PGM << PSTR("\r\n   CHG TEMP errors: ") << bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_USER_CHG_TEMP]
        << PGM << PSTR("\r\n    CHG OCD errors: ") << bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_USER_CHG_OCD]
        << EOL;
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
        ser.write('.');
        mcu::Watchdog::reset();
    }
    cout << EOL;
}

}
