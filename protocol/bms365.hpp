#pragma once

#include <stdint.h>
#include "GyverLibs/GyverCore_uart.h"

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

#define RECV_BUFFER_SIZE 0x88
#define PACKET_SIZE      0x80

class Protocol_365 {
    uint16_t recv_buffer_head = 0;
    uint16_t recv_buffer_tail = 0;
    uint8_t  recv_buffer[RECV_BUFFER_SIZE];
    uint8_t  recv_buffer_full = 0;
    
    uint16_t packet[PACKET_SIZE];
    uint16_t m_Chksum;
    uint16_t m_Await;
    int16_t  m_Count;
    
    void two8to16(uint8_t rega, uint8_t a, uint8_t b) {
        packet[rega] = ((uint16_t)a | ((uint16_t)b) << 8);
    }
    
    uint8_t get_buffer(uint16_t index){
        uint16_t idx = (recv_buffer_tail + index);
        if (idx >= RECV_BUFFER_SIZE) idx -= RECV_BUFFER_SIZE;
        return recv_buffer[(uint8_t)(idx)];
    }
    
    uint8_t flush_buffer(uint16_t items){
        recv_buffer_tail += items;
        if (recv_buffer_tail >= RECV_BUFFER_SIZE) recv_buffer_tail -= RECV_BUFFER_SIZE;
        recv_buffer_full = 0;
        return recv_buffer_tail;
    }
    
    uint16_t putc_byte(uint8_t c) {
        uart.write(c);
        return (uint16_t)c;
    }
    
    uint16_t putc_16bmsb(uint16_t cc) {
        return putc_byte((uint8_t)((cc >> 8) & 0xff));
    }
    
    uint16_t putc_16blsb(uint16_t cc) {
        return putc_byte((uint8_t)((cc) & 0xff));
    }
    
    uint16_t put_buf(uint8_t idx, uint16_t sz){
        uint8_t parity = sz & 0x01;
        uint8_t sc = sz >> 1;
        uint16_t s = 0;
        for (uint16_t i = idx; i < (idx + sc); i++) {
            s += putc_16blsb(packet[i & 0x7f]);
            s += putc_16bmsb(packet[i & 0x7f]);
        }
        if (parity != 0) {
            s += putc_16blsb(packet[(uint8_t)((idx + sc + 1) & 0x7f)]);
        }
        return s;
    }
    
    void handle(const uint8_t cmd) {
        uint16_t summ = 0;
        switch (cmd) {
            case 0x01:
                putc_byte(0x55);
                putc_byte(0xaa);
                summ += putc_byte(get_buffer(6) + 2);
                summ += putc_byte(0x25); // BMS --> M
                summ += putc_byte(0x01);
                summ += putc_byte(get_buffer(5));
                summ += put_buf(get_buffer(5), get_buffer(6));
                summ ^= 0xffff;
                putc_16blsb(summ);
                putc_16bmsb(summ);
                break;
            default: { /* unhandled */ }
            break;
        }
    }
public:
    
    Protocol_365(uint16_t _ver = 0x115, uint16_t _cap = 0x19C8, uint16_t _date = 0x2A2A) {
        for (uint8_t x = 0; x < PACKET_SIZE; ++x) { packet[x] = 0; }
        two8to16(Reg_Serial + 0, '3', 'J');
        two8to16(Reg_Serial + 1, 'U', 'M');
        two8to16(Reg_Serial + 2, 'A', 'N');
        two8to16(Reg_Serial + 3, 'O', 'T');
        two8to16(Reg_Serial + 4, '3', '6');
        two8to16(Reg_Serial + 5, '5', 'B');
        two8to16(Reg_Serial + 6, 'M', 'S');
        packet[Reg_Version]  = _ver;
        packet[Reg_Capacity] = _cap;
        packet[Reg_Date]     = _date;
        set_Status_bit(b_Ready, true);
        set_Voltage_Pack(4100);
        set_Temp(26, 27);
        set_Health(99);
        set_Cell_Voltage(0, 3100);
        set_Cell_Voltage(1, 3100);
        set_Cell_Voltage(2, 3100);
        set_Cell_Voltage(3, 3100);
        set_Cell_Voltage(4, 3100);
        set_Cell_Voltage(5, 3100);
        set_Cell_Voltage(6, 3100);
        set_Cell_Voltage(7, 3100);
        set_Cell_Voltage(8, 3100);
        set_Cell_Voltage(9, 3100);
    }

    void set_Status_bit(M36StatusBits bit, bool state) {
        if (state) {
            packet[Reg_Status] |= (1 << bit); 
        } else {
            packet[Reg_Status] &= ~(1 << bit);
        }
    }
    
    void set_Health(uint16_t Health) {
        packet[Reg_PackHealth] = Health;
    }
    
    void set_Voltage_Pack(uint16_t mVdiv10) {
        packet[Reg_Voltage]  = mVdiv10;
    }
    
    void set_Cell_Voltage(uint8_t cell, uint16_t mV) {
        cell += Reg_Cell_0;
        if (cell <= Reg_Cell_9) { packet[cell] = mV; }
    }
    
    void set_Current(int16_t Ax100) {
        packet[Reg_Current] = Ax100;
    }
    
    void set_Percentage(uint8_t p) {
        if (p > 100) { p = 100; }
        packet[Reg_Percentage] = p;
        packet[Reg_CapLeft] = (uint16_t)(((uint32_t)p * packet[Reg_Capacity])/100);
    }
    
    void set_Temp(int8_t t1, int8_t t2) {
        t1 += 20;
        t2 += 20;
        uint8_t rt1 = 0, rt2 = 0;
        if (t1 >= 0) rt1 = t1;
        if (t2 >= 0) rt2 = t2;
        two8to16(Reg_Temp, rt1, rt2);
    }
    
    void put_char(uint8_t c) {
        if (recv_buffer_full == 0) {
            recv_buffer[recv_buffer_head] = c;
            recv_buffer_head++;
            if (recv_buffer_head >= RECV_BUFFER_SIZE) recv_buffer_head -= RECV_BUFFER_SIZE;
            if (recv_buffer_head == recv_buffer_tail) recv_buffer_full = 1;
        }
    }
    
    void update() {
        m_Count = recv_buffer_head - recv_buffer_tail;
        if (m_Count < 0) m_Count += RECV_BUFFER_SIZE;
        if (m_Count == 0 && recv_buffer_full == 1) m_Count = RECV_BUFFER_SIZE;
        
        m_Await = 2;
        if (m_Count >= (m_Await + 6)) {
            if ((get_buffer(0) == 0x55) && (get_buffer(1) == 0xAA)) { // magic found
                if ((get_buffer(2) + 6) < RECV_BUFFER_SIZE) { m_Await = get_buffer(2); } else { m_Await=2; }
                if ((m_Await + 6) <= m_Count) {
                    // calc crc
                    m_Chksum = 0;
                    for (int16_t x = m_Await + 3; x > 1; x--) { m_Chksum += get_buffer(x); }
                    m_Chksum = m_Chksum^0xffff;
                    
                    if (get_buffer(m_Await + 4) == (m_Chksum & 0xff) &&
                        get_buffer(m_Await + 5) == ((m_Chksum >> 8) & 0xff) &&
                        get_buffer(3) == 0x22) // M --> BMS 0x22
                    {
                        handle(get_buffer(4));
                        flush_buffer(m_Await + 6);
                    } else {
                        // bad crc, remove chars
                        flush_buffer(m_Count);
                    }
                }
            } else {
                flush_buffer(1);
            }
        }
    }
    
};

