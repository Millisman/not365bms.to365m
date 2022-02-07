/* Shell console for battery management based bq769x Ic
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

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include "mcu/watchdog.h"
#include "mcu/pin.h"

#include "protocol/not365to365.h"
#define PIN_LED_SCK MAKEPIN(B, 5, OUT)

#include "GyverLibs/GyverPower.h"
#include "GyverLibs/uptime2.h"

void activate_INT0();
void activate_Rx_change_int();
void deactivate_Rx_change_int();

mcu::Pin led(PIN_LED_SCK);  
protocol::Console proto; // Console load conf
static volatile bool isr_alert;
uint32_t moment;
static volatile uint32_t moment_2000;
static volatile uint32_t last_Activity;
// ISR(INT1_vect) // ISR(INT2_vect)

ISR(INT0_vect)   { isr_alert = true; }           // ALERT
ISR(PCINT2_vect) { last_Activity = millis2(); }  // USART

void task_alert() {
    led = 1;
    power.hardwareEnable( PWR_UART0 | PWR_TIMER2 | PWR_I2C );
    proto.update(true);
    led = 0;
}

void task_standby() {
    led = 1;
    power.hardwareEnable( PWR_UART0 | PWR_TIMER2 | PWR_I2C );
    proto.update(false);
    led = 0;
}

void task_running() {
    led = 1;
    power.hardwareEnable( PWR_UART0 | PWR_TIMER2 | PWR_I2C );
    proto.update(false);
    led = 0;
}


int main(void) {
    moment_2000 = 0;
    last_Activity = 0;
    isr_alert = false;
    MCUSR = 0;
    mcu::Watchdog::disable();
    power.hardwareDisable(PWR_ALL);
    power.hardwareEnable( PWR_UART0 | PWR_TIMER2 | PWR_I2C );
    uptime2Init();
    activate_INT0();
    activate_Rx_change_int();
    power.setSleepMode(POWERSAVE_SLEEP);
    mcu::Watchdog::enable(WDTO_4S);
    proto.begin();
    
    for(;;) {
        
        moment = millis2();

        if (isr_alert) { task_alert(); isr_alert = false; }
        
        if ((moment - moment_2000 >= 2000)) {
            moment_2000 += 2000;
            task_standby();
        }
        
        if((uint32_t)(moment - last_Activity) >= 15000) { // 15 sec to idle
            power.hardwareDisable(PWR_UART0 | PWR_I2C);
            _delay_ms(5);
            power.sleep(POWERSAVE_SLEEP);
        } else {
            task_running();
        }
        
        mcu::Watchdog::reset();
    }
    
    return 0;
}

#define CHANGE  1
#define FALLING 2
#define RISING  3

void activate_INT0() {
    EICRA = (1 << ISC01) | (1 << ISC00) | (RISING << ISC00);
    EIMSK = (1 << INT0);
    //----------------------------------------------------------------- INT1_vect
    //EICRA = (1 << ISC11) | (1 << ISC10) | (RISING << ISC10);
    //EIMSK = (1 << INT1);
    //----------------------------------------------------------------- INT2_vect
    //EICRA = (1 << ISC21) | (1 << ISC20) | (RISING << ISC20);
    //EIMSK = (1 << INT2);
}

void activate_Rx_change_int() {
    // Enable pin change interrupt on the PCINT16 pin using Pin Change Mask Register 2 (PCMSK2)
    PCMSK2 |= (1 << PCINT16); // PD0 RXD
    // Enable pin change interrupt 2 using the Pin Change Interrrupt Control Register (PCICR)
    PCICR |= (1 << PCIE2);    
}

void deactivate_Rx_change_int() {
    // Disable pin change interrupt 2 using the Pin Change Interrrupt Control Register (PCICR)
    PCICR &= ~(1 << PCIE2);
    // Enable pin change interrupt on the PCINT16 pin using Pin Change Mask Register 2 (PCMSK2)
    PCMSK2 &= ~(1 << PCINT16); // PD0 RXD
}
