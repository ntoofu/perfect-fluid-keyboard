#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>

#define _KEY_LED_PORT 3
#define _KEY_SWITCH_PORT 6
#define _PORT_CTRL_MACRO_JOIN2(pin) PIN ## pin ## CTRL
// Work-around for the complicated behavior of token-pasting operator
#define _PORT_CTRL_MACRO_JOIN(pin) _PORT_CTRL_MACRO_JOIN2(pin)
#define _KEY_SWITCH_PORT_CTRL _PORT_CTRL_MACRO_JOIN(_KEY_SWITCH_PORT)
#define _MUXER_INH 7
#define _TWI_ADDR (0x01)
#define _PWM_PERIOD (0x0341)
#define _TIMEOUT_THRESHOLD 2

static volatile uint8_t twi_timeout_cnt = 0;
static volatile uint8_t twi_rx_data = 0;
static volatile uint8_t cnt = 0;

void led_message(uint8_t val){
    for(int i=7; i>=0; --i) {
        led_level(0x00);
        _delay_ms(100);
        led_level(0x80);
        _delay_ms(50);
        if (val & (1 << i))
            _delay_ms(200);
    }
    led_level(0x00);
    _delay_ms(200);
}

void led_level(uint8_t level) {
    uint16_t cmp = 0;
    for (uint8_t i=8; i>0; --i) {
        if (level & 0x01) {
            cmp += _PWM_PERIOD >> i;
        }
        level >>= 1;
    }
    TCA0.SINGLE.CMP0BUF = cmp;
}

void i2c_init_slave(uint8_t addr) {
    TWI0.CTRLA = TWI_SDASETUP_8CYC_gc | TWI_SDAHOLD_50NS_gc & ~TWI_FMPEN_bm;
    TWI0.SADDR = (addr & 0x7f) << 1 | 0 << TWI_ADDREN_bp;
    TWI0.SCTRLA = TWI_DIEN_bm | TWI_PIEN_bm | TWI_APIEN_bm & ~TWI_PMEN_bm & ~TWI_SMEN_bm | TWI_ENABLE_bm;
}

void timer_init(void) {
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_CLKSEL_DIV4_gc | TCA_SINGLE_OVF_bm;
    TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm
                        | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
    TCA0.SINGLE.EVCTRL &= ~(TCA_SINGLE_CNTEI_bm);
    TCA0.SINGLE.PERBUF = _PWM_PERIOD;
    TCA0.SINGLE.CMP0BUF = 0x0000;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV4_gc
                        | TCA_SINGLE_ENABLE_bm;
}

void initialization(void) {
    PORTA.DIR |= _BV(_KEY_LED_PORT);
    PORTA.DIR &= ~_BV(_KEY_SWITCH_PORT);
    PORTA.DIR |= _BV(_MUXER_INH);
    PORTA.OUT = 0;
    PORTA._KEY_SWITCH_PORT_CTRL |= PORT_PULLUPEN_bm;
    timer_init();
    i2c_init_slave(_TWI_ADDR);
    led_level(0x80);
    _delay_ms(100);
    led_level(0x00);
    sei();
}

/*
    Both read and write operation is assumed to handle only one byte
*/
ISR(TWI0_TWIS_vect) {
    // Invalid State
    if ((TWI0.SSTATUS & TWI_APIF_bm) && (TWI0.SSTATUS & TWI_DIF_bm)) {
        led_message(1);
        return;
    }

    // Collision
    if (TWI0.SSTATUS & TWI_COLL_bm) {
        led_message(2);
        return;
    }

    // Bus Error
    if (TWI0.SSTATUS & TWI_BUSERR_bm) {
        led_message(3);
        return;
    }

    // Valid Address
    if ((TWI0.SSTATUS & TWI_APIF_bm) && (TWI0.SSTATUS & TWI_AP_bm)) {
        TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc; // send ACK

        if (TWI0.SSTATUS & TWI_DIR_bm) {    // Master Read
            // _delay_us(5);    // TWI0.SDATA write needs minimum 5 us delay at 10 MHz clock
            twi_timeout_cnt = 0;
            while (!(TWI0.SSTATUS & TWI_CLKHOLD_bm)) {
                if (twi_timeout_cnt > _TIMEOUT_THRESHOLD) return;
            }
            TWI0.SDATA = ( PORTA.IN & _BV(_KEY_SWITCH_PORT) ) ? 0 : 1; // send key switch status (1: pressed)
            TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;
        }
        return;
    }

    // Slave operation (read or write) succeeded and continue
    if (TWI0.SSTATUS & TWI_DIF_bm) {
        if (TWI0.SSTATUS & TWI_DIR_bm) {    // Master Read
            if (!(TWI0.SSTATUS & TWI_RXACK_bm)) {   // ACK from master
                // _delay_us(5);    // TWI0.SDATA write needs minimum 5 us delay at 10 MHz clock
                twi_timeout_cnt = 0;
                while (!(TWI0.SSTATUS & TWI_CLKHOLD_bm)) {
                    if (twi_timeout_cnt > _TIMEOUT_THRESHOLD) return;
                }
                TWI0.SDATA = ( PORTA.IN & _BV(_KEY_SWITCH_PORT) ) ? 0 : 1; // send key switch status (1: pressed)
                TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;
            } else {    // NACK from master
                TWI0.SSTATUS |= (TWI_DIF_bm | TWI_APIF_bm); // Reset module
                TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
            }
        } else {    // Master Write
            twi_rx_data = TWI0.SDATA;
            led_level(twi_rx_data);
            TWI0.SCTRLB = TWI_ACKACT_NACK_gc | TWI_SCMD_RESPONSE_gc; // send NACK
        }
        return;
    }

    // Stop
    if ((TWI0.SSTATUS & TWI_APIF_bm) && (!(TWI0.SSTATUS & TWI_AP_bm))) {
        TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
        return;
    }
}

ISR(TCA0_OVF_vect) {
    twi_timeout_cnt++;
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}

int main(void){
    initialization();
    for(;;){}
}
