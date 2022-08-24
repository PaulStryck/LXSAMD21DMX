/**************************************************************************/
/*!
    @file     LXSAMD21DMX.cpp
    @author   Claude Heintz
    @license  BSD (see SAMD21DMX.h or http://lx.claudeheintzdesign.com/opensource.html)
    @copyright 2016 by Claude Heintz

    DMX Driver for Arduino MKR1000

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/

#include "LXSAMD21DMX.h"
#include <compiler.h>

// #include <hal_gpio.h>
// #include <inttypes.h>
// #include <stdlib.h>
#include <cstring>
#include <hpl_usart.h>
#include <peripheral_clk_config.h>

//**************************************************************************************

// TODO: make basic usart config a parameter to class



// ***************** define registers, flags and interrupts  ****************

    //***** baud rate defines
    #define DMX_DATA_BAUD   250000
    #define DMX_BREAK_BAUD  250000
    //99900

    //***** states indicate current position in DMX stream
    #define DMX_STATE_BREAK 0
    #define DMX_STATE_START 1
    #define DMX_STATE_DATA  2
    #define DMX_STATE_IDLE  3

    //***** status is if interrupts are enabled and IO is active
    #define ISR_DISABLED        0
    #define ISR_OUTPUT_ENABLED  1
    #define ISR_INPUT_ENABLED   2

// **************************** global data (can be accessed in ISR)  ***************


uint8_t*  _shared_dmx_data;
uint8_t   _shared_dmx_state;
uint16_t  _shared_dmx_slot;
uint16_t  _shared_max_slots = DMX_MIN_SLOTS;
LXRecvCallback _shared_receive_callback = NULL;


static void init_dmx_uart(void *const hw, dmx_usart_configuration conf);


// USART_0_CLOCK_init() before this
static void init_dmx_uart(void *const hw, dmx_usart_configuration conf) {
    ASSERT(hw);

    // usart_init
    if (!hri_sercomusart_is_syncing(hw, SERCOM_USART_SYNCBUSY_SWRST)) {
        uint32_t mode = conf.ctrl_a & SERCOM_USART_CTRLA_MODE_Msk;
        if (hri_sercomusart_get_CTRLA_reg(hw, SERCOM_USART_CTRLA_ENABLE)) {
            hri_sercomusart_clear_CTRLA_ENABLE_bit(hw);
            hri_sercomusart_wait_for_sync(hw, SERCOM_USART_SYNCBUSY_ENABLE);
        }
        hri_sercomusart_write_CTRLA_reg(hw, SERCOM_USART_CTRLA_SWRST | mode);
    }
    hri_sercomusart_wait_for_sync(hw, SERCOM_USART_SYNCBUSY_SWRST);

    hri_sercomusart_write_CTRLA_reg(hw, conf.ctrl_a);
    hri_sercomusart_write_CTRLB_reg(hw, conf.ctrl_b);
    if ((conf.ctrl_a & SERCOM_USART_CTRLA_SAMPR(0x1)) || (conf.ctrl_a & SERCOM_USART_CTRLA_SAMPR(0x3))) {
        ((Sercom *)hw)->USART.BAUD.FRAC.BAUD = conf.baud;
        ((Sercom *)hw)->USART.BAUD.FRAC.FP   = conf.fractional;
    } else {
        hri_sercomusart_write_BAUD_reg(hw, conf.baud);
    }

    hri_sercomusart_write_RXPL_reg(hw, conf.rxpl);
    hri_sercomusart_write_DBGCTRL_reg(hw, conf.debug_ctrl);

    // Async NVIC init
    IRQn_Type IdNvic = SERCOM0_IRQn; // TODO!
    NVIC_DisableIRQ(IdNvic);
    NVIC_ClearPendingIRQ(IdNvic);
    NVIC_EnableIRQ(IdNvic);
    // NVIC_SetPriority (IdNvic, SERCOM_NVIC_PRIORITY);  /* set Priority */

    // Set CTRLA register
    // Set INTENSET register
    hri_sercomusart_write_INTEN_RXC_bit(hw, true);
    hri_sercomusart_write_INTEN_ERROR_bit(hw, true);

    // Mode is Internal clock
    // compute baud rate and set

    // enable
    hri_sercomusart_set_CTRLA_ENABLE_bit(hw);
    // hri_sercomusart_clear_CTRLA_ENABLE_bit(hw);
    hri_sercomusart_wait_for_sync(hw, SERCOM_USART_SYNCBUSY_ENABLE);
}


//************************************************************************************
// ************************  LXSAMD21DMXOutput member functions  ********************

LXSAMD21DMX::LXSAMD21DMX ( Sercom *const hw, dmx_usart_configuration conf )
    : _hw(hw), _conf(conf) {
    _direction_pin = DIRECTION_PIN_NOT_USED;    //optional
    _shared_max_slots = DMX_MAX_SLOTS;
    _interrupt_status = ISR_DISABLED;

    //zero buffer including _dmxData[0] which is start code
    memset(_dmxData, 0, DMX_MAX_SLOTS+1);
}



LXSAMD21DMX::~LXSAMD21DMX ( void ) {
    stop();
    _shared_dmx_data = NULL;
    _shared_receive_callback = NULL;
}


void LXSAMD21DMX::setBaudRate(uint32_t baudrate) {
    ASSERT(_hw);

    hri_sercomusart_clear_CTRLA_ENABLE_bit(_hw);
    hri_sercomusart_wait_for_sync(_hw, SERCOM_USART_SYNCBUSY_ENABLE);

    // TODO: get actual sample rate
    uint16_t sampleRateValue = 16;

    // see SERCOM.initUART
    // TODO: get actual clock speed
    uint32_t baudTimes8 = (4000000 * 8) / (sampleRateValue * baudrate);

    _hw->USART.BAUD.FRAC.FP   = (baudTimes8 % 8);
    _hw->USART.BAUD.FRAC.BAUD = (baudTimes8 / 8);

    hri_sercomusart_set_CTRLA_ENABLE_bit(_hw);
    hri_sercomusart_wait_for_sync(_hw, SERCOM_USART_SYNCBUSY_ENABLE);
}

void LXSAMD21DMX::startOutput ( void ) {
    if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
        gpio_set_pin_level(_direction_pin, true);
    }

    if ( _interrupt_status == ISR_INPUT_ENABLED ) {
        stop();
    }

    if ( _interrupt_status == ISR_DISABLED ) {  //prevent messing up sequence if already started...
                                                //SerialDMX.begin(DMX_BREAK_BAUD, (uint8_t)SERIAL_8N2);
        init_dmx_uart(_hw, _conf);
        setBaudRate(DMX_BREAK_BAUD);

        // Assign pin mux to SERCOM functionality (must come after SerialDMX.begin)
        gpio_set_pin_function(PIN_DMX_RX, MUX_DMX_RX);
        gpio_set_pin_function(PIN_DMX_TX, MUX_DMX_TX);

        _interrupt_status = ISR_OUTPUT_ENABLED;
        _shared_dmx_data = dmxData();
        _shared_dmx_slot = 0;
        _shared_dmx_state = DMX_STATE_START;

        // _hw->USART.INTENSET.reg =  SERCOM_USART_INTENSET_TXC | SERCOM_USART_INTENSET_ERROR;
        // _hw->USART.DATA.reg = 0;
        hri_sercomusart_write_INTEN_RXC_bit(_hw, false);
        hri_sercomusart_write_INTEN_TXC_bit(_hw, true);
        hri_sercomusart_write_INTEN_ERROR_bit(_hw, true);

        hri_sercomusart_write_DATA_reg(_hw, 1);

    }
}

void LXSAMD21DMX::startInput ( void ) {
    if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
        gpio_set_pin_level(_direction_pin, false);
    }
    if ( _interrupt_status == ISR_OUTPUT_ENABLED ) {
        stop();
    }
    if ( _interrupt_status == ISR_DISABLED ) {  //prevent messing up sequence if already started...
        //SerialDMX.begin(DMX_DATA_BAUD, (uint8_t)SERIAL_8N2);
        init_dmx_uart(_hw, _conf);
        setBaudRate(DMX_DATA_BAUD);

        // Assign pin mux to SERCOM functionality (must come after SerialDMX.begin)
        gpio_set_pin_function(PIN_DMX_RX, MUX_DMX_RX);
        gpio_set_pin_function(PIN_DMX_TX, MUX_DMX_TX);

        _shared_dmx_data = dmxData();
        _shared_dmx_slot = 0;
        _shared_dmx_state = DMX_STATE_IDLE;

        _interrupt_status = ISR_INPUT_ENABLED;
    }
}

void LXSAMD21DMX::stop ( void ) {
   // SerialDMX.end();
    _interrupt_status = ISR_DISABLED;
}

void LXSAMD21DMX::setDirectionPin( uint8_t pin ) {
    _direction_pin = pin;
    gpio_set_pin_direction(_direction_pin, GPIO_DIRECTION_OUT);
}

void LXSAMD21DMX::setMaxSlots (int slots) {
    if ( slots > DMX_MIN_SLOTS ) {
        _shared_max_slots = slots;
    } else {
        _shared_max_slots = DMX_MIN_SLOTS;
    }
}

uint8_t LXSAMD21DMX::getSlot (int slot) {
    return _dmxData[slot];
}

void LXSAMD21DMX::setSlot (int slot, uint8_t value) {
    _dmxData[slot] = value;
}

uint8_t* LXSAMD21DMX::dmxData(void) {
    return &_dmxData[0];
}

void LXSAMD21DMX::setDataReceivedCallback(LXRecvCallback callback) {
    _shared_receive_callback = callback;
}


void LXSAMD21DMX::IrqHandler(void) {

  if ( _interrupt_status == ISR_OUTPUT_ENABLED ) {

    if ( _hw->USART.INTFLAG.bit.TXC ) {
      if ( _hw->USART.INTFLAG.bit.DRE ) {

        if ( _shared_dmx_state == DMX_STATE_DATA ) {
          hri_sercomusart_write_DATA_reg(_hw,_shared_dmx_data[_shared_dmx_slot++]);    //send next slot;
          if ( _shared_dmx_slot > _shared_max_slots ) {
            _shared_dmx_state = DMX_STATE_BREAK;
          }
        } else if ( _shared_dmx_state == DMX_STATE_BREAK ) {
          setBaudRate(DMX_BREAK_BAUD);
          _shared_dmx_state = DMX_STATE_START;
          _shared_dmx_slot = 0;
          hri_sercomusart_write_DATA_reg(_hw, 0); // break
        } else if ( _shared_dmx_state == DMX_STATE_START ) {
          setBaudRate(DMX_DATA_BAUD);
          _shared_dmx_state = DMX_STATE_DATA;
          hri_sercomusart_write_DATA_reg(_hw, _shared_dmx_data[_shared_dmx_slot++]);
        }

      }  //DRE
    }    //TXC

  } else if ( _interrupt_status == ISR_INPUT_ENABLED ) {

        if ( _hw->USART.INTFLAG.bit.ERROR ) {
           _hw->USART.INTFLAG.bit.ERROR = 1;     //acknowledge error, clear interrupt

            if ( _hw->USART.STATUS.bit.FERR ) {  //framing error happens when break is sent
                _shared_dmx_state = DMX_STATE_BREAK;
                if ( _shared_dmx_slot > 0 ) {
                    if ( _shared_receive_callback != NULL ) {
                        _shared_receive_callback(_shared_dmx_slot);
                    }
                }
                _shared_dmx_slot = 0;
              return;
            }
            // other error flags?
            //return;?
        }   //ERR

        if ( _hw->USART.INTFLAG.bit.RXC ) {
        uint8_t incoming_byte = _hw->USART.DATA.reg;             // read buffer to clear interrupt flag
            switch ( _shared_dmx_state ) {
                case DMX_STATE_BREAK:
                    if ( incoming_byte == 0 ) {                                 // start code == zero (DMX)
                        _shared_dmx_data[_shared_dmx_slot] = incoming_byte;
                        _shared_dmx_state = DMX_STATE_DATA;
                    } else {
                        _shared_dmx_state = DMX_STATE_IDLE;
                    }
                    break;

                case DMX_STATE_DATA:
                    _shared_dmx_data[_shared_dmx_slot++] = incoming_byte;   // increments BEFORE assignment
                    if ( _shared_dmx_slot > DMX_MAX_SLOTS ) {
                        _shared_dmx_state = DMX_STATE_IDLE;                     // go to idle, wait for next break
                    }
                    break;
            }
        } // RXC

  }   // input enabled

}         //--IrqHandler(void)
