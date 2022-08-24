/* LXSAMD21DMX.h
   Copyright 2016 by Claude Heintz Design
   All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of LXSAMD21DMX nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
-----------------------------------------------------------------------------------

   The LXSAMD21DMX library supports output and input of DMX using
   sercom4 of a MKR1000 microcontroller in UART mode. (pins 7 & 8)

   This is the circuit for a simple unisolated DMX Shield
   that could be used with LXSAMD21DMX.  It uses a line driver IC
   to convert the output from the SAM D-21 to DMX:

 MKR1000 Pin
 |                         SN 75176 A or MAX 481CPA
 V                            _______________
       |                      | 1      Vcc 8 |------(+5v)
RX (5) |----------------------|              |                 DMX Output
       |                 +----| 2        B 7 |---------------- Pin 2
       |                 |    |              |
   (3) |----------------------| 3 DE     A 6 |---------------- Pin 3
       |                      |              |
TX (4) |----------------------| 4 DI   Gnd 5 |---+------------ Pin 1
       |                                         |
       |                                       (GND)

       Data Enable (DE) and Inverted Read Enable (!RE) can be wired to +5v for output or Gnd for input
       if direction switching is not needed.
*/

#ifndef LXSAM21_DMX_H
#define LXSAM21_DMX_H

#include <inttypes.h>
#include <hpl_usart.h>
#include <atmel_start_pins.h>

#define DMX_MIN_SLOTS 24
#define DMX_MAX_SLOTS 96

#define DIRECTION_PIN_NOT_USED 255

#define DMX_USART_CONF_SAMPLE                                           \
    {                                                                   \
        SERCOM_USART_CTRLA_MODE(UART_INT_CLOCK)                         \
            | (0 << SERCOM_USART_CTRLA_RUNSTDBY_Pos)                    \
            | (0 << SERCOM_USART_CTRLA_IBON_Pos)                        \
            | SERCOM_USART_CTRLA_SAMPR(SAMPLE_RATE_x16)                 \
            | SERCOM_USART_CTRLA_TXPO(1)                                \
            | SERCOM_USART_CTRLA_RXPO(1)                                \
            | SERCOM_USART_CTRLA_SAMPA(0x0)                             \
            | SERCOM_USART_CTRLA_FORM(0)                                \
            | (0 << SERCOM_USART_CTRLA_CMODE_Pos)                       \
            | (0 << SERCOM_USART_CTRLA_CPOL_Pos)                        \
            | (1 << SERCOM_USART_CTRLA_DORD_Pos),                       \
        SERCOM_USART_CTRLB_CHSIZE(0x0)                                  \
            | (USART_STOP_BITS_ONE << SERCOM_USART_CTRLB_SBMODE_Pos)    \
            | (0 << SERCOM_USART_CTRLB_COLDEN_Pos)                      \
            | (0 << SERCOM_USART_CTRLB_SFDE_Pos)                        \
            | (0 << SERCOM_USART_CTRLB_ENC_Pos)                         \
            | (USART_PARITY_NONE << SERCOM_USART_CTRLB_PMODE_Pos)       \
            | (1 << SERCOM_USART_CTRLB_TXEN_Pos)                        \
            | (1 << SERCOM_USART_CTRLB_RXEN_Pos),                       \
        0,                                                              \
        0,                                                              \
        0,                                                              \
        0,                                                              \
    }


enum usart_clock_mode {
    UART_EXT_CLOCK = 0,
    UART_INT_CLOCK = 0x1u
};

enum usart_sample_rate {
    SAMPLE_RATE_x16 = 0x1,  //Fractional
    SAMPLE_RATE_x8 = 0x3,   //Fractional
};

struct dmx_usart_configuration {
    hri_sercomusart_ctrla_reg_t   ctrl_a;
    hri_sercomusart_ctrlb_reg_t   ctrl_b;
    hri_sercomusart_baud_reg_t    baud;
    uint8_t                       fractional;
    hri_sercomusart_rxpl_reg_t    rxpl;
    hri_sercomusart_dbgctrl_reg_t debug_ctrl;
};


typedef void (*LXRecvCallback)(int);

/*!
@class LXSAMD21DMX
@abstract
   LXSAMD21DMX is a driver for sending or receiving DMX using one of a SAM D-21's five serial peripheral interfaces (SERCOMs).

   LXSAMD21DMX output mode continuously sends DMX once its interrupts have been enabled using startOutput().
   Use setSlot() to set the level value for a particular DMX dimmer/address/channel.

   LXSAMD21DMX input mode continuously receives DMX once its interrupts have been enabled using startInput()
   Use getSlot() to read the level value for a particular DMX dimmer/address/channel.

   LXSAMD21DMX is used with a single instance called SAMD21DMX  .
*/

class LXSAMD21DMX  {

  public:

    LXSAMD21DMX  ( Sercom *const hw, dmx_usart_configuration conf );
   ~LXSAMD21DMX  ( void );

   /*!
    * @brief starts interrupt that continuously sends DMX output
    * @discussion Sets up baud rate, bits and parity,
    *             sets globals accessed in ISR,
    *             enables transmission (TE) and tx interrupts (TIE/TCIE).
   */
   void startOutput( void );

   /*!
    * @brief starts interrupt that continuously reads DMX data
    * @discussion sets up baud rate, bits and parity,
    *             sets globals accessed in ISR,
    *             enables receive (RE) and rx interrupt (RIE)
   */
   void startInput( void );

   /*!
    * @brief disables tx, rx and interrupts.
   */
    void stop( void );

    /*!
     * @brief optional utility sets the pin used to control driver chip's
     *        DE (data enable) line, HIGH for output, LOW for input.
    * @param pin to be automatically set for input/output direction
    */
   void setDirectionPin( uint8_t pin );

    /*!
     * @brief Sets the number of slots (aka addresses or channels) sent per DMX frame.
     * @discussion defaults to 512 or DMX_MAX_SLOTS and should be no less DMX_MIN_SLOTS slots.
     *             The DMX standard specifies min break to break time no less than 1024 usecs.
     *             At 44 usecs per slot ~= 24
     * @param slot the highest slot number (~24 to 512)
    */
    void setMaxSlots (int slot);

    /*!
    * @brief reads the value of a slot/address/channel
    * @discussion NOTE: Data is not double buffered.
    *                   So a complete single frame is not guaranteed.
    *                   The ISR continuously reads the next frame into the buffer
    * @return level (0-255)
   */
   uint8_t getSlot (int slot);

    /*!
     * @brief Sets the output value of a slot
     * @param slot number of the slot/address/channel (1-512)
     * @param value level (0-255)
    */
   void setSlot (int slot, uint8_t value);

   /*!
    * @brief provides direct access to data array
    * @return pointer to dmx array
   */
   uint8_t* dmxData(void);

   /*!
    * @brief Function called when DMX frame has been read
    * @discussion Sets a pointer to a function that is called
    *             on the break after a DMX frame has been received.
    *             Whatever happens in this function should be quick!
    *             Best used to set a flag that is polled outside of ISR for available data.
   */
   void setDataReceivedCallback(LXRecvCallback callback);

   /*!
    * @brief interrupt handler function
   */
   void IrqHandler();

  private:
   /*!
    * @brief Indicates mode ISR_OUTPUT_ENABLED or ISR_INPUT_ENABLED or ISR_DISABLED
   */
    uint8_t  _interrupt_status;

    /*!
   * @brief pin used to control direction of output driver chip
   */
    uint8_t _direction_pin;

    /*!
    * @brief Array of dmx data including start code
   */
    uint8_t  _dmxData[DMX_MAX_SLOTS+1];

    /*!
    * @brief Pointer to usart hardware interface. Usally set to SERCOMn
    */
    Sercom *const _hw;

    /*!
    * @brief USART configuration
    */
    dmx_usart_configuration _conf;

    /*!
    * @brief Adjust baud rate. Register values are computed
    */
    void setBaudRate(uint32_t baudrate);

};

/**************************
   Change the following lines to use other than SERCOM4.
   The following table shows the MKR1000 default uses of the SERCOMs
   and their associated pins:

   SERCOM0  Wire        11  12
   SERCOM1  SPI     8   9   10
   SERCOM2  ATWINC1501B SPI
   SERCOM3
   SERCOM4
   SERCOM5  Serial1 13  14

   Changing the SERCOM also requires changing the pin MUX.
   The following are MUX options for SERCOM3 and SERCOM4
   Mkr1000 pins

    Sercom 3
    00 S/0
    01 S/1
    06 SA/2
    07 SA/3
    08 SA/0
    09 SA/1
    10 SA/3

    Sercom 4
    04 SA/2
    05 SA/3

    see datasheet pg 33 for sercom4 mux options
    Need to be PB10 PB11, pad 2, pad 3  pad3=rx=3, pad 2=tx=1
    which correspond to MKR10000 pins 4 & 5.

    In general, tx is either pad 0 = CRegA_TXPO 0x0 or pad 2 = CRegA_TXPO 0x1
    see datasheet pg 481 for control register A

    NOTE:  You MUST change the name of the SERCOMn_Handler() function in LXSAMD21DMX.cpp
           if you use a different SERCOM
*/

#define PIN_DMX_RX DMX_RX
#define PIN_DMX_TX DMX_TX
#define PAD_DMX_RX PINMUX_PA04D_SERCOM0_PAD0 // TODO
#define PAD_DMX_TX PINMUX_PA05D_SERCOM0_PAD1 // TODO

// Set to PIO_SERCOM or PIO_SERCOM_ALT
#define MUX_DMX_RX PINMUX_PA05D_SERCOM0_PAD1
#define MUX_DMX_TX PINMUX_PA06D_SERCOM0_PAD2

#endif // ifndef LXSAM21_DMX_H
