/************************************************************************/
/* XBoot Extensible AVR Bootloader                                      */
/*                                                                      */
/* tested with ATXMEGA64A3, ATXMEGA128A1, ATXMEGA256A1, ATXMEGA32A4     */
/*                                                                      */
/* xboot.c                                                              */
/*                                                                      */
/* Alex Forencich <alex@alexforencich.com>                              */
/*                                                                      */
/* Copyright (c) 2010 Alex Forencich                                    */
/*                                                                      */
/* Permission is hereby granted, free of charge, to any person          */
/* obtaining a copy of this software and associated documentation       */
/* files(the "Software"), to deal in the Software without restriction,  */
/* including without limitation the rights to use, copy, modify, merge, */
/* publish, distribute, sublicense, and/or sell copies of the Software, */
/* and to permit persons to whom the Software is furnished to do so,    */
/* subject to the following conditions:                                 */
/*                                                                      */
/* The above copyright notice and this permission notice shall be       */
/* included in all copies or substantial portions of the Software.      */
/*                                                                      */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,      */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF   */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND                */
/* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS  */
/* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN   */
/* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN    */
/* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE     */
/* SOFTWARE.                                                            */
/*                                                                      */
/************************************************************************/

#include "xmegaClocks.h"

#include "xboot.h"

#ifdef USE_INTERRUPTS
volatile unsigned char comm_mode;

volatile unsigned char rx_buff0;
volatile unsigned char rx_buff1;
volatile unsigned char rx_char_cnt;

volatile unsigned char tx_buff0;
volatile unsigned char tx_char_cnt;
#else
#ifdef __AVR_XMEGA__
unsigned char comm_mode;
#else // __AVR_XMEGA__
// Force data section on atmega
// Seems to be a bug in newer versions of gcc
// this ensures .bss is placed after .data
unsigned char comm_mode = 1;
#endif // __AVR_XMEGA__
#endif // USE_INTERRUPTS

unsigned char buffer[SPM_PAGESIZE];

#ifdef NEED_CODE_PROTECTION
unsigned char protected;
#endif // NEED_CODE_PROTECTION



// Main code
int main(void)
{
        cli();
        //RST.STATUS = 0x3f;
        ADDR_T address = 0;
        unsigned char in_bootloader = 0;
        unsigned char val = 0;
        int i = 0;
        uint32_t j;
        uint8_t k;

        #ifdef NEED_CODE_PROTECTION
        protected = 1;
        #endif // NEED_CODE_PROTECTION

        #ifdef USE_I2C_ADDRESS_NEGOTIATION
        unsigned short devid_bit;
        #endif // USE_I2C_ADDRESS_NEGOTIATION

        comm_mode = MODE_UNDEF;

        #ifdef USE_INTERRUPTS
        rx_char_cnt = 0;
        tx_char_cnt = 0;
        #endif // USE_INTERRUPTS

        // Initialization section
        // Entry point and communication methods are initialized here
        // --------------------------------------------------


      	init_clock(SYSCLK,PLL,true,CLOCK_CALIBRATION);


        #ifdef NEED_INTERRUPTS
        // remap interrupts to boot section
        CCP = CCP_IOREG_gc;
        #ifdef USE_INTERRUPTS
        PMIC.CTRL = PMIC_IVSEL_bm | PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;
        #else
        PMIC.CTRL = PMIC_IVSEL_bm;
        #endif // USE_INTERRUPTS
        #endif // NEED_INTERRUPTS


        // LED

        #ifdef USE_LED
        // Initialize LED pin
        LED_PORT.DIRSET = (1 << LED_PIN);
        #if LED_PIN_INV
        LED_PORT.OUTCLR = (1 << LED_PIN);
        #else
        LED_PORT.OUTSET = (1 << LED_PIN);
        #endif // LED_PIN_INV
        #endif // USE_LED

        // Enter pin

        // Initialize UART
        uart_init();

        // Initialize RX pin pull-up

        #ifdef UART_RX_PUEN
        // Enable RX pin pullup
        UART_RX_PIN_CTRL = 0x18;   // war 0x18 , war jetzt 0x90
        #endif // UART_RX_PUEN



        // Initialize UART EN pin
        UART_EN_PORT.DIRSET = (1 << UART_TxEN_PIN) | (1 << UART_RxEN_PIN) | (1 << UART_TX_PIN);
        UART_EN_PORT.OUTCLR = (1 << UART_TxEN_PIN) | (1 << UART_RxEN_PIN);

        /*
        unsigned char p;
        for(p=67;p<87;p++)
          send_char(p);
        */

        #ifdef USE_FIFO
        // Initialize FIFO
        fifo_init();
        #endif // USE_FIFO

        // --------------------------------------------------
        // End initialization section

        // One time trigger section
        // Triggers that are checked once, regardless of
        // whether or not USE_ENTER_DELAY is selected
        // --------------------------------------------------

        // --------------------------------------------------
        // End one time trigger section

#ifdef USE_ENTER_DELAY
        #pragma message "Benutze Start-Verzögerung"
        k = ENTER_BLINK_COUNT*2;
        j = ENTER_BLINK_WAIT;
        while (!in_bootloader && k > 0)
        {
                if (j-- <= 0)
                {
                        #ifdef USE_LED
                        LED_PORT.OUTTGL = (1 << LED_PIN);
                        #endif // USE_LED
                        j = ENTER_BLINK_WAIT;
                        k--;
                }
#else // USE_ENTER_DELAY
                // Need a small delay when not running loop
                // so we don't accidentally enter the bootloader
                // on power-up with USE_ENTER_PIN selected
                asm("nop");
                asm("nop");
                asm("nop");
                asm("nop");
#endif // USE_ENTER_DELAY

                // Main trigger section
                // Set in_bootloader here to enter the bootloader
                // Checked when USE_ENTER_DELAY is selected
                // --------------------------------------------------

                #ifdef USE_ENTER_UART
                // Check for received character
                #ifdef ENTER_UART_NEED_SYNC
                #pragma message "Benutze UART SYNC"
                if (uart_char_received() && (uart_cur_char() == CMD_SYNC))
                #else // ENTER_UART_NEED_SYNC
                if (uart_char_received())
                #endif // ENTER_UART_NEED_SYNC
                {
                        in_bootloader = 1;
                        comm_mode = MODE_UART;
                }

                #endif // USE_ENTER_UART

                #ifdef USE_ENTER_FIFO
                // Check for received character
                #ifdef ENTER_FIFO_NEED_SYNC
                if (fifo_char_received() && (fifo_cur_char() == CMD_SYNC))
                #else // ENTER_FIFO_NEED_SYNC
                if (fifo_char_received())
                #endif // ENTER_FIFO_NEED_SYNC
                {
                        in_bootloader = 1;
                        comm_mode = MODE_FIFO;
                }

                #endif // USE_ENTER_FIFO

                #ifdef USE_ENTER_EEPROM
                if (enter_eeprom_check())
                {
                    enter_eeprom_reset();
                    in_bootloader = 1;
                }
                #endif // USE_ENTER_EEPROM
                // --------------------------------------------------
                // End main trigger section

                WDT_Reset();

#ifdef USE_ENTER_DELAY
        }
#endif // USE_ENTER_DELAY

        #ifdef USE_INTERRUPTS
        // Enable interrupts
        sei();
        #endif // USE_INTERRUPTS

        #ifdef USE_WATCHDOG
        WDT_EnableAndSetTimeout();
        #endif // USE_WATCHDOG

        // Main bootloader
        while (in_bootloader)
        {
                #ifdef USE_LED
                LED_PORT.OUTTGL = (1 << LED_PIN);
                #endif // USE_LED

                val = get_char();

                #ifdef USE_WATCHDOG
                WDT_Reset();
                #endif // USE_WATCHDOG

                // Main bootloader parser
                // check autoincrement status
                if (val == CMD_CHECK_AUTOINCREMENT)
                {
                        // yes, it is supported
                        send_char(REPLY_YES);
                }
                // Set address
                else if (val == CMD_SET_ADDRESS)
                {
                        // Read address high then low
                        address = get_2bytes();
                        // acknowledge
                        send_char(REPLY_ACK);
                }
                // Extended address
                else if (val == CMD_SET_EXT_ADDRESS)
                {
                        // Read address high then low
                        //address = ((ADDR_T)get_char() << 16) | get_2bytes();
                        asm volatile (
                                "call get_char"    "\n\t"
                                "mov  %C0,r24"     "\n\t"
                                "call get_2bytes"  "\n\t"
                                "clr  %D0"         "\n\t"
                                : "=r" (address)
                                :
                        );

                        // acknowledge
                        send_char(REPLY_ACK);
                }
                // Chip erase
                else if (val == CMD_CHIP_ERASE)
                {
                        // Erase the application section
                        // XMEGA E5: ERASE_APP NVM command (0x20) erases the entire flash - as a workaround, we erase page-by-page.
                        // From Atmel Support: "The NVM controller design is such that the entire flash will get erased always when application/bootloader erase is called."
                        #if defined(__AVR_ATxmega8E5__) || defined(__AVR_ATxmega16E5__) || defined(__AVR_ATxmega32E5__)
                        for(uint32_t addr = APP_SECTION_START; addr < APP_SECTION_END; addr += SPM_PAGESIZE)
                        {
	                            Flash_EraseWriteApplicationPage(addr);
	                            // Wait for completion
	                            #ifdef __AVR_XMEGA__
	                            #ifdef USE_WATCHDOG
	                            while (NVM_STATUS & NVM_NVMBUSY_bp)
	                            {
	                                // reset watchdog while waiting for erase completion
	                                WDT_Reset();
	                            }
	                            #else // USE_WATCHDOG
	                            SP_WaitForSPM();
	                            #endif // USE_WATCHDOG
	                            #endif // __AVR_XMEGA__
                        }
                        #else
                        Flash_EraseApplicationSection();
                        // Wait for completion
                        #ifdef __AVR_XMEGA__
                        #ifdef USE_WATCHDOG
                        while (NVM_STATUS & NVM_NVMBUSY_bp)
                        {
	                            // reset watchdog while waiting for erase completion
	                            WDT_Reset();
                        }
                        #else // USE_WATCHDOG
                        SP_WaitForSPM();
                        #endif // USE_WATCHDOG
                        #endif // __AVR_XMEGA__
                        #endif

                        // Erase EEPROM
                        EEPROM_erase_all();

                        // turn off read protection
                        #ifdef NEED_CODE_PROTECTION
                        protected = 0;
                        #endif // NEED_CODE_PROTECTION

                        // acknowledge
                        send_char(REPLY_ACK);
                }
                #ifdef ENABLE_BLOCK_SUPPORT
                // Check block load support
                else if (val == CMD_CHECK_BLOCK_SUPPORT )
                {
                        // yes, it is supported
                        send_char(REPLY_YES);
                        // Send block size (page size)
                        send_char((SPM_PAGESIZE >> 8) & 0xFF);
                        send_char(SPM_PAGESIZE & 0xFF);
                }
                // Block load
                else if (val == CMD_BLOCK_LOAD)
                {
                        // Block size
                        i = get_2bytes();
                        // Memory type
                        val = get_char();
                        // Load it
                        send_char(BlockLoad(i, val, &address));
                }
                // Block read
                else if (val == CMD_BLOCK_READ)
                {
                        // Block size
                        i = get_2bytes();
                        // Memory type
                        val = get_char();
                        // Read it
                        BlockRead(i, val, &address);
                }
                #endif // ENABLE_BLOCK_SUPPORT
                #ifdef ENABLE_FLASH_BYTE_SUPPORT
                // Read program memory byte
                else if (val == CMD_READ_BYTE)
                {
                        unsigned int w = Flash_ReadWord((address << 1));

                        #ifdef ENABLE_CODE_PROTECTION
                        if (protected)
                                w = 0xffff;
                        #endif // ENABLE_CODE_PROTECTION

                        send_char(w >> 8);
                        send_char(w);

                        address++;
                }
                // Write program memory low byte
                else if (val == CMD_WRITE_LOW_BYTE)
                {
                        // get low byte
                        i = get_char();
                        send_char(REPLY_ACK);
                }
                // Write program memory high byte
                else if (val == CMD_WRITE_HIGH_BYTE)
                {
                        // get high byte; combine
                        i |= (get_char() << 8);
                        Flash_LoadFlashWord((address << 1), i);
                        address++;
                        send_char(REPLY_ACK);
                }
                // Write page
                else if (val == CMD_WRITE_PAGE)
                {
                        if (address >= (APP_SECTION_SIZE>>1))
                        {
                                // don't allow bootloader overwrite
                                send_char(REPLY_ERROR);
                        }
                        else
                        {
                                Flash_WriteApplicationPage( address << 1);
                                send_char(REPLY_ACK);
                        }
                }
                #endif // ENABLE_FLASH_BYTE_SUPPORT
                #ifdef ENABLE_EEPROM_BYTE_SUPPORT
                // Write EEPROM memory
                else if (val == CMD_WRITE_EEPROM_BYTE)
                {
                        EEPROM_write_byte(address, get_char());
                        address++;
                        send_char(REPLY_ACK);
                }
                // Read EEPROM memory
                else if (val == CMD_READ_EEPROM_BYTE)
                {
                        char c = EEPROM_read_byte(address);

                        #ifdef ENABLE_EEPROM_PROTECTION
                        if (protected)
                                c = 0xff;
                        #endif // ENABLE_EEPROM_PROTECTION

                        send_char(c);
                        address++;
                }
                #endif // ENABLE_EEPROM_BYTE_SUPPORT
                #ifdef ENABLE_LOCK_BITS
#ifdef __AVR_XMEGA__
                // Write lockbits
                else if (val == CMD_WRITE_LOCK_BITS)
                {
                        SP_WriteLockBits( get_char() );
                        send_char(REPLY_ACK);
                }
                // Read lockbits
                else if (val == CMD_READ_LOCK_BITS)
                {
                        send_char(SP_ReadLockBits());
                }
#endif // __AVR_XMEGA__
                #endif // ENABLE_LOCK_BITS
                #ifdef ENABLE_FUSE_BITS
#ifdef __AVR_XMEGA__
                // Read low fuse bits
                else if (val == CMD_READ_LOW_FUSE_BITS)
                {
                        send_char(SP_ReadFuseByte(0));
                }
                // Read high fuse bits
                else if (val == CMD_READ_HIGH_FUSE_BITS)
                {
                        send_char(SP_ReadFuseByte(1));
                }
                // Read extended fuse bits
                else if (val == CMD_READ_EXT_FUSE_BITS)
                {
                        send_char(SP_ReadFuseByte(2));
                }
#endif // __AVR_XMEGA__
                #endif // ENABLE_FUSE_BITS
                // Enter and leave programming mode
                else if ((val == CMD_ENTER_PROG_MODE) || (val == CMD_LEAVE_PROG_MODE))
                {
                        // just acknowledge
                        send_char(REPLY_ACK);
                }
                // Exit bootloader
                else if (val == CMD_EXIT_BOOTLOADER)
                {
                        in_bootloader = 0;
                        send_char(REPLY_ACK);
                }
                // Get programmer type
                else if (val == CMD_PROGRAMMER_TYPE)
                {
                        // serial
                        send_char('S');
                }
                // Return supported device codes
                else if (val == CMD_DEVICE_CODE)
                {
                        // send only this device
                        send_char(123); // TODO
                        // terminator
                        send_char(0);
                }
                // Set LED, clear LED, and set device type
                else if ((val == CMD_SET_LED) || (val == CMD_CLEAR_LED) || (val == CMD_SET_TYPE))
                {
                        // discard parameter
                        get_char();
                        send_char(REPLY_ACK);
                }
                // Return program identifier
                else if (val == CMD_PROGRAM_ID)
                {
                        _delay_ms(1);
                        send_char('A');
                        send_char('V');
                        send_char('R');
                        send_char('1');
                        send_char('0');
                        send_char('9');
                        send_char('+');
                }
                // Read software version
                else if (val == CMD_VERSION)
                {
                        send_char('0' + XBOOT_VERSION_MAJOR);
                        send_char('0' + XBOOT_VERSION_MINOR);
                }
                // Read signature bytes
                else if (val == CMD_READ_SIGNATURE)
                {
                        send_char(SIGNATURE_2);
                        send_char(SIGNATURE_1);
                        send_char(SIGNATURE_0);
                }
                #ifdef ENABLE_CRC_SUPPORT
                else if (val == CMD_CRC)
                {
                        uint32_t start = 0;
                        uint32_t length = 0;
                        uint16_t crc;

                        val = get_char();

                        switch (val)
                        {
                                case SECTION_FLASH:
                                        length = PROGMEM_SIZE;
                                        break;
                                case SECTION_APPLICATION:
                                        length = APP_SECTION_SIZE;
                                        break;
                                case SECTION_BOOT:
                                        start = BOOT_SECTION_START;
                                        length = BOOT_SECTION_SIZE;
                                        break;
                                #ifdef ENABLE_API
                                case SECTION_APP:
                                        length = XB_APP_SIZE;
                                        break;
                                case SECTION_APP_TEMP:
                                        start = XB_APP_TEMP_START;
                                        length = XB_APP_TEMP_SIZE;
                                        break;
                                #endif // ENABLE_API
                                default:
                                        send_char(REPLY_ERROR);
                                        continue;
                        }

                        crc = crc16_block(start, length);

                        send_char((crc >> 8) & 0xff);
                        send_char(crc & 0xff);
                }
                #endif // ENABLE_CRC_SUPPORT

                else if ( val== 0 )
                { ;
                }
                // ESC (0x1b) to sync
                // otherwise, error
                else if (val != CMD_SYNC)
                {
                        send_char(REPLY_ERROR);
                }

                // Wait for any lingering SPM instructions to finish
                Flash_WaitForSPM();

                // End of bootloader main loop
        }

        #ifdef NEED_INTERRUPTS
        // Disable interrupts
        cli();
        #endif // NEED_INTERRUPTS

        // Bootloader exit section
        // Code here runs after the bootloader has exited,
        // but before the application code has started
        // --------------------------------------------------

        #ifdef USE_FIFO
        // Shut down FIFO
        fifo_deinit();
        #endif // USE_FIFO


        //#ifdef USE_UART
        // Shut down UART
        uart_deinit();

        // Disable RX pin pull-up
        #ifdef UART_RX_PUEN
        // Disable RX pin pullup
        UART_RX_PIN_CTRL = 0;
        #endif // UART_RX_PUEN

        // Shut down UART EN pin
        UART_EN_PORT.DIRCLR = (1 << UART_TxEN_PIN) | (1 << UART_RxEN_PIN);
        UART_EN_PORT.OUTCLR = (1 << UART_TxEN_PIN) | (1 << UART_RxEN_PIN);

#ifdef __AVR_XMEGA__
        #ifdef LOCK_SPM_ON_EXIT
        // Lock SPM writes
        SP_LockSPM();
        #endif // LOCK_SPM_ON_EXIT
#endif // __AVR_XMEGA__

        // LED
        #ifdef USE_LED
        // Turn off LED on exit
        LED_PORT.DIRCLR = (1 << LED_PIN);
        LED_PORT.OUTCLR = (1 << LED_PIN);
        #endif // USE_LED

#ifdef __AVR_XMEGA__
        #ifdef NEED_INTERRUPTS
        // remap interrupts back to application section
        CCP = CCP_IOREG_gc;
        PMIC.CTRL = 0;
        #endif // NEED_INTERRUPTS
#endif // __AVR_XMEGA__

        #ifdef USE_WATCHDOG
        WDT_Disable();
        #endif // USE_WATCHDOG

        // --------------------------------------------------
        // End bootloader exit section

        // Jump into main code
        asm("jmp 0");

        #ifdef __builtin_unreachable
        // Size optimization as the asm jmp will not return
        // However, it seems it is not available on older versions of gcc
        __builtin_unreachable();
        #endif
}

#ifdef USE_INTERRUPTS
unsigned char __attribute__ ((noinline)) get_char(void)
{
        unsigned char ret;

        while (rx_char_cnt == 0) { };

        cli();

        ret = rx_buff0;
        rx_buff0 = rx_buff1;
        rx_char_cnt--;

        sei();

        return ret;
}

void __attribute__ ((noinline)) send_char(unsigned char c)
{
        while (1)
        {
                cli();

                if (tx_char_cnt == 0)
                {
                        tx_buff0 = c;
                        tx_char_cnt = 1;

                        #ifdef USE_UART
                        if (comm_mode == MODE_UART)
                        {
                                uart_send_char(c);
                        }
                        #endif // USE_UART

                        #ifdef USE_I2C
                        #error I2C interrupts are not yet implemented
                        #endif

                        #ifdef USE_FIFO
                        if (comm_mode == MODE_FIFO)
                        {
                                fifo_send_char(c);
                        }
                        #endif // USE_FIFO

                        sei();
                        return;
                }

                sei();
        }
}

#else

unsigned char __attribute__ ((noinline)) get_char(void)
{
        unsigned char ret;

        while (1)
        {
                #ifdef USE_UART
                // Get next character
                if (comm_mode == MODE_UNDEF || comm_mode == MODE_UART)
                {
                        if (uart_char_received())
                        {
								_delay_us(12); // war 52
                                comm_mode = MODE_UART;
                                return uart_cur_char();
                    }
                }
                #endif // USE_UART

                #ifdef USE_I2C
                // Get next character
                if (comm_mode == MODE_UNDEF || comm_mode == MODE_I2C)
                {
#ifdef __AVR_XMEGA__
                        if (i2c_address_match())
                        {
                                // Address match, send ACK
                                i2c_send_ack();
                                comm_mode = MODE_I2C;
                                first_byte = 1;
                        }
                        if (i2c_char_received())
                        {
                                // Data has arrived
                                ret = i2c_cur_char();
                                i2c_send_ack();
                                return ret;
                        }
                        if (i2c_ready_data())
                        {
                                if (!first_byte && i2c_got_ack())
                                {
                                        i2c_end_transmission(); // end transaction
                                }
                                else
                                {
                                        first_byte = 0;
                                        // Wants data, but there is no data to send...
                                        // also include NAK
                                        i2c_send_char(REPLY_ERROR);
                                        i2c_send_nak();
                                }
                        }
#else // __AVR_XMEGA__
                        #error Not implemented!
#endif // __AVR_XMEGA__
                }
                #endif // USE_I2C

                #ifdef USE_FIFO
                // Get next character
                if (comm_mode == MODE_UNDEF || comm_mode == MODE_FIFO)
                {
                        if (fifo_char_received())
                        {
                                comm_mode = MODE_FIFO;
                                return fifo_cur_char();
                        }
                }
                #endif // USE_FIFO

        }

        return ret;
}

void __attribute__ ((noinline)) send_char(unsigned char c)
{
      UART_EN_PORT.DIRSET = (1 << UART_TxEN_PIN) | (1 << UART_RxEN_PIN) | (1 << UART_TX_PIN); // 0xb0;
			UART_DEVICE.CTRLB = 0;
      UART_EN_PORT.OUTSET = (1 << UART_RxEN_PIN);
			UART_DEVICE.CTRLB = USART_TXEN_bm;
      UART_EN_PORT.OUTSET = (1 << UART_TxEN_PIN) ;
      UART_DEVICE.STATUS |= USART_TXCIF_bm;
			_delay_us(12); // mindestens 8 beim atxmega256a3u, 5 bei stxmega32a4u
			UART_DEVICE.DATA = c; //	UDR0 = data; 			        // Start transmittion
			_delay_us(12); // mindestens 8 beim atxmega256a3u, 5 bei stxmega32a4u
      while (!(UART_DEVICE.STATUS & USART_TXCIF_bm))
      { }
      UART_DEVICE.STATUS |= USART_TXCIF_bm;
      UART_DEVICE.STATUS |= USART_RXCIF_bm;
			_delay_us(12); // mindestens 8 beim atxmega256a3u, 5 bei atxmega32a4u

      UART_EN_PORT.OUTCLR = (1 << UART_TxEN_PIN);
      UART_DEVICE.CTRLB = 0;
      UART_EN_PORT.OUTCLR = (1 << UART_RxEN_PIN);
      UART_DEVICE.CTRLB = USART_RXEN_bm;
}

#endif // USE_INTERRUPTS

unsigned int __attribute__ ((noinline)) get_2bytes()
{
        // return (get_char() << 8) | get_char();
        unsigned int result;
        asm volatile (
                "call get_char"    "\n\t"
                "push r24"         "\n\t"
                "call get_char"    "\n\t"
                "pop  %B0"         "\n\t"
                "mov  %A0,r24"     "\n\t"
                : "=r" (result)
                :
        );
        return result;
}

void clear_buffer(void)
{
        unsigned char *ptr = buffer;
        for (long i = 0; i < SPM_PAGESIZE; i++)
        {
                *(ptr++) = 0xff;
        }
}

unsigned char BlockLoad(unsigned int size, unsigned char mem, ADDR_T *address)
{
        ADDR_T tempaddress;

        #ifdef USE_WATCHDOG
        WDT_Reset();
        #endif // USE_WATCHDOG

        // fill up buffer
        for (int i = 0; i < SPM_PAGESIZE; i++)
        {
                char c = 0xff;

                if (i < size)
                        c = get_char();

                buffer[i] = c;
        }

        // EEPROM memory type.
        if(mem == MEM_EEPROM)
        {
                EEPROM_write_block(*address, buffer, size);
                (*address) += size;

                return REPLY_ACK; // Report programming OK
        }

        // Flash memory type
#ifdef __AVR_XMEGA__
        else if (mem == MEM_FLASH || mem == MEM_USERSIG)
#else // __AVR_XMEGA__
        else if (mem == MEM_FLASH)
#endif // __AVR_XMEGA__
        {
                // NOTE: For flash programming, 'address' is given in words.
                tempaddress = (*address) << 1;  // Store address in page.

                (*address) += size >> 1;

#ifdef __AVR_XMEGA__

                if (mem == MEM_FLASH)
                {
                        #ifdef ENABLE_FLASH_ERASE_WRITE
                        Flash_ProgramPage(tempaddress, buffer, 1);
                        #else
                        Flash_ProgramPage(tempaddress, buffer, 0);
                        #endif
                }
                else if (mem == MEM_USERSIG)
                {
                        Flash_LoadFlashPage(buffer);
                        Flash_EraseUserSignatureRow();
                        Flash_WaitForSPM();
                        Flash_WriteUserSignatureRow();
                        Flash_WaitForSPM();
                }

#else // __AVR_XMEGA__
                #ifdef ENABLE_FLASH_ERASE_WRITE
                Flash_ProgramPage(tempaddress, buffer, 1);
                #else
                Flash_ProgramPage(tempaddress, buffer, 0);
                #endif
#endif // __AVR_XMEGA__

                return REPLY_ACK; // Report programming OK
        }

        // Invalid memory type?
        else
        {
                return REPLY_ERROR;
        }
}



void BlockRead(unsigned int size, unsigned char mem, ADDR_T *address)
{
        int offset = 0;
        int size2 = size;

        // EEPROM memory type.

        if (mem == MEM_EEPROM) // Read EEPROM
        {
                EEPROM_read_block(*address, buffer, size);
                (*address) += size;
        }

        // Flash memory type.
#ifdef __AVR_XMEGA__
        else if (mem == MEM_FLASH || mem == MEM_USERSIG || mem == MEM_PRODSIG)
#else // __AVR_XMEGA__
        else if (mem == MEM_FLASH)
#endif // __AVR_XMEGA__
        {
                (*address) <<= 1; // Convert address to bytes temporarily.

                do
                {
#ifdef __AVR_XMEGA__
                        if (mem == MEM_FLASH)
                        {
                                buffer[offset++] = Flash_ReadByte(*address);
                        }
                        else if (mem == MEM_USERSIG)
                        {
                                buffer[offset++] = SP_ReadUserSignatureByte(*address);
                        }
                        else if (mem == MEM_PRODSIG)
                        {
                                buffer[offset++] = SP_ReadCalibrationByte(*address);
                        }
#else // __AVR_XMEGA__
                        buffer[offset++] = Flash_ReadByte(*address);
#endif // __AVR_XMEGA__

                        Flash_WaitForSPM();

                        (*address)++;    // Select next word in memory.
                        size--;          // Subtract two bytes from number of bytes to read
                } while (size);         // Repeat until all block has been read

                (*address) >>= 1;       // Convert address back to Flash words again.
        }
        else
        {
                // bad memory type
                return;
        }

        // code protection
        if (
        #ifdef ENABLE_CODE_PROTECTION
                (protected && mem == MEM_FLASH) ||
        #endif // ENABLE_CODE_PROTECTION
        #ifdef ENABLE_EEPROM_PROTECTION
                (protected && mem == MEM_EEPROM) ||
        #endif // ENABLE_EEPROM_PROTECTION
        #ifdef ENABLE_BOOTLOADER_PROTECTION
                (*address >= (BOOT_SECTION_START >> 1) && mem == MEM_FLASH) ||
        #endif // ENABLE_BOOTLOADER_PROTECTION
                0
        )
                clear_buffer();

        // send bytes
        for (int i = 0; i < size2; i++)
        {
                send_char(buffer[i]);
        }

}

uint16_t crc16_block(uint32_t start, uint32_t length)
{
        uint16_t crc = 0;

        int bc = SPM_PAGESIZE;

        for ( ; length > 0; length--)
        {
                if (bc == SPM_PAGESIZE)
                {
                        Flash_ReadFlashPage(buffer, start);
                        start += SPM_PAGESIZE;
                        bc = 0;
                }

                crc = _crc16_update(crc, buffer[bc]);

                bc++;
        }

        return crc;
}
