// circ_buffer.c is based on a program of the same name
// in Chapter 11 (UART) of NU's mechatronics textbook.
// Here I adapt it into a library for the TM4C123G...

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include "circ_buffer.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/gpio.h"

#include "setup.h" // provides access to LED color macros

static volatile uint16_t read = 0, write = 0;	// circ buffer indices
static volatile buffer_t data_buf;  // array that stores the data
//static volatile buf_out_t data_out;

void buffer_reset(void) { // resets read and write indices
    read = 0;
    write = 0;
}

uint16_t get_read_index(void) { // return the value of the read index
	return read;
}
uint16_t get_write_index(void) {// return the value of the write index
	return write;
}

uint8_t buffer_empty(void) { // return true if the buffer is empty (read=write)
	return read==write;
}

uint8_t buffer_full(void) { // return true if the buffer is full
	return ((write + 1) % BUFLEN) == read;
}

// reads from current buffer index and increment read index by 1:
void buffer_read(buf_out_t *data_out) // input arg is pointer to struct of type buf_out_struct
{
	while(buffer_empty())  // wait for data to be in the buffer
    {
        //GPIOPinWrite(GPIO_PORTF_BASE, LED_BLUE, LED_BLUE); // LED on
    }
    //GPIOPinWrite(GPIO_PORTF_BASE, LED_BLUE, 0); // LED off

    data_out->state         = data_buf.buf_state[read];
	data_out->halls         = data_buf.buf_halls[read];
	data_out->abc_hi_lo_fl  = data_buf.buf_abc_hi_lo_fl[read];
    data_out->curr_A        = data_buf.buf_curr_A[read];
    data_out->curr_B        = data_buf.buf_curr_B[read];
    data_out->curr_C        = data_buf.buf_curr_C[read];
    data_out->read_index    = data_buf.buf_read_index[read];
    data_out->write_index       = data_buf.buf_write_index[read];
	++read;
	if(read >= BUFLEN) // wraparound
    {
		read = 0;
	}
}

// write to the buffer and increment the write index by 1:
void buffer_write(  uint8_t     state,\
                    uint8_t     halls,\
                    int8_t      abc_hi_lo_fl,\
                    int16_t     curr_A,\
                    int16_t     curr_B,\
                    int16_t     curr_C,\
                    uint16_t    read_index,\
                    uint16_t    write_index)
{
	if(!buffer_full()) { // if the buffer is full, the data is lost
		data_buf.buf_state[write]           = state;
		data_buf.buf_halls[write]           = halls;
		data_buf.buf_abc_hi_lo_fl[write]    = abc_hi_lo_fl;
        data_buf.buf_curr_A[write]          = curr_A;
        data_buf.buf_curr_B[write]          = curr_B;
        data_buf.buf_curr_C[write]          = curr_C;
        data_buf.buf_read_index[write]      = read_index;
        data_buf.buf_write_index[write]     = write_index;
		++write;
		if(write >= BUFLEN) // wraparound
        {
			write = 0;
		}
    }
}
