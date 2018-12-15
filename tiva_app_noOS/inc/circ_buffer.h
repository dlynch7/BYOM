#ifndef CIRC_BUFFER_H
#define CIRC_BUFFER_H
// Header file for circ_buffer.c
// helps implement a circular buffer

// circ_buffer.c is based on a program of the same name
// in Chapter 11 (UART) of NU's mechatronics textbook.
// Here I adapt it into a library for the TM4C123G...

#include <stdio.h>
#include <stdint.h>

#define BUFLEN 2048
#define DECIMATION 10 // only store every DECIMATION samples in the circular buffer
#define NSAMPLES (2000/DECIMATION)

typedef struct {
    uint8_t     buf_state[BUFLEN];
    uint8_t     buf_halls[BUFLEN];
    int8_t      buf_abc_hi_lo_fl[BUFLEN];
    int16_t     buf_curr_A[BUFLEN];
    int16_t     buf_curr_B[BUFLEN];
    int16_t     buf_curr_C[BUFLEN];
    uint16_t    buf_read_index[BUFLEN];
    uint16_t    buf_write_index[BUFLEN];
} buffer_t;

typedef struct {
    uint8_t     state;
    uint8_t     halls;
    int8_t      abc_hi_lo_fl; // phase states: HIGH, LOW, FLOATING
    int16_t     curr_A;
    int16_t     curr_B;
    int16_t     curr_C;
    uint16_t    read_index;
    uint16_t    write_index;
} buf_out_t;

void buffer_reset(void); // resets read and write indices
uint16_t get_read_index(void); // return the value of the read index
uint16_t get_write_index(void);// return the value of the write index
uint8_t buffer_empty(void); // return true if the buffer is empty (read=write)
uint8_t buffer_full(void);  // return true if the buffer is full

// reads from current buffer index and increment read index by 1:
void buffer_read(buf_out_t *);
// write to the buffer and increment the write index by 1:
void buffer_write(  uint8_t,\
                    uint8_t,\
                    int8_t,\
                    int16_t,\
                    int16_t,\
                    int16_t,\
                    uint16_t,\
                    uint16_t);

#endif
