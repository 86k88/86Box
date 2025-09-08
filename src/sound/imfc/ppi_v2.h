#ifndef PPI8255_H
#define PPI8255_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
/* --- Callback interface --- */
typedef struct {
    void (*stb_changed)(int port, int level, void *user);  // Strobe line
    void (*ack_changed)(int port, int level, void *user);  // Acknowledge line
    void (*ibf_changed)(int port, int level, void *user);  // Input buffer full
    void (*obf_changed)(int port, int level, uint8_t data, void *user);  // Output buffer full
    void (*intr_changed)(int port, int level, void *user); // Interrupt request
} ppi8255_callbacks_t;

/* --- Port direction enum --- */
typedef enum {
    PPI_MODE0 = 0,
    PPI_MODE1_INPUT,
    PPI_MODE1_OUTPUT,
    PPI_MODE2   // Port A only
} ppi8255_mode_t;

/* --- Control word type --- */
typedef enum {
    PPI_STD,
    PPI_BSR
} ppi8255_control_t;

/* --- Access check result --- */
typedef enum {
    PPI_ACCESS_OK = 0,       // read/write allowed
    PPI_ACCESS_WARN = 1,     // allowed but only returns latch
    PPI_ACCESS_ILLEGAL = 2,  // illegal operation
    PPI_ACCESS_UNKNOWN = 3   // invalid port or undefined
} ppi_access_t;

/* --- Operation type --- */
typedef enum {
    PPI_OP_READ,
    PPI_OP_WRITE
} ppi8255_op_t;

/* --- PPI device state --- */
typedef struct {
    uint8_t port_latches[3];  // Internal latches (A, B, C)
    uint8_t out_port[3];      // Output port values
    uint8_t in_port[3];       // Input port values

    uint8_t control;          // Control word register

    ppi8255_mode_t mode_a;    // Mode for Port A
    ppi8255_mode_t mode_b;    // Mode for Port B

    bool port_a_in;
    bool port_b_in;
    bool port_c_upper_in;     // PC7-4
    bool port_c_lower_in;     // PC3-0

    bool ibf_a, obf_a, intr_a; // Handshake/status A
    bool ibf_b, obf_b, intr_b; // Handshake/status B
	bool inte_a, inte_b;
    bool strb_a, strb_b;       // Strobe flip-flops

    ppi8255_callbacks_t cb;    // Callbacks
    void *user;                // User data
} ppi8255_t;

/* --- Public API --- */
#ifdef __cplusplus
extern "C" {
#endif
uint8_t ppi8255_portc_read(ppi8255_t *ppi);
void 	 check_intr(ppi8255_t *ppi, uint8_t int_num, uint8_t op);
void     ppi_init(ppi8255_t *ppi, ppi8255_callbacks_t *cb, void *user);
uint8_t  ppi_read(ppi8255_t *ppi, uint8_t port);
void     ppi_write(ppi8255_t *ppi, uint8_t port, uint8_t data);
void     ppi_strobe(ppi8255_t *ppi, uint8_t group);
ppi8255_control_t ppi_decode_control(ppi8255_t *ppi, uint8_t control_val);

#ifdef __cplusplus
}
#endif

#endif /* PPI8255_H */
