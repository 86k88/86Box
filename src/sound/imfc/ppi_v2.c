#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
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
    PPI_MODE2,   // Port A only
} ppi8255_mode_t;

typedef enum {
	PPI_STD,
	PPI_BSR
} ppi8255_control_t;
typedef enum {
    PPI_ACCESS_OK = 0,        // read/write allowed
    PPI_ACCESS_WARN = 1,      // allowed but only returns latch, not external
    PPI_ACCESS_ILLEGAL = 2,    // illegal operation
	PPI_ACCESS_UNKNOWN = 3    // illegal operation
} ppi_access_t;

/* --- PPI device state --- */
typedef struct {
    /* Data latches */
    uint8_t port_latches[3];
	uint8_t out_port[3];
	uint8_t in_port[3];
    /* Control word register */
    uint8_t control;

    /* Port configuration */
    ppi8255_mode_t mode_a;
    ppi8255_mode_t mode_b;
	bool port_a_in;
	bool port_b_in;	
    bool port_c_upper_in; // PC7-4
    bool port_c_lower_in; // PC3-0
	
    /* Handshake flip-flops (per port group) */
    bool ibf_a, obf_a, intr_a;
    bool ibf_b, obf_b, intr_b;
	bool inte_a, inte_b;
	bool strb_a, strb_b;
    // Mode 2 on port A has more complex INTR logic

    /* External callbacks */
    ppi8255_callbacks_t cb;
    void *user;
} ppi8255_t;



typedef enum {
    PPI_OP_READ,
    PPI_OP_WRITE
} ppi8255_op_t;
void check_intr(ppi8255_t *ppi, uint8_t int_num, uint8_t op)
{
	int_num = int_num & 0x01;
    switch (int_num) {
        case 0: // Port A
            switch (ppi->mode_a) {
                case PPI_MODE0:
                    // Always legal (simple I/O)
                    break;

				case PPI_MODE1_INPUT:
					if (op == 0) { // STB
						ppi->ibf_a = 1;
						ppi->intr_a = 1;
						if (ppi->cb.ibf_changed) ppi->cb.ibf_changed(0, 1, ppi->user);
					}
					if (op == 1) { // CPU read
						ppi->ibf_a = 0;
						ppi->intr_a = 0;
						if (ppi->cb.ibf_changed) ppi->cb.ibf_changed(0, 0, ppi->user);
					}
					if (ppi->inte_a){
					if (ppi->cb.intr_changed) 
						ppi->cb.intr_changed(0, ppi->intr_a, ppi->user);
					}
					break;


				case PPI_MODE1_OUTPUT:
					if (op == 2) { 
						ppi->obf_a = 0;
						ppi->intr_a = 1;
						ppi->out_port[0] = ppi->port_latches[0];
						if (ppi->cb.obf_changed) ppi->cb.obf_changed(0, 0, ppi->out_port[0], ppi->user);
					}
					if (op == 3) { 
						ppi->obf_a = 1;
						ppi->intr_a = 0;
						if (ppi->cb.obf_changed) ppi->cb.obf_changed(0, 1, 0xFF, ppi->user);
					}
					if (ppi->inte_a){
					if (ppi->cb.intr_changed) 
						ppi->cb.intr_changed(0, ppi->intr_a, ppi->user);
					}
					break;

                case PPI_MODE2:
					break;
            }
            break;

        case 1: // Port B
            switch (ppi->mode_b) {
                case PPI_MODE0:
                    break;

				case PPI_MODE1_INPUT:
					if (op == 0) { // STB
						ppi->ibf_b = 1;
						ppi->intr_b = 1;
						if (ppi->cb.ibf_changed) ppi->cb.ibf_changed(1, 1, ppi->user);
					}
					if (op == 1) { // CPU read
						ppi->ibf_b = 0;
						ppi->intr_b = 0;
						if (ppi->cb.ibf_changed) ppi->cb.ibf_changed(1, 0, ppi->user);
					}
					if(ppi->inte_b){
					if (ppi->cb.intr_changed) 
						ppi->cb.intr_changed(1, ppi->intr_b, ppi->user);
					}
					break;


				case PPI_MODE1_OUTPUT:
					if (op == 2) {
						ppi->obf_b = 0;
						ppi->intr_b = 1;
						ppi->out_port[1] = ppi->port_latches[1];
						if (ppi->cb.obf_changed) ppi->cb.obf_changed(1,0, ppi->out_port[1], ppi->user);
					}
					if (op == 3) { 
						ppi->obf_b = 1;
						ppi->intr_b = 0;
						if (ppi->cb.obf_changed) ppi->cb.obf_changed(1,1, 0xFF, ppi->user);
					}
					if(ppi->inte_b){
					if (ppi->cb.intr_changed) 
						ppi->cb.intr_changed(1, ppi->intr_b, ppi->user);
					}
					break;

                default:
                    break;
            }
            break;
			
	}
	
}
// ai helped me with this :P
uint8_t ppi8255_portc_read(ppi8255_t *ppi)
{
    /* start with latched/last-written output value */
    uint8_t val = ppi->port_latches[2];

    /* port A in mode 1? */
    if (ppi->mode_a == PPI_MODE1_INPUT) {
        /* PC5 = IBF_A (input buffer full) */
        if (!ppi->ibf_a) val |=  (1 << 5);
        else            val &= ~(1 << 5);

        /* PC3 = INTR_A */
        if (ppi->intr_a) val |=  (1 << 3);
        else             val &= ~(1 << 3);

        /* PC4 = INTE_A (flip-flop) */
        if (!ppi->intr_a) val |=  (1 << 4);
        else             val &= ~(1 << 4);
    } else if (ppi->mode_a == PPI_MODE1_OUTPUT)
	{
		/* PC5 = IBF_A (input buffer full) */
        if (ppi->obf_a) val |=  (1 << 7);
        else            val &= ~(1 << 7);

        /* PC3 = INTR_A */
        if (ppi->intr_a) val |=  (1 << 3);
        else             val &= ~(1 << 3);

        /* PC4 = INTE_A (flip-flop) */
        if (!ppi->intr_a) val |=  (1 << 6);
        else             val &= ~(1 << 6);
		
	}

    /* port B in mode 1? */
    if (ppi->mode_b == PPI_MODE1_INPUT) {
        /* PC1 = IBF_B */
        if (!ppi->ibf_b) val |=  (1 << 1);
        else            val &= ~(1 << 1);

        /* PC0 = INTR_B */
        if (ppi->intr_b) val |=  (1 << 0);
        else             val &= ~(1 << 0);

        /* PC2 = INTE_B */
        if (!ppi->intr_b) val |=  (1 << 2);
        else             val &= ~(1 << 2);
    } else if (ppi->mode_b == PPI_MODE1_OUTPUT)
	{
		        /* PC1 = IBF_B */
        if (ppi->obf_b) val |=  (1 << 1);
        else            val &= ~(1 << 1);

        /* PC0 = INTR_B */
        if (ppi->intr_b) val |=  (1 << 0);
        else             val &= ~(1 << 0);

        /* PC2 = INTE_B */
        if (!ppi->intr_b) val |=  (1 << 2);
        else             val &= ~(1 << 2);
		
		
	}
	//ppi->port_latches[2] = val;
    return val;
}

static ppi_access_t ppi_check_port_mode(ppi8255_t *ppi, int port, ppi8255_op_t op)
{
    switch (port) {
        case 0: // Port A
            switch (ppi->mode_a) {
                case PPI_MODE0:
                    // Always legal (simple I/O)
                    return PPI_ACCESS_OK;

                case PPI_MODE1_INPUT:
                    if (op == PPI_OP_WRITE) {
                        printf("Illegal: write to Port A while in Mode 1 input\n");
                        return PPI_ACCESS_ILLEGAL;
                    }
					if (op == PPI_OP_READ) {
						check_intr(ppi, 0, 1);
                    }
                    return PPI_ACCESS_OK;

                case PPI_MODE1_OUTPUT:
                    if (op == PPI_OP_READ) {
                        printf("Warning: read from Port A in Mode 1 output returns latch only\n");
                    }
					if (op == PPI_OP_WRITE) {
						check_intr(ppi, 0, 2);
						return PPI_ACCESS_OK;
					}
					
                    return PPI_ACCESS_WARN;

                case PPI_MODE2:
                    if (op == PPI_OP_WRITE && ppi->obf_a) {
                        printf("Illegal: write to Port A ignored while OBF=1 in Mode 2\n");
                        return PPI_ACCESS_ILLEGAL;
                    }
                    if (op == PPI_OP_READ && !ppi->ibf_a) {
                        printf("Illegal: read from Port A ignored while IBF=0 in Mode 2\n");
                        return PPI_ACCESS_ILLEGAL;
                    }
                    return PPI_ACCESS_OK;
            }
            break;

        case 1: // Port B
            switch (ppi->mode_b) {
                case PPI_MODE0:
                    return PPI_ACCESS_OK;

                case PPI_MODE1_INPUT:
                    if (op == PPI_OP_WRITE) {
                        printf("Illegal: write to Port B while in Mode 1 input\n");
                        return PPI_ACCESS_ILLEGAL;
                    }
					if (op == PPI_OP_READ) {
						check_intr(ppi, 1, 1);
                    }
                    return PPI_ACCESS_OK;

                case PPI_MODE1_OUTPUT:
                    if (op == PPI_OP_READ) {
                        printf("Warning: read from Port B in Mode 1 output returns latch only\n");
                    }
					if (op == PPI_OP_WRITE) {
						check_intr(ppi, 1, 2);
						return PPI_ACCESS_OK;
					}
                    return PPI_ACCESS_WARN;

                default:
                    return PPI_ACCESS_UNKNOWN;
            }
            break;

        case 2: // Port C
            // Upper/lower halves may differ
            if (op == PPI_OP_WRITE && ppi->port_c_upper_in && ppi->port_c_lower_in) {
                printf("Illegal: write to Port C while configured as full input\n");
                return PPI_ACCESS_ILLEGAL;
            }
            // Reads are always legal (returns latch or pin values depending on config)
            return PPI_ACCESS_OK;

        default:
            printf("Invalid port %d\n", port);
            return PPI_ACCESS_UNKNOWN;
    }
    return PPI_ACCESS_UNKNOWN;
}

static ppi8255_control_t ppi_decode_control(ppi8255_t *ppi, uint8_t control_val)
{
	uint8_t bsr_set = 0x01;
	uint8_t mode_flag = 0x80;
	if ((control_val & mode_flag) == 0)
	{
		int sel_bit = (control_val >> 1) & 0x07; // D3-D1 select bit 0-7
		int set_bit = control_val & bsr_set;
		if (set_bit){
                ppi->port_latches[2] |=  (1 << sel_bit); // set bit
				ppi->port_latches[2] |= ((1 << 6));
				if(sel_bit == 2)
				{
					ppi->inte_b = 1;
				}
				if(sel_bit == 6)
				{
					ppi->inte_a = 1;
				}
				if(sel_bit == 4)
				{
					ppi->inte_a = 1;
				}
		}else{
                ppi->port_latches[2] &= ~(1 << sel_bit); // reset bit
				if(sel_bit == 2)
				{
					ppi->inte_b = 0;
				}
				if(sel_bit == 6)
				{
					ppi->inte_a = 0;
				}
				if(sel_bit == 4)
				{
					ppi->inte_a = 0;
				}
		}
		return PPI_BSR;
	} else {

		ppi->control = control_val;
		if (ppi->mode_a == PPI_MODE1_OUTPUT)
		{
			ppi->port_c_lower_in = (control_val >> 3) & 0x01;
		} else {
			ppi->port_c_lower_in = (control_val >> 0) & 0x01;
		}
		
		ppi->port_b_in       = (control_val >> 1) & 0x01;
		uint8_t mode_b_bit   = (control_val >> 2) & 0x01;
		ppi->port_c_upper_in = (control_val >> 3) & 0x01;
		ppi->port_a_in       = (control_val >> 4) & 0x01;
		uint8_t mode_a_bits  = (control_val >> 5) & 0x03;

		if (mode_a_bits == 0x00) {
			ppi->mode_a = PPI_MODE0;
		} else if (mode_a_bits == 0x01) {
			ppi->mode_a = ppi->port_a_in ? PPI_MODE1_INPUT : PPI_MODE1_OUTPUT;
		} else {
			ppi->mode_a = PPI_MODE2;
		}

		if (mode_b_bit) {
			ppi->mode_b = ppi->port_b_in ? PPI_MODE1_INPUT : PPI_MODE1_OUTPUT;
		} else {
			ppi->mode_b = PPI_MODE0;
		}

		return PPI_STD;
	}
	
	return PPI_STD;
}
void ppi_strobe(ppi8255_t *ppi, uint8_t group)
{
	group = group & 0x01;
    switch (group) {
        case 0: // Port A
            switch (ppi->mode_a) {
                case PPI_MODE0:
                    // Always legal (simple I/O)
                    break;

                case PPI_MODE1_INPUT:
					if (!ppi->ibf_a){
						ppi->ibf_a = true;
						
						//if (ppi->cb.ibf_changed) ppi->cb.ibf_changed(0, 1, ppi->user);
						check_intr(ppi, 0, 0);
						ppi->port_latches[0] = ppi->in_port[0];
					}
					break;

                case PPI_MODE1_OUTPUT:
                    break;

                case PPI_MODE2:
					break;
            }
            break;

        case 1: // Port B
            switch (ppi->mode_b) {
                case PPI_MODE0:
                    break;

                case PPI_MODE1_INPUT:
					if (!ppi->ibf_b){
						ppi->ibf_b = true;
						//if (ppi->cb.ibf_changed) ppi->cb.ibf_changed(1, 1, ppi->user);
						check_intr(ppi, 1, 0);
						ppi->port_latches[1] = ppi->in_port[1];
					}
					break;
                case PPI_MODE1_OUTPUT:
                    break;


                default:
                    break;
            }
            break;
			
	}
}

uint8_t ppi_read(ppi8255_t *ppi, uint8_t port)
{	
	uint8_t iport = port & 0x03;
	switch (iport)
	{
		case 0x00:
		case 0x01:
		{
			switch (ppi_check_port_mode(ppi, iport, PPI_OP_READ))
			{
				case PPI_ACCESS_OK:
					return ppi->in_port[iport];
				case PPI_ACCESS_WARN:
					// both are valid, just for verbosity
					return ppi->port_latches[iport];
				case PPI_ACCESS_ILLEGAL:
				case PPI_ACCESS_UNKNOWN:
					// invalid, but shouldn't happen...
					return 0xFF;
					
				default:
					return 0xFF;
					
			
			}
			
		}
		case 0x02:
		uint8_t val = ppi8255_portc_read(ppi);
		switch (ppi_check_port_mode(ppi, iport, PPI_OP_READ))
			{
				case PPI_ACCESS_OK:
				case PPI_ACCESS_WARN:
					// both are valid, just for verbosity
					ppi->port_latches[2] = val;
					return val;
				case PPI_ACCESS_ILLEGAL:
				case PPI_ACCESS_UNKNOWN:
					// invalid, but shouldn't happen...
					return 0xFF;
					
				default:
					return 0xFF;
					
			
			}
			
		case 0x03:
		{
			// invalid read
			return 0xFF;
		}
	}
	return 0xFF;

}

void ppi_write(ppi8255_t *ppi, uint8_t port, uint8_t data)
{	
	uint8_t iport = port & 0x03;


	switch (iport)
	{
		case 0x00:
		case 0x01:
			ppi->port_latches[iport] = data;
		case 0x02:
		{
			
			switch (ppi_check_port_mode(ppi, iport, PPI_OP_WRITE))
			{
				case PPI_ACCESS_OK:
				case PPI_ACCESS_WARN:
					// both are valid, just for verbosity - for now, should be *ok*
					ppi->out_port[iport] = data;


					break;
				case PPI_ACCESS_ILLEGAL:
				case PPI_ACCESS_UNKNOWN:
					// invalid, but shouldn't happen...
					break;
					
				default:
					break;
					
			
			}
			break;
			
		}
		case 0x03:
		{
			ppi_decode_control(ppi, data);
				printf("[DEBUG] ppi_write: port=%d data=0x%02X mode_a=%d mode_b=%d\n",
       iport, data, ppi->mode_a, ppi->mode_b);
			break;
		}
	}

}

void ppi_init(ppi8255_t *ppi, ppi8255_callbacks_t *cb, void *user)
{
    memset(ppi, 0, sizeof(*ppi));  // clears everything, avoids garbage

    ppi->control = 0x9B; // default control word
    ppi->mode_a  = PPI_MODE0;
    ppi->mode_b  = PPI_MODE0;
    ppi->port_a_in = ppi->port_b_in = true;
    ppi->port_c_upper_in = ppi->port_c_lower_in = true;

    if (cb) {
        ppi->cb = *cb;
    } else {
        memset(&ppi->cb, 0, sizeof(ppi->cb));
    }
    ppi->user = user;
}

