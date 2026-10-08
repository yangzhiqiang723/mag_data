
#ifndef _MAG_HUB_H_
#define _MAG_HUB_H_

#include "usb_device.h"

typedef struct
{
	unsigned char	mode;
	unsigned char	odr;
	unsigned short	delay;
} mag_hub_odr_t;

typedef struct
{
	unsigned char	slave;
	unsigned char	chipid;
	int	sel_rng;
	int	sel_odr;
	int	sel_osr1;
	int	sel_osr2;
	int	sel_sr;
	int	sel_fifo_mode;
	int	sel_fifo_wmk;
}mag_hub_t;

typedef struct
{
	short	delay;
	short	mag_lsb;

	unsigned int data_i;
	unsigned int data_fail;
	short		raw[3];
}mag_hub_data_t;


unsigned short mag_hub_set(int rng, int odr, int osr1, int osr2, int sr, int fifo_mode, int fifo_wmk);
void mag_hub_read_info(unsigned char* chip_id, unsigned char* v_id, unsigned char* w_id, unsigned short* d_id, unsigned short* l_id);
void mag_hub_enable(void);
void mag_hub_disable(void);
void mag_hub_read_data(float *out);
int mag_hub_do_selftest(int out[3]);

#endif

