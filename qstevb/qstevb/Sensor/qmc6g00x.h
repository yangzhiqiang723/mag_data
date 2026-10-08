#ifndef __QMC6G00X_H
#define __QMC6G00X_H


#define QMC6G00X_RECOVER

#define QMC6G00X_OK						1
#define QMC6G00X_FAIL					0

#define QMC6G00X_IIC_ADDR				0x0c

/* chip id*/
#define QMC6G00V_CHIP_ID				0x10
#define QMC6G00H_CHIP_ID				0x20
/*data output register*/
#define QMC6G00X_CHIP_ID_REG			0x00
#define QMC6G00X_DATA_OUT_X_LSB_REG		0x01
#define QMC6G00X_DATA_OUT_X_MSB_REG		0x02
#define QMC6G00X_DATA_OUT_Y_LSB_REG		0x03
#define QMC6G00X_DATA_OUT_Y_MSB_REG		0x04
#define QMC6G00X_DATA_OUT_Z_LSB_REG		0x05
#define QMC6G00X_DATA_OUT_Z_MSB_REG		0x06
#define QMC6G00X_TEMP_OUT_LSB_REG		0x07
#define QMC6G00X_TEMP_OUT_MSB_REG		0x08

#define QMC6G00X_DATA_OUT_ST_X			0x13

/*Status registers */
#define QMC6G00X_STATUS_REG				0x09
/* configuration registers */
#define QMC6G00X_CTL_REG_ONE			0x0A  /* Contrl register one */
#define QMC6G00X_CTL_REG_TWO			0x0B  /* Contrl register two */
#define QMC6G00X_CTL_REG_PSET			0x0C
#define QMC6G00X_CTL_REG_SELFTEST		0x0E
#define QMC6G00X_CTL_IBI				0x21

#define QMC6G00X_FIFO_REG_STATUS		0x20
#define QMC6G00X_FIFO_REG_CTRL			0x2E
#define QMC6G00X_FIFO_REG_DATA			0x2F

#define QMC6G00X_DATA_OFFSET_X_LSB_REG	0x22

// maestro1 set/reset
#define QMC6G00X_SET_RESET_OFF 			0
#define QMC6G00X_SET_ON 				1
#define QMC6G00X_RESET_ON 				3

#define QMC6G00X_RNG_20G				0

#define QMC6G00X_MODE_SUSPEND			0
#define QMC6G00X_MODE_NORMAL			1
#define QMC6G00X_MODE_SINGLE			2
#define QMC6G00X_MODE_HPFM				3

// maestro1 odr
#define QMC6G00X_ODR_HPF				0
#define QMC6G00X_ODR_1HZ				0
#define QMC6G00X_ODR_10HZ				1
#define QMC6G00X_ODR_20HZ				2
#define QMC6G00X_ODR_50HZ				3
#define QMC6G00X_ODR_100HZ				4
#define QMC6G00X_ODR_200HZ				5
#define QMC6G00X_ODR_400HZ				6
#define QMC6G00X_ODR_1000HZ				7

#define QMC6G00X_OSR1_1					1
#define QMC6G00X_OSR1_2					0
#define QMC6G00X_OSR1_4					4
#define QMC6G00X_OSR1_8					5
#define QMC6G00X_OSR1_16				6
#define QMC6G00X_OSR1_32				7

// Moving average depth normal mode the maestro0:depth:16/8/4/2/1(7~4/3/2/1/0) maestro1:depth:8/4/2/1(3/2/1/0). Single Mode also use this filter.
#define QMC6G00X_OSR2_1					0
#define QMC6G00X_OSR2_2					1
#define QMC6G00X_OSR2_4					2
#define QMC6G00X_OSR2_8					3
//#define QMC6G00X_OSR2_16				4

#define QMC6G00X_STATUS_DRDY			0x01
#define QMC6G00X_STATUS_OVFL			0x02
#define QMC6G00X_STATUS_ST_RDY			0x04
#define QMC6G00X_STATUS_NVM_RDY			0x08
#define QMC6G00X_STATUS_NVM_LOAD_DONE	0x10

#define QMC6G00X_FIFO_STATUS_OR			0x04
#define QMC6G00X_FIFO_STATUS_WMK		0x02
#define QMC6G00X_FIFO_STATUS_FULL		0x01

#define QMC6G00X_SELFTEST_MAX_X			(50)
#define QMC6G00X_SELFTEST_MIN_X			(1)
#define QMC6G00X_SELFTEST_MAX_Y			(50)
#define QMC6G00X_SELFTEST_MIN_Y			(1)
#define QMC6G00X_SELFTEST_MAX_Z			(50)
#define QMC6G00X_SELFTEST_MIN_Z			(1)

#define QMC6G00X_ABS(X) 				((X) < 0 ? (-1 * (X)) : (X))
#define QMC6G00X_ABSF(X) 				((X) < 0.0f ? (-1.0 * (X)) : (X))
#define QMC6G00X_MIN(x, y)				((x) < (y) ? (x) : (y))
#define QMC6G00X_MAX(x, y)				((x) > (y) ? (x) : (y))


//#define QMC6G00X_MODE_SWITCH
#define QMC6G00X_TCO

typedef enum
{
	QMC6G00X_FIFO_MODE_BYPASS = (0<<6),
	QMC6G00X_FIFO_MODE_FIFO = (1<<6),
	QMC6G00X_FIFO_MODE_STREAM = (2<<6),
	QMC6G00X_FIFO_MODE_DEFAULT = (3<<6)
} qmc6g00x_fifo_mode;

typedef enum
{
	QMC6G00X_IBI_OFF = 0x00,
	QMC6G00X_IBI_DRDY = 0x01,
	QMC6G00X_IBI_OVFL = 0x02,
	QMC6G00X_IBI_ST_RDY = 0x04,
	QMC6G00X_IBI_FIFO_FULL = 0x08,
	QMC6G00X_IBI_FIFO_WMK = 0x10,
	QMC6G00X_IBI_EN = 0x20
} qmc6g00x_fifo_ibi;

typedef struct
{
	char			mode;
	unsigned short	count;
} qmc6g00x_sr_ctl_t;

typedef union
{
	struct
	{
		unsigned char mode:2;
		unsigned char osr1:3;
		unsigned char osr2:2;
		unsigned char rev:1;
	}bit;
	unsigned char value;
} qmc6g00x_ctrla;

typedef union
{
	struct
	{
		unsigned char set_rst:2;
		unsigned char range:2;
		unsigned char odr:3;
		unsigned char soft_rst:1;
	}bit;
	unsigned char value;
} qmc6g00x_ctrlb;

typedef union
{
	struct
	{
		unsigned char period_set:6;
		unsigned char user_set:1;
		unsigned char rev:1;
	}bit;
	unsigned char value;
} qmc6g00x_pset;

typedef union
{
	struct
	{
		signed short kx:7;	
		signed short ky:9;
	}bit;
	unsigned short value;
} qmc6g00x_kxky;

typedef union
{
	struct
	{
		signed short ky:9;
		signed short rev:7;
	}bit;
	unsigned short value;
} qmc6g00x_ky;


typedef struct
{
	unsigned char			protocol;
	unsigned char			slave_addr;
	unsigned char			chipid;
	unsigned short			ssvt;
	short					last_data[4];

	unsigned char			ctl1_val;
	unsigned char			ctl2_val;
	unsigned char			fifo_ctrl;
	//float					kx;
	//float					ky;
	unsigned short			fail_num;
#if defined(QMC6G00X_TCO)
	float					t_data;
#endif
#if defined(QMC6G00X_MODE_SWITCH)
	qmc6g00x_sr_ctl_t		set_ctl;
#endif
} qmc6g00x_data_t;

#endif

