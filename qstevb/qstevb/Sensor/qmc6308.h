#ifndef __QMC6308_H
#define __QMC6308_H

#define QMC6308_OK			1
#define QMC6308_FAIL		0
/* vendor chip id*/
#define QMC6308_IIC_ADDR				0x2c		// 6308: (0x2c<<1) 6310: (0x1c<<1)
#define QMC6310_IIC_ADDR_U				0x1c		// 6310U
#define QMC6310_IIC_ADDR_N				0x3c		// 6310N

#define QMC6308_CHIP_ID_REG				0x00

/*data output register*/
#define QMC6308_DATA_OUT_X_LSB_REG		0x01
#define QMC6308_DATA_OUT_X_MSB_REG		0x02
#define QMC6308_DATA_OUT_Y_LSB_REG		0x03
#define QMC6308_DATA_OUT_Y_MSB_REG		0x04
#define QMC6308_DATA_OUT_Z_LSB_REG		0x05
#define QMC6308_DATA_OUT_Z_MSB_REG		0x06
/*Status registers */
#define QMC6308_STATUS_REG				0x09
/* configuration registers */
#define QMC6308_CTL_REG_ONE				0x0A  /* Contrl register one */
#define QMC6308_CTL_REG_TWO				0x0B  /* Contrl register two */
#define QMC6308_CTL_REG_THREE			0x0D  /* Contrl register three */


#define QMC6308_SET_RESET_ON 			0
#define QMC6308_SET_ON 					1
#define QMC6308_SET_RESET_OFF 			2

#define QMC6308_RNG_30G					0
#define QMC6308_RNG_12G					1
#define QMC6308_RNG_8G					2
#define QMC6308_RNG_2G					3

#define QMC6308_MODE_SUSPEND			0
#define QMC6308_MODE_NORMAL				1
#define QMC6308_MODE_SINGLE				2
#define QMC6308_MODE_HPF				3

#define QMC6308_ODR_HPFM				0
#define QMC6308_ODR_10HZ				0
#define QMC6308_ODR_50HZ				1
#define QMC6308_ODR_100HZ				2
#define QMC6308_ODR_200HZ				3

#define QMC6308_OSR1_8					0
#define QMC6308_OSR1_4					1
#define QMC6308_OSR1_2					2
#define QMC6308_OSR1_1					3

#define QMC6308_OSR2_1					0
#define QMC6308_OSR2_2					1
#define QMC6308_OSR2_4					2
#define QMC6308_OSR2_8					3

#define QMC6308_SELFTEST_MAX_X			(1800)
#define QMC6308_SELFTEST_MIN_X			(120)
#define QMC6308_SELFTEST_MAX_Y			(1800)
#define QMC6308_SELFTEST_MIN_Y			(120)
#define QMC6308_SELFTEST_MAX_Z			(1800)
#define QMC6308_SELFTEST_MIN_Z			(120)

#define QMC6308_ABS(X) 					((X) < 0.0f ? (-1 * (X)) : (X))
#define QMC6308_ABSF(X) 				((X) < 0.0f ? (-1.0 * (X)) : (X))
//#define QMC6308_FILTER_SUPPORT
//#define QMC6308_MODE_SWITCH

enum
{
	QMC_NONE = 0,
	QMC_6308,
	QMC_6308_MPW,
	QMC_6310,

	QMC_TOTAL
};

//enum
//{
//	AXIS_X = 0,
//	AXIS_Y = 1,
//	AXIS_Z = 2,

//	AXIS_TOTAL
//};

typedef struct
{
	char			mode;
	unsigned short	count;
} qmc6308_set_ctl_t;


#define AVG_FILTER_SIZE		4
typedef struct
{
	char	init;
	int		array[AVG_FILTER_SIZE];
	//float	avg;
} QmcAvgFilter_t;


typedef union
{
	struct
	{
		unsigned char mode	:	2;
		unsigned char odr	:	2;
		unsigned char osr1	:	2;
		unsigned char osr2	:	2;
	}bit;
	unsigned char value;
} ctrl_reg1;

typedef union
{
	struct
	{
		unsigned char setrst	:	2;
		unsigned char range		:	2;
		unsigned char rev		:	2;
		unsigned char selftest	:	1;
		unsigned char softrst	:	1;
	}bit;
	unsigned char value;
} ctrl_reg2;

typedef struct
{
	unsigned char slave_addr;
	//unsigned char chip_id;
	unsigned char chip_type;
	unsigned short ssvt;
	float 		last_data[3];
	ctrl_reg1	ctrl1;
	ctrl_reg2	ctrl2;
#if defined(QMC6308_MODE_SWITCH)
	qmc6308_set_ctl_t	set_ctl;
#endif
}qmc6308_data_t;


#endif

