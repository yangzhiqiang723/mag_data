
// mag_hub.cpp : 定义应用程序的类行为。
//
#include "stdafx.h"
#if defined(QST_USE_DEVICE)
#include "usb_device.h"
#include "qst_sensor_id.h"
#include "mag_hub.h"
#include "qmc6308.h"
#include "qmc6309.h"
#include "qmc6309v.h"
#include "qmc6g00x.h"

#define MAG_HUB_LOG		_cprintf

#define SLAVE_2C	0x2c
#define SLAVE_0C	0x0c

#define HPF_DELAY			5
#define N200_DELAY			6
#define N100_DELAY			11

#define N50_DELAY			(N100_DELAY*2)
#define N20_DELAY			(N100_DELAY*5)
#define N10_DELAY			(N100_DELAY*10)
#define N1_DELAY			(N100_DELAY*100)
#define SINGLE_DELAY		20

const mag_hub_odr_t qmc6309_odr[] =
{
	{QMC6309_MODE_HPFM, 	QMC6309_ODR_HPFM, 	HPF_DELAY		},
	{QMC6309_MODE_NORMAL, 	QMC6309_ODR_200HZ, 	N200_DELAY		},
	{QMC6309_MODE_NORMAL,	QMC6309_ODR_100HZ,	N100_DELAY		},
	{QMC6309_MODE_NORMAL, 	QMC6309_ODR_50HZ, 	N50_DELAY		},
	{QMC6309_MODE_NORMAL, 	QMC6309_ODR_10HZ, 	N10_DELAY		},
	{QMC6309_MODE_NORMAL, 	QMC6309_ODR_1HZ,	N1_DELAY		},
	{QMC6309_MODE_SINGLE,	QMC6309_ODR_HPFM,	SINGLE_DELAY	},
	{QMC6309_MODE_SUSPEND,	QMC6309_ODR_HPFM,	0,	}
};

const mag_hub_odr_t qmc6309v_odr[] =
{
	{QMC6309V_MODE_HPFM, 	QMC6309V_ODR_HPFM, 	HPF_DELAY		},
	{QMC6309V_MODE_NORMAL, 	QMC6309V_ODR_200HZ, 	N200_DELAY		},
	{QMC6309V_MODE_NORMAL,	QMC6309V_ODR_100HZ,	N100_DELAY		},
	{QMC6309V_MODE_NORMAL, 	QMC6309V_ODR_50HZ, 	N50_DELAY		},
	{QMC6309V_MODE_NORMAL, 	QMC6309V_ODR_10HZ, 	N10_DELAY		},
	{QMC6309V_MODE_NORMAL, 	QMC6309V_ODR_1HZ,	N1_DELAY		},
	{QMC6309V_MODE_SINGLE,	QMC6309V_ODR_HPFM,	SINGLE_DELAY	},
	{QMC6309V_MODE_SUSPEND,	QMC6309V_ODR_HPFM,	0,	}
};

const mag_hub_odr_t qmc6g00x_odr[] =
{
	{QMC6G00X_MODE_HPFM, 	QMC6G00X_ODR_HPF,		HPF_DELAY		},
	{QMC6G00X_MODE_NORMAL, 	QMC6G00X_ODR_1000HZ, 	HPF_DELAY		},
	{QMC6G00X_MODE_NORMAL, 	QMC6G00X_ODR_400HZ, 	HPF_DELAY		},
	{QMC6G00X_MODE_NORMAL, 	QMC6G00X_ODR_200HZ, 	N200_DELAY		},
	{QMC6G00X_MODE_NORMAL,	QMC6G00X_ODR_100HZ,		N100_DELAY		},
	{QMC6G00X_MODE_NORMAL, 	QMC6G00X_ODR_50HZ, 		N50_DELAY		},
	{QMC6G00X_MODE_NORMAL, 	QMC6G00X_ODR_20HZ, 		N20_DELAY		},
	{QMC6G00X_MODE_NORMAL, 	QMC6G00X_ODR_10HZ, 		N10_DELAY		},
	{QMC6G00X_MODE_NORMAL, 	QMC6G00X_ODR_1HZ,		N1_DELAY		},
	{QMC6G00X_MODE_SINGLE,	QMC6G00X_ODR_HPF,		SINGLE_DELAY	},
	{QMC6G00X_MODE_SUSPEND,	QMC6G00X_ODR_HPF,		0	}
};

const mag_hub_odr_t qmc6308_odr[] =
{
	{QMC6308_MODE_HPF, 		QMC6308_ODR_HPFM, 	HPF_DELAY		},
	{QMC6308_MODE_NORMAL, 	QMC6308_ODR_200HZ, 	N200_DELAY		},
	{QMC6308_MODE_NORMAL,	QMC6308_ODR_100HZ,	N100_DELAY		},
	{QMC6308_MODE_NORMAL, 	QMC6308_ODR_50HZ, 	N50_DELAY		},
	{QMC6308_MODE_NORMAL, 	QMC6308_ODR_10HZ, 	N10_DELAY		},
	{QMC6308_MODE_SINGLE,	QMC6308_ODR_HPFM,	SINGLE_DELAY	},
	{QMC6308_MODE_SUSPEND,	QMC6308_ODR_HPFM,	0,	}
};


const unsigned char qmc6309_range[] = { QMC6309_RNG_32G };	//{QMC6309_RNG_32G, QMC6309_RNG_16G, QMC6309_RNG_8G};
const unsigned char qmc6309_osr1[] = { QMC6309_OSR1_1, QMC6309_OSR1_2, QMC6309_OSR1_4, QMC6309_OSR1_8 };
const unsigned char qmc6309_osr2[] = { QMC6309_OSR2_1, QMC6309_OSR2_2, QMC6309_OSR2_4, QMC6309_OSR2_8, QMC6309_OSR2_16 };
const unsigned char qmc6309_sr[] = { QMC6309_SET_RESET_ON, QMC6309_SET_ON, QMC6309_RESET_ON, QMC6309_SET_RESET_OFF };
const unsigned char qmc6309_fifomode[] = {0, 0x40, 0x80 };

const unsigned char qmc6309v_range[] = { QMC6309V_RNG_32G };	//{QMC6309V_RNG_32G, QMC6309V_RNG_16G, QMC6309V_RNG_8G};
const unsigned char qmc6309v_osr1[] = { QMC6309V_OSR1_1, QMC6309V_OSR1_2, QMC6309V_OSR1_4, QMC6309V_OSR1_8 };
const unsigned char qmc6309v_osr2[] = { QMC6309V_OSR2_1, QMC6309V_OSR2_2, QMC6309V_OSR2_4, QMC6309V_OSR2_8 };
const unsigned char qmc6309v_sr[] = { QMC6309V_SET_RESET_ON, QMC6309V_SET_ON, QMC6309V_RESET_ON, QMC6309V_SET_RESET_OFF };
const unsigned char qmc6309v_fifomode[] = { 0, 0x40, 0x80 };

const unsigned char qmc6g00x_range[] = { QMC6G00X_RNG_20G };
const unsigned char qmc6g00x_osr1[] = { QMC6G00X_OSR1_1, QMC6G00X_OSR1_2, QMC6G00X_OSR1_4, QMC6G00X_OSR1_8
#if defined(QMC6G00X_OSR1_EXT)
										, QMC6G00X_OSR1_16, QMC6G00X_OSR1_32 
#endif
};
const unsigned char qmc6g00x_osr2[] = { QMC6G00X_OSR2_1, QMC6G00X_OSR2_2, QMC6G00X_OSR2_4, QMC6G00X_OSR2_8 };
const unsigned char qmc6g00x_sr[] = { QMC6G00X_SET_RESET_OFF, QMC6G00X_SET_ON, QMC6G00X_RESET_ON };
const unsigned char qmc6g00x_fifomode[] = { 0, 0x40, 0x80 };

const unsigned char qmc6308_range[] = { QMC6308_RNG_30G, QMC6308_RNG_12G, QMC6308_RNG_8G, QMC6308_RNG_2G };
const unsigned char qmc6308_osr1[] = { QMC6308_OSR1_1, QMC6308_OSR1_2, QMC6308_OSR1_4, QMC6308_OSR1_8 };
const unsigned char qmc6308_osr2[] = { QMC6308_OSR2_1, QMC6308_OSR2_2, QMC6308_OSR2_4, QMC6308_OSR2_8 };
const unsigned char qmc6308_sr[] = { QMC6308_SET_RESET_ON, QMC6308_SET_ON, QMC6308_SET_RESET_OFF };

static mag_hub_t m_cfg;
static mag_hub_data_t m_out;

void mag_hub_enable_qmc6309(mag_hub_t* cfg)
{
	qmc6309_ctrlreg1	ctrl1;
	qmc6309_ctrlreg2	ctrl2;
	unsigned char ctl_fifo = 0x00;

	ctrl1.bit.mode = qmc6309_odr[cfg->sel_odr].mode;
	ctrl1.bit.osr1 = qmc6309_osr1[cfg->sel_osr1];
	ctrl1.bit.osr2 = qmc6309_osr2[cfg->sel_osr2];
	ctrl1.bit.zdbl_enb = QMC6309H_ZDBL_ENB_OFF;

	ctrl2.bit.set_rst = qmc6309_sr[cfg->sel_sr];
	ctrl2.bit.range = qmc6309_range[cfg->sel_rng];
	ctrl2.bit.odr = qmc6309_odr[cfg->sel_odr].odr;
	ctrl2.bit.soft_rst = 0;

	i2c_write_reg(m_cfg.slave, QMC6309_FIFO_REG_CTRL, 0x00);
	i2c_write_reg(m_cfg.slave, QMC6309_CTL_REG_ONE, 0x00);
	Sleep(2);
	i2c_write_reg(m_cfg.slave, QMC6309_CTL_REG_TWO, ctrl2.value);
	i2c_write_reg(m_cfg.slave, QMC6309_CTL_REG_ONE, ctrl1.value);
	if ((cfg->sel_fifo_mode > 0) && (cfg->sel_fifo_wmk > 0))
	{
		ctl_fifo = (qmc6309_fifomode[cfg->sel_fifo_mode] | (cfg->sel_fifo_wmk << 3) | QMC6309_FIFO_CH_ALL);
		i2c_write_reg(m_cfg.slave, QMC6309_FIFO_REG_CTRL, ctl_fifo);
	}
}

void mag_hub_enable_qmc6309v(mag_hub_t* cfg)
{
	qmc6309v_ctrlreg1	ctrl1;
	qmc6309v_ctrlreg2	ctrl2;
	qmc6309v_ctrlreg3	ctrl3;
	unsigned char ctl_fifo = 0x00;

	ctrl1.bit.mode = qmc6309v_odr[cfg->sel_odr].mode;
	ctrl1.bit.osr1 = qmc6309v_osr1[cfg->sel_osr1];
	ctrl1.bit.osr2 = qmc6309v_osr2[cfg->sel_osr2];
	ctrl1.bit.rev = 0;
	ctrl1.bit.zdbl_enb = QMC6309V_ZDBL_ENB_OFF;

	ctrl2.bit.set_rst = qmc6309v_sr[cfg->sel_sr];
	ctrl2.bit.range = qmc6309v_range[cfg->sel_rng];
	ctrl2.bit.odr = qmc6309v_odr[cfg->sel_odr].odr;
	ctrl2.bit.soft_rst = 0;

	i2c_read_reg(SLAVE_0C, QMC6309V_CTL_REG_THREE, &ctrl3.value, 1);
	ctrl3.bit.osr1_z = qmc6309v_osr1[cfg->sel_osr1];

	i2c_write_reg(m_cfg.slave, QMC6309V_FIFO_REG_CTRL, 0x00);
	i2c_write_reg(m_cfg.slave, QMC6309V_CTL_REG_ONE, 0x00);
	Sleep(2);
	i2c_write_reg(m_cfg.slave, QMC6309V_CTL_REG_THREE, ctrl3.value);
	i2c_write_reg(m_cfg.slave, QMC6309V_CTL_REG_TWO, ctrl2.value);
	i2c_write_reg(m_cfg.slave, QMC6309V_CTL_REG_ONE, ctrl1.value);
	if ((cfg->sel_fifo_mode > 0) && (cfg->sel_fifo_wmk > 0))
	{
		ctl_fifo = (qmc6309v_fifomode[cfg->sel_fifo_mode] | cfg->sel_fifo_wmk);
		i2c_write_reg(m_cfg.slave, QMC6309V_FIFO_REG_CTRL, ctl_fifo);
	}
}

void mag_hub_enable_qmc6g00x(mag_hub_t* cfg)
{
	qmc6g00x_ctrla	ctrl1;
	qmc6g00x_ctrlb	ctrl2;
	unsigned char ctl_fifo = 0x00;

	ctrl1.bit.rev = 0;
	ctrl1.bit.mode = qmc6g00x_odr[cfg->sel_odr].mode;
	ctrl1.bit.osr1 = qmc6g00x_osr1[cfg->sel_osr1];
	ctrl1.bit.osr2 = qmc6g00x_osr2[cfg->sel_osr2];

	ctrl2.bit.set_rst = qmc6g00x_sr[cfg->sel_sr];
	ctrl2.bit.range = qmc6g00x_range[cfg->sel_rng];
	ctrl2.bit.odr = qmc6g00x_odr[cfg->sel_odr].odr;
	ctrl2.bit.soft_rst = 0;

	i2c_write_reg(m_cfg.slave, QMC6G00X_FIFO_REG_CTRL, 0x00);
	i2c_write_reg(m_cfg.slave, QMC6G00X_CTL_REG_ONE, 0x00);
	Sleep(2);
	i2c_write_reg(m_cfg.slave, QMC6G00X_CTL_REG_TWO, ctrl2.value);
	i2c_write_reg(m_cfg.slave, QMC6G00X_CTL_REG_ONE, ctrl1.value);
	if ((cfg->sel_fifo_mode > 0) && (cfg->sel_fifo_wmk > 0))
	{
		ctl_fifo = (qmc6g00x_fifomode[cfg->sel_fifo_mode] | cfg->sel_fifo_wmk);
		i2c_write_reg(m_cfg.slave, QMC6309_FIFO_REG_CTRL, ctl_fifo);
	}
}

void mag_hub_enable_qmc6308(mag_hub_t* cfg)
{
	ctrl_reg1 ctrl1;
	ctrl_reg2 ctrl2;

	ctrl1.bit.mode = qmc6308_odr[cfg->sel_odr].mode;
	ctrl1.bit.odr = qmc6308_odr[cfg->sel_odr].odr;
	ctrl1.bit.osr1 = qmc6308_osr1[cfg->sel_osr1];
	ctrl1.bit.osr2 = qmc6308_osr2[cfg->sel_osr2];

	ctrl2.bit.setrst = qmc6308_sr[cfg->sel_sr];
	ctrl2.bit.range = qmc6308_range[cfg->sel_rng];
	ctrl2.bit.rev = 0;
	ctrl2.bit.selftest = 0;
	ctrl2.bit.softrst = 0;

	i2c_write_reg(m_cfg.slave, QMC6308_CTL_REG_ONE, 0x00);
	Sleep(2);
	i2c_write_reg(m_cfg.slave, 0x0d, 0x40);
	//i2c_write_reg(m_cfg.slave, 0x0d, 0x04);
	//i2c_write_reg(m_cfg.slave, 0x0f, 0x04);
	i2c_write_reg(m_cfg.slave, QMC6308_CTL_REG_TWO, ctrl2.value);
	i2c_write_reg(m_cfg.slave, QMC6308_CTL_REG_ONE, ctrl1.value);
}

void mag_hub_enable(void)
{
	mag_hub_t *cfg = &m_cfg;
	if ((cfg->chipid == 0x90) || (cfg->chipid == 0x92))
	{
		mag_hub_enable_qmc6309(cfg);
	}
	else if (cfg->chipid == 0x91)
	{
		mag_hub_enable_qmc6309v(cfg);
	}
	else if (cfg->chipid == 0x20)
	{
		mag_hub_enable_qmc6g00x(cfg);
	}
	else if (cfg->chipid == 0x80)
	{
		mag_hub_enable_qmc6308(cfg);
	}
}

void mag_hub_disable(void)
{
	mag_hub_t* cfg = &m_cfg;
	if ((cfg->chipid == 0x90) || (cfg->chipid == 0x92))
	{
		i2c_write_reg(m_cfg.slave, QMC6309_CTL_REG_ONE, 0x00);
	}
	else if (cfg->chipid == 0x91)
	{
		i2c_write_reg(m_cfg.slave, QMC6309V_CTL_REG_ONE, 0x00);
	}
	else if (cfg->chipid == 0x20)
	{
		i2c_write_reg(m_cfg.slave, QMC6309_CTL_REG_ONE, 0x00);
	}
	else if (cfg->chipid == 0x80)
	{
		i2c_write_reg(m_cfg.slave, QMC6308_CTL_REG_ONE, 0x00);
	}
}

void mag_hub_read_data(float *out)
{
	unsigned char buf[6];
	int ret = 0;
	int t1 = 0;
	unsigned char rdy = 0;

	ret = i2c_read_reg(m_cfg.slave, 0x09, &rdy, 1);
	while (!(rdy & 0x01) && (t1++ < 5))
	{
		Sleep(1);
		ret = i2c_read_reg(m_cfg.slave, 0x09, &rdy, 1);
	}

	if ((ret == FALSE) || (!(rdy & 0x01)))
	{
		m_out.data_fail++;
	}
	else
	{
		ret = i2c_read_reg(m_cfg.slave, 0x01, buf, 6);	// read mag
		if (ret == FALSE)
		{
			m_out.data_fail++;
		}
		else
		{
			m_out.data_i = 0;
		}
		//qst_evb_mag_enable_single();
		m_out.raw[0] = (short)(((buf[1]) << 8) | buf[0]);
		m_out.raw[1] = (short)(((buf[3]) << 8) | buf[2]);
		m_out.raw[2] = (short)(((buf[5]) << 8) | buf[4]);
	}

	out[0] = (float)(m_out.raw[0] * 100.0f / m_out.mag_lsb);
	out[1] = (float)(m_out.raw[1] * 100.0f / m_out.mag_lsb);
	out[2] = (float)(m_out.raw[2] * 100.0f / m_out.mag_lsb);
	m_out.data_i++;

	if ((m_out.data_fail * m_out.delay) > 2000)
	{
		// reset
		mag_hub_enable();
	}
}

int mag_hub_do_selftest(int out[3])
{
	int selftest_result = 0;
	int selftest_retry = 0;
	signed char  st_data[3];
	unsigned char abs_data[3];
	unsigned char rdy = 0x00;
	int t1 = 0;
	int ret = 0;

	selftest_retry = 0;
	selftest_result = 0;
	while ((selftest_result == 0) && (selftest_retry < 3))
	{
		selftest_retry++;
		ret = i2c_write_reg(m_cfg.slave, 0x0a, 0x00);
		//qst_delay_ms(2);
		if (m_cfg.chipid == 0x20)
		{
			ret = i2c_write_reg(m_cfg.slave, 0x0b, 0x01);
			ret = i2c_write_reg(m_cfg.slave, 0x0a, 0x17);
		}
		else
		{
			ret = i2c_write_reg(m_cfg.slave, 0x0b, 0x00);
			//qst_delay_ms(2);
			ret = i2c_write_reg(m_cfg.slave, 0x0a, 0x03);
		}
		Sleep(20);
		ret = i2c_write_reg(m_cfg.slave, 0x0e, 0x80);
		if (ret == FALSE)
		{
			continue;
		}

		rdy = 0x00;
		t1 = 0;
		while (!(rdy & 0x04))
		{
			Sleep(10);
			i2c_read_reg(m_cfg.slave, 0x09, &rdy, 1);
			if (t1++ > 50)
			{
				break;
			}
		}

		if (rdy & 0x04)
		{
			ret = i2c_read_reg(m_cfg.slave, 0x13, (unsigned char*)st_data, 3);
			if (ret == FALSE)
			{
				continue;
			}
		}
		else
		{
			MAG_HUB_LOG("mag selftest drdy fail[0x%02x]!\r\n", rdy);
			st_data[0] = st_data[1] = st_data[2] = 0;
		}
		abs_data[0] = QMC6309_ABS(st_data[0]);
		abs_data[1] = QMC6309_ABS(st_data[1]);
		abs_data[2] = QMC6309_ABS(st_data[2]);

		if (((abs_data[0] < QMC6309_SELFTEST_MAX_X) && (abs_data[0] > QMC6309_SELFTEST_MIN_X))
			&& ((abs_data[1] < QMC6309_SELFTEST_MAX_Y) && (abs_data[1] > QMC6309_SELFTEST_MIN_Y))
			&& ((abs_data[2] < QMC6309_SELFTEST_MAX_Z) && (abs_data[2] > QMC6309_SELFTEST_MIN_Z)))
		{
			selftest_result = 1;
		}
		else
		{
			selftest_result = 0;
		}
	}
	i2c_write_reg(m_cfg.slave, 0x0b, 0x00);
	i2c_write_reg(m_cfg.slave, 0x0a, 0x00);

	m_out.data_i++;
	//MAG_HUB_LOG("%d,%d,%d,M-%d,SELFTEST-%s\r\n", st_data[0], st_data[1], st_data[2], m_out.data_i, (selftest_result ? "PASS" : "FAIL"));
	out[0] = (int)st_data[0];
	out[1] = (int)st_data[1];
	out[2] = (int)st_data[2];

	return selftest_result;
}

unsigned short mag_hub_set(int rng, int odr, int osr1, int osr2, int sr, int fifo_mode, int fifo_wmk)
{
	m_cfg.sel_rng = rng;
	m_cfg.sel_odr = odr;
	m_cfg.sel_osr1 = osr1;
	m_cfg.sel_osr2 = osr2;
	m_cfg.sel_sr = sr;
	m_cfg.sel_fifo_mode = fifo_mode;
	m_cfg.sel_fifo_wmk = fifo_wmk;

	switch (m_cfg.chipid)
	{
	case 0x90:
	case 0x91:
	case 0x92:
		if (m_cfg.sel_rng == 0)
			m_out.mag_lsb = 1000;
		else if (m_cfg.sel_rng == 1)
			m_out.mag_lsb = 2000;
		else if (m_cfg.sel_rng == 2)
			m_out.mag_lsb = 4000;
		break;
	case 0x20:
		m_out.mag_lsb = 1000;
		break;
	case 0x80:
		if (m_cfg.sel_rng == 0)
			m_out.mag_lsb = 1000;
		else if (m_cfg.sel_rng == 1)
			m_out.mag_lsb = 2500;
		else if (m_cfg.sel_rng == 2)
			m_out.mag_lsb = 3750;
		else if (m_cfg.sel_rng == 3)
			m_out.mag_lsb = 15000;
		break;
	}
	
	return qmc6309_odr[m_cfg.sel_odr].delay;
}

void mag_hub_read_info(unsigned char *chip_id, unsigned char *v_id, unsigned char* w_id, unsigned short *d_id, unsigned short* l_id)
{
	BOOL ret = FALSE;
	unsigned char slave_array[] = { SLAVE_2C ,SLAVE_0C };

	for (int i = 0; i < sizeof(slave_array) / sizeof(slave_array[0]); i++)
	{
		m_cfg.slave = slave_array[i];
		m_cfg.chipid = 0x00;
		ret = i2c_read_reg(m_cfg.slave, 0x00, &m_cfg.chipid, 1);
		if( (ret) && ( (m_cfg.chipid ==0x90)|| (m_cfg.chipid == 0x91)|| (m_cfg.chipid == 0x92)||(m_cfg.chipid == 0x20)) )
		{
			unsigned char buf[3];

			*chip_id = m_cfg.chipid;
			ret = i2c_read_reg(m_cfg.slave, 0x12, buf, 1);
			*v_id = buf[0];
			ret = i2c_read_reg(m_cfg.slave, 0x37, buf, 3);
			*w_id = buf[0]&0x1f;
			*d_id = (unsigned short)((buf[1] << 8) | buf[0]);
			ret = i2c_read_reg(m_cfg.slave, 0x41, buf, 2);
			*l_id = (unsigned short)((buf[1] << 8) | buf[0]);
			break;
		}
	}
}
#endif