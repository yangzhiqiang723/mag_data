
// qstevbDlg.h
//

#pragma once

#include "afxwin.h"
#include "SerialPort.h"
#include "SerialPortInfo.h"
#include "ChartCtrl/ChartCtrl.h" 
#include "ChartAxisLabel.h"
#include "ChartLineSerie.h"
#include "ChartBarSerie.h"
#include "MyFilter.h"
#include "CSVReader.hpp"
#include "qst_sensor_id.h"
#include "qst_algo_imu_cali.h"
#include "mag_hub.h"

using namespace itas109;
using namespace std;

//#define QST_CHART
#define QST_TEST_REG
#define QST_TEST_ODR
#define QST_TEST_ANGLE
#define QST_SENSOR_NUM		1

#define EVB_TIMER_ID_1		1
#define EVB_TIMER_ID_2		2
#define EVB_TIMER_ID_3		3
#define EVB_TIMER_ID_4		4
#define EVB_TIMER_ID_5		5

#define EVB_INIT_INFO		"G6RT03#5"
#define EVB_INIT_CHIPID		0x91
#define EVB_TIMER3_DELAY	2500
#define EVB_TIMER3_IBI_DELAY	4500

typedef enum
{
	FIFO_MODE_BYPASS = (0<<6),
	FIFO_MODE_FIFO = (1<<6),
	FIFO_MODE_STREAM = (2<<6),
	FIFO_MODE_DEFAULT = (3<<6)
} fifo_mode;

typedef enum
{
	TEST_DATA = 0,
	TEST_POLL_ODR,

	TEST_WRITE_READ_REGISTER,
	TEST_WORK_CURRENT,
	TEST_SETRESET_SWITCH,
	TEST_SELFTEST,
	TEST_SOFT_RESET,
	TEST_OTP,
	TEST_FACTORY,

	TEST_I3C_CCC,

	TEST_ANGLE,

#if defined(FPGA)
	TEST_GAIN_OFFSET,	// fpga only
	TEST_TCO,			// fpga only
	TEST_TCS,			// fpga only
	TEST_KMTX,			// fpga only
#endif
	
	TEST_MISC,
	TEST_MAX
} qst_sensor_test_item;

typedef struct
{
	string	portName;
	int		comid;
	int		baud;
	int		parity;
	int		data;
	int		stop;
	int		portNum;
	//BOOL	connect;
} evb_com_config;

typedef struct
{
	const  char				**range;
	int						range_num;
	const  char				**odr;
	int						odr_num;
	const  char				**osr1;
	int						osr1_num;
	const  char				**osr2;
	int						osr2_num;
	const  char				**sr;
	int						sr_num;
	const  char				**fifo_mode;
	int						fifo_mode_num;
	const  char				**fifo_wmk;
	int						fifo_wmk_num;
	const  char				** report;
	int						report_num;
} evb_mag_set_t;

typedef struct
{
	unsigned char		chipid;
	unsigned char		v_id;
	unsigned char		w_id;
	unsigned short		d_id;
	unsigned short		l_id;
	char				name[9];

	int					odr;

	int					test_i;
	int					fifo_en;
	int					fifo_mode;
	int					fifo_wmk;

	int					mode;
	int					osr1;
	int					osr2;
	int					range;
	int					sr;

	int					max_count;
	int					delay;
	int					report_mode;	// 0:polling 1:ibi
	//int					sr_num;

	int					at_test;
	int					cfg_i;
	int					cfg_max;

	int					id;
	int					id_max;

	float				mat[3][3];
	float				bias_a[3];
	float				bias_g[3];
	BOOL				imu_cali_flag;
} evb_mag_config;

typedef struct 
{
	__int64				tm_start;
	__int64				tm_now;	
	unsigned int		samples1;
	unsigned int		samples2;
	unsigned int		samples_last;
	float				odr;
} evb_sensor_odr;

#define EVB_ASCII_BUF_LEN	512
typedef struct 
{
	int				step;
	int				index;
	unsigned char	buf[EVB_ASCII_BUF_LEN];
} evb_sensor_ascii_decode;

typedef struct
{
	int				m_raw[3];
	float			m_out[3];
	float			mc_out[3];
	float			a_out[3];
	float			g_out[3];
	float			e_out[3];
	//float			std[3];
} evb_sensor_data;

// CqstevbDlg
class CqstevbDlg : public CDialogEx,public has_slots<>
{
//
public:
	CqstevbDlg(CWnd* pParent = NULL);

#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_QSTEVB_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);
public:
	//ui
	CButton				m_BtComOpen;
	CEdit				m_edit_rx;
	CString				m_string_rx;
	CEdit				m_edit_id;
	CEdit				m_edit_cmd;
	CString				m_id_info;
	CButton				m_BtSaveLog;
	CButton				m_BtOutPic;
	//CFont				m_font1;
	CFont				m_edit_font;
	CButton				m_BtSampling;	
	CButton				m_BtCatData;
	CButton				m_BtUser1;
	// com config
	CComboBox			m_ComId;
	CComboBox			m_ComBaud;
	CComboBox			m_ComParity;
	CComboBox			m_ComData;
	CComboBox			m_ComStop;
	HANDLE				m_hCom;	
	evb_com_config		m_comset;
	unsigned char		*m_rx_buf;
	BOOL				m_usbdev_detect;
	int					m_usbdev_type;
	BOOL				m_dev_connect;

	CComboBox			m_MagId;
	CComboBox			m_MagSensorId;
	CComboBox			m_MagMode;
	CComboBox			m_MagOsr1;
	CComboBox			m_MagOsr2;
	CComboBox			m_MagRange;
	CComboBox			m_MagSetReset;
	CComboBox			m_MagFifoMode;
	CComboBox			m_MagFifoWmk;
	CComboBox			m_MagTestI;	
	CComboBox			m_MagReport;

	CStatusBar			m_StatusBar;

	evb_mag_config		m_cfg;
	evb_mag_set_t		m_set;

	evb_sensor_data		m_data;
	evb_sensor_ascii_decode	m_asi_decode;
// log
	unsigned char		log_folder[64];
	CString				log_path;
	CFile 				log_file;
	BOOL				log_flag;
	BOOL				log_button;
	BOOL				log_each_flag;
	BOOL				log_temp;
// log
// user log
	CString				user_path;
	CFile 				user_file;
	BOOL				user_button;
// user log
	BOOL				m_graphic_flag;
	unsigned int		m_chart_sample;
	evb_sensor_odr		m_odr;

	BOOL				m_sample_flag;
	BOOL				m_user_cmd_flag;

	CString				m_BinPath;
private:
	CSerialPort			m_SerialPort;
	CSVReader			m_config_tbl;
	CSVReader			m_csv_para;
protected:
	HICON m_hIcon;
	//HANDLE m_hTimerQueue;   // 定时器队列句柄
	//HANDLE m_hTimer;        // 定时器句柄

	virtual BOOL OnInitDialog();
	virtual BOOL PreTranslateMessage(MSG* pMsg);
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
public:
	void evb_open_bin_path();
	void evb_refresh_ui();
	void evb_init_common_ui(void);
	void evb_init_config_ui(void);
	void evb_refresh_config_ui(void);
	void evb_rx_func(void);
	void evb_rx_display_buf(unsigned char *ptr, unsigned int len);
	void evb_rx_decode_ascii_buf(unsigned char *ptr, unsigned int len);
	// save log	
	void evb_open_log_file(void);
	void evb_write_log_file(char *str_buf, unsigned int len);
	void evb_write_log_file_ext(char* str_buf, unsigned int len);
	void evb_close_log_file(void);
	void evb_clear_user_file(CString path);
	void evb_write_user_file(CString path, char *str_buf, unsigned int len);
	void evb_open_config_file(void);
	void evb_create_folder(void);

	void evb_tx_func(unsigned char *buf, int len);
	int evb_calc_sample_odr(unsigned int sample);
	BOOL evb_check_digital(CString &str);
	BOOL evb_get_config(void);

	void evb_refresh_com(void);
	void evb_make_full_path(const CString& path);
	void evb_plot_file(CString path, int mode);

	// add by qst0103
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedComOpen();
#if defined(QST_USE_DEVICE)
	afx_msg void OnBnClickedUsbOpen();
#endif
	afx_msg void OnBnClickedOpen();
	afx_msg void OnCbnSelchangeComboComId();
	afx_msg void OnCbnSelchangeComboComBrud();
	afx_msg void OnCbnSelchangeComboComParity();
	afx_msg void OnCbnSelchangeComboComData();
	afx_msg void OnCbnSelchangeComboComStop();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnClose();
	afx_msg void OnCbnSelchangeComboMode();
	afx_msg void OnCbnSelchangeComboOsr1();
	afx_msg void OnCbnSelchangeComboOsr2();
	afx_msg void OnCbnSelchangeComboRange();
	afx_msg void OnBnClickedButtonSampling();
	afx_msg void OnBnClickedButtonImuCali();
	afx_msg void OnSelchangeComboSetReset();
	afx_msg void OnCbnSelchangeComboFifoMode();
	afx_msg void OnCbnSelchangeComboFifoWmk();
	afx_msg void OnCbnSelchangeComboTestI();
	afx_msg void OnBnClickedButtonUser();
	afx_msg void OnCbnSelchangeComboFifoReportMode();
	afx_msg LRESULT OnExeComplete(WPARAM wParam, LPARAM lParam);
	afx_msg void OnCbnSelchangeComboId();
	afx_msg void OnCbnSelchangeComboSensorType();

	//afx_msg HBRUSH OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor);
	afx_msg void OnBnClickedCheckOutPic();
};

