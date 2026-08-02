
// qstevbDlg.h
//

#pragma once

#include "afxwin.h"
#include "SerialPort.h"
#include "SerialPortInfo.h"
#include "ChartCtrl/ChartCtrl.h" 
#include ".\ChartCtrl\ChartAxisLabel.h"
#include ".\ChartCtrl\ChartLineSerie.h"
#include ".\ChartCtrl\ChartBarSerie.h"
#include ".\Filter\MyFilter.h"
#include ".\CSVReader\CSVReader.hpp"
#include "fusion_interface.h"
//#include "imu_soft_st.h"
#include "qst_sensor_id.h"
#include "smartpower.h"

using namespace itas109;
using namespace std;


#define QST_STD
#define QST_CHART
#define QST_FS_CACHE
//#define QST_TEST_SR
//#define QST_DIGITAL_POWER
//#define QST_MEASURE_CURRENT

#define EVB_TIMER_ID_1		1
#define EVB_TIMER_ID_2		2
#define EVB_TIMER_ID_3		3
#define EVB_TIMER_ID_4		4


typedef enum
{
	EVB_INTERFACE_NONE,
	EVB_INTERFACE_ST_COM,
	EVB_INTERFACE_USB_COM,
	EVB_INTERFACE_USB_CH341,
	EVB_INTERFACE_TOTAL
} evb_interface_t;

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
	TEST_WORK_MODE_SWITCH,
	TEST_SETRESET_SWITCH,
	TEST_SELFTEST,
	TEST_SOFT_RESET,
	TEST_OTP,
	TEST_FACTORY,

	TEST_I3C_IBI,
	TEST_I3C_CCC,

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
	BOOL	connect;
	int		portNum;
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
} evb_mag_set_t;

//typedef struct
//{
//	unsigned short	progress_i;
//	unsigned short	id;
//	unsigned short	delay;
//	unsigned short	test_i;
//	unsigned char	fifo_en;
//	unsigned char	fifo_mode;
//	unsigned char	fifo_wmk;
//	unsigned int	max_num;
//	unsigned char	mode;
//	unsigned char	range;
//	unsigned char	osr1;
//	unsigned char	osr2;
//	unsigned char	sr;
//} evb_mag_at_t;

typedef struct
{
	unsigned char		chipid;
	unsigned char		v_id;
	unsigned char		w_id;
	unsigned short		d_id;
	char				name[9];

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
	int					sr_num;
	int					report_mode;	// 0:polling 1:ibi

	int					id;
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
	float			raw[3];
	float			std[3];
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
	CEdit				m_edit_volt;
	CString				m_id_info;
	CButton				m_BtSaveLog;
	CButton				m_BtZDBL;
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

	CComboBox			m_MagMode;
	CComboBox			m_MagOsr1;
	CComboBox			m_MagOsr2;
	CComboBox			m_MagRange;
	CComboBox			m_MagSetReset;
	CComboBox			m_MagFifoMode;
	CComboBox			m_MagFifoWmk;
	CComboBox			m_MagTestI;	
	CComboBox			m_MagReport;
	// digital power
	CComboBox			m_DpEn;
	int					m_DpEnSel;
	float				m_DpVolt;
	// digital power

	CStatusBar			m_StatusBar;

	evb_mag_config		m_cfg;
	evb_mag_set_t		m_set;

	evb_sensor_data		m_data;
	evb_sensor_ascii_decode	m_asi_decode;
// log
	unsigned char		log_folder[32];
	CString				log_path;
	CFile 				log_file;
	BOOL				log_flag;
	BOOL				log_button;
// log
// user log
	CString				user_path;
	CFile 				user_file;
	BOOL				user_button;
// user log
	BOOL				m_graphic_flag;
	unsigned int		m_chart_sample;
	evb_sensor_odr		m_odr;

	//QstStd3				m_std_3;
	BOOL				m_sample_flag;
	BOOL				m_at_enable;
	int					m_at_i;
	int					m_at_max;

	BOOL				m_user_cmd_flag;
	BOOL				m_user_vlot_flag;
	BOOL				m_ComPause;

#if defined(QST_MEASURE_CURRENT)
// power monitor
	int 				m_dev_num;
	wchar_t 			m_dev_sn[30];
	double				m_dev_value[4];
	double				m_dev_avg_value[4];
// power monitor
// thread
	BOOL				m_thread_run;
	unsigned int		m_thread_count;
	CWinThread			* m_pThread;
// thread
#endif

	//CWinThread* 		m_log_Thread; 
private:
	CSerialPort			m_SerialPort;
	CSVReader			m_csv_at;
protected:
	HICON m_hIcon;

	virtual BOOL OnInitDialog();
	virtual BOOL PreTranslateMessage(MSG* pMsg);
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
public:
	void evb_refresh_ui();
	void evb_init_common_ui(void);	
	void evb_init_config_ui(void);
	void evb_rx_func(void);
	void evb_rx_display_buf(unsigned char *ptr, unsigned int len);
	void evb_rx_decode_ascii_buf(unsigned char *ptr, unsigned int len);
	// save log	
	void evb_open_log_file(void);
	void evb_write_log_file(char *str_buf, unsigned int len);
	void evb_close_log_file(void);
	//static UINT evb_log_thread(LPVOID pParam);
	void evb_write_user_file(CString path, char *str_buf, unsigned int len);
	void evb_open_config_file(void);
	void evb_create_folder(char *name);

	void evb_tx_func(unsigned char *buf, int len);
	int evb_write_reg(unsigned char slave, unsigned char reg, unsigned char value);
	void qst_evb_process_cmd(LPWSTR cmd);
	void evb_calc_sample_odr(unsigned int sample);
	BOOL evb_check_digital(CString &str);

	void evb_refresh_com(void);
#if defined(QST_MEASURE_CURRENT)
	static UINT MeasureThreadWork(LPVOID pParam);
	void MeasureThreadStart();
	void MeasureThreadStop();
#endif
	// add by qst0103
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedComOpen();
	afx_msg void OnCbnSelchangeComboComId();
	afx_msg void OnCbnSelchangeComboComBrud();
	afx_msg void OnCbnSelchangeComboComParity();
	afx_msg void OnCbnSelchangeComboComData();
	afx_msg void OnCbnSelchangeComboComStop();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnClose();
//	afx_msg HBRUSH OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor);
	afx_msg void OnCbnSelchangeComboMode();
	afx_msg void OnCbnSelchangeComboOsr1();
	afx_msg void OnCbnSelchangeComboOsr2();
	afx_msg void OnCbnSelchangeComboRange();
	afx_msg void OnBnClickedButtonSampling();
	afx_msg void OnBnClickedButtonUser1();
	afx_msg void OnSelchangeComboSetReset();
	afx_msg void OnCbnSelchangeComboFifoMode();
	afx_msg void OnCbnSelchangeComboFifoWmk();
	afx_msg void OnCbnSelchangeComboTestI();
	afx_msg void OnBnClickedButtonSetReset();
	afx_msg void OnBnClickedButtonUser();
	afx_msg void OnCbnSelchangeComboDigitalPowerEn();
	afx_msg void OnBnClickedButtonDigitalPowerSet();
	afx_msg void OnCbnSelchangeComboFifoReportMode();
};


