
// qstevbDlg.cpp :
//

#include "stdafx.h"
#include "qstevb.h"
#include "qstevbDlg.h"
#include "SubDialog.h"
#include "afxdialogex.h"
#include <fcntl.h> 
#include <io.h>
#include "conio.h"

#define UART_WRITE_DELAY			100	//20	// uint millsecond
#define MAX_RX_FRAME_BUF			1024*2
#define MAX_RX_READY_BUF			MAX_RX_FRAME_BUF

static char rx_ready_buf[MAX_RX_READY_BUF];
static char	m_cfg_info[80];

static int mag_at_item[7] = {0,0,0,0,0,0,0};

static const long com_baudrate[] = {4800,9600,14400,19200,38400,56000,57600,115200,256000,460800,512000,921600};
static const int com_databit[] = {8, 7, 6, 5};		// bit
static const int com_parity[] = {0, 1, 2, 3, 4};	// none; odd; even; mark; Space
static const int com_stop[] = {0, 1, 2};			// one, oneandhalf, two
static const int com_flow[] = {0, 1, 2};			// none, hardware, software
static const UINT indicators[] = {IDS_VERSION, IDS_SAMPLES, IDS_TIME};

static const short mag_svvt_array[] = {1000, 2000, 4000};

static const char* mag_mode_odr_maestro[] = {"hpf", "1000Hz", "400Hz", "200Hz", "100Hz", "50Hz", "20Hz", "10Hz", "1Hz"};
static const char* mag_mode_odr_qmc6309x[] = {"hpf", "200Hz", "100Hz", "50Hz", "10Hz", "1Hz"};
static const char* mag_mode_odr_qmc6308[] = {"hpf", "200Hz", "100Hz", "50Hz", "10Hz"};

static const char* mag_range_maestro[] = {"20Gs"};
static const char* mag_range_qmc6309x[] = {"32Gs"};	//{"32Gs", "16Gs", "8Gs"};
static const char* mag_range_qmc6308[] = {"30Gs"};	//{"30Gs", "12Gs", "8Gs", "2Gs"};

static const char* mag_osr1_maestro[] = {"1", "2", "4", "8", "16", "32"};
static const char* mag_osr1_qmc6309x[] = {"1", "2", "4", "8"};
static const char* mag_osr1_qmc6308[] = {"1", "2", "4", "8"};

static const char* mag_osr2_maestro[] = {"1", "2", "4", "8"};
static const char* mag_osr2_qmc6309x[] = {"1", "2", "4", "8", "16"};
static const char* mag_osr2_qmc6309v[] = { "1", "2", "4", "8" };
static const char* mag_osr2_qmc6308[] = {"1", "2", "4", "8"};

// static const char* mag_setreset_maestro[] = {"sroff", "seton", "rston"};
// static const char* mag_setreset_qmc6309x[] = {"sron", "seton", "rston", "sroff"};
// static const char* mag_setreset_qmc6308[] = {"sron", "seton", "sroff"};
static const char* mag_setreset_maestro[] = {"sr0", "s1", "r1"};
static const char* mag_setreset_qmc6309x[] = {"sr1", "s1", "r1", "sr0"};
static const char* mag_setreset_qmc6308[] = {"sr1", "s1", "sr0"};


static const char* mag_zdbl[] = {"zd0", "zd1"};

static const char* mag_fifo_mode[] = {"bypass", "fifo", "stream",};

static const char* mag_fifo_level_16[] = {"0","1","2","3","4","5","6","7","8","9","10","11","12","13","14","15"};
static const char* mag_fifo_level_8[] = {"0","1","2","3","4","5","6","7"};


static struct
{
	int		test_i;
	char	*str;
} mag_test_item[6];

static CSubDialog *m_dlg_chart = NULL;

#ifdef _DEBUG
#define new DEBUG_NEW
//#define DEBUG_CONSOLE
#endif
#define DEBUG_CONSOLE

#ifdef DEBUG_CONSOLE
#define QST_PRINTF		_cprintf		//printf
static BOOL console_flag = FALSE;

static void InitConsoleWindow()
{
	if(console_flag == FALSE)
	{
		AllocConsole();
		HANDLE handle = GetStdHandle(STD_OUTPUT_HANDLE);
		int hCrt = _open_osfhandle((long)handle, _O_TEXT);
		FILE * hf = _fdopen(hCrt, "w");
		*stdout = *hf;
		console_flag = TRUE;
		QST_PRINTF("InitConsoleWindow \n");
	}
}

static void CloseConsoleWindow()
{
	if(console_flag == TRUE)
	{
		FreeConsole();
		console_flag = FALSE;
		QST_PRINTF("CloseConsoleWindow \n");
	}
}
#else
#define QST_PRINTF
#endif

static void evb_simulate_key_press(BYTE key)
{
	// VK_RETURN
	keybd_event(key, 0, 0, 0);
	keybd_event(key, 0, KEYEVENTF_KEYUP, 0);
}

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV

// 
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()

// CqstevbDlg 

CqstevbDlg::CqstevbDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_QSTEVB_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CqstevbDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);

	DDX_Control(pDX, IDC_BUTTON_COM_OPEN, m_BtComOpen);
	DDX_Control(pDX, IDC_COMBO_COM_ID, m_ComId);
	DDX_Control(pDX, IDC_COMBO_COM_BRUD, m_ComBaud);
	DDX_Control(pDX, IDC_COMBO_COM_JYW, m_ComParity);
	DDX_Control(pDX, IDC_COMBO_COM_SJW, m_ComData);
	DDX_Control(pDX, IDC_COMBO_COM_TZW, m_ComStop);

	DDX_Control(pDX, IDC_COMBO_MODE, m_MagMode);
	DDX_Control(pDX, IDC_COMBO_OSR1, m_MagOsr1);
	DDX_Control(pDX, IDC_COMBO_OSR2, m_MagOsr2);
	DDX_Control(pDX, IDC_COMBO_RANGE, m_MagRange);
	DDX_Control(pDX, IDC_COMBO_SET_RESET, m_MagSetReset);
	DDX_Control(pDX, IDC_COMBO_FIFO_MODE, m_MagFifoMode);
	DDX_Control(pDX, IDC_COMBO_FIFO_WMK, m_MagFifoWmk);
	DDX_Control(pDX, IDC_COMBO_TEST_I, m_MagTestI);
	DDX_Control(pDX, IDC_COMBO_FIFO_REPORT_MODE, m_MagReport);
	
	DDX_Control(pDX, IDC_COMBO_DIGITAL_POWER_EN, m_DpEn);
	

	DDX_Control(pDX, IDC_EDIT_RX, m_edit_rx);
	DDX_Control(pDX, IDC_EDIT_TEST_ID, m_edit_id);
	DDX_Control(pDX, IDC_EDIT_CUSTOM_CMD, m_edit_cmd);
	DDX_Control(pDX, IDC_EDIT_DIGITAL_POWER_VOLT, m_edit_volt);

	DDX_Control(pDX, IDC_CHECK_SAVE_LOG, m_BtSaveLog);
	DDX_Control(pDX, IDC_CHECK_ZDBL, m_BtZDBL);
	DDX_Control(pDX, IDC_BUTTON_SAMPLING, m_BtSampling);
	DDX_Control(pDX, IDC_BUTTON_USER1, m_BtUser1);	
	DDX_Control(pDX, IDC_BUTTON_USER, m_BtCatData);
}

BEGIN_MESSAGE_MAP(CqstevbDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON_COM_OPEN, &CqstevbDlg::OnBnClickedComOpen)
	ON_CBN_SELCHANGE(IDC_COMBO_COM_ID, &CqstevbDlg::OnCbnSelchangeComboComId)
	ON_CBN_SELCHANGE(IDC_COMBO_COM_BRUD, &CqstevbDlg::OnCbnSelchangeComboComBrud)
	ON_CBN_SELCHANGE(IDC_COMBO_COM_JYW, &CqstevbDlg::OnCbnSelchangeComboComParity)
	ON_CBN_SELCHANGE(IDC_COMBO_COM_SJW, &CqstevbDlg::OnCbnSelchangeComboComData)
	ON_CBN_SELCHANGE(IDC_COMBO_COM_TZW, &CqstevbDlg::OnCbnSelchangeComboComStop)
	ON_WM_TIMER()
	ON_WM_CLOSE()
	ON_WM_CTLCOLOR()
	ON_CBN_SELCHANGE(IDC_COMBO_MODE, &CqstevbDlg::OnCbnSelchangeComboMode)
	ON_CBN_SELCHANGE(IDC_COMBO_OSR1, &CqstevbDlg::OnCbnSelchangeComboOsr1)
	ON_CBN_SELCHANGE(IDC_COMBO_OSR2, &CqstevbDlg::OnCbnSelchangeComboOsr2)
	ON_CBN_SELCHANGE(IDC_COMBO_RANGE, &CqstevbDlg::OnCbnSelchangeComboRange)
	ON_BN_CLICKED(IDC_BUTTON_SAMPLING, &CqstevbDlg::OnBnClickedButtonSampling)
	ON_BN_CLICKED(IDC_BUTTON_USER1, &CqstevbDlg::OnBnClickedButtonUser1)
	ON_CBN_SELCHANGE(IDC_COMBO_SET_RESET, &CqstevbDlg::OnSelchangeComboSetReset)
	ON_CBN_SELCHANGE(IDC_COMBO_FIFO_MODE, &CqstevbDlg::OnCbnSelchangeComboFifoMode)
	ON_CBN_SELCHANGE(IDC_COMBO_FIFO_WMK, &CqstevbDlg::OnCbnSelchangeComboFifoWmk)
	ON_CBN_SELCHANGE(IDC_COMBO_TEST_I, &CqstevbDlg::OnCbnSelchangeComboTestI)
	ON_BN_CLICKED(IDC_BUTTON_USER, &CqstevbDlg::OnBnClickedButtonUser)
	ON_CBN_SELCHANGE(IDC_COMBO_DIGITAL_POWER_EN, &CqstevbDlg::OnCbnSelchangeComboDigitalPowerEn)
	ON_BN_CLICKED(IDC_BUTTON_DIGITAL_POWER_SET, &CqstevbDlg::OnBnClickedButtonDigitalPowerSet)
	ON_CBN_SELCHANGE(IDC_COMBO_FIFO_REPORT_MODE, &CqstevbDlg::OnCbnSelchangeComboFifoReportMode)
END_MESSAGE_MAP()


// CqstevbDlg 
BOOL CqstevbDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// IDM_ABOUTBOX 
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		CString strCutom;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);		
		//pSysMenu->EnableMenuItem(SC_CLOSE, MF_GRAYED);	// MF_DISABLED MF_GRAYED
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);

			strCutom.LoadString(IDS_STRING_REFRESH_COM);
			pSysMenu->AppendMenu(MF_STRING, IDM_REFRESH_COM, strCutom);

			strCutom.LoadString(IDS_STRING_GPAPHIC);
			pSysMenu->AppendMenu(MF_STRING, IDM_GRAPHIC, strCutom);

			strCutom.LoadString(IDS_STRING_AUTO_TEST);
			pSysMenu->AppendMenu(MF_STRING, IDM_AUTO_TEST, strCutom);

			strCutom.LoadString(IDS_STRING_CUSTOM_CMD);
			pSysMenu->AppendMenu(MF_STRING, IDM_USER_CMD, strCutom);

			strCutom.LoadString(IDS_STRING_CUSTOM_VOLT);
			pSysMenu->AppendMenu(MF_STRING, IDM_USER_VOLT, strCutom);

			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
		//	pSysMenu->CheckMenuItem(IDM_AUTO_TEST, MF_CHECKED | MF_BYCOMMAND);
	}

	m_StatusBar.Create(this);
	m_StatusBar.SetIndicators(indicators, 3);
	m_StatusBar.SetPaneInfo(0, IDS_VERSION, SBPS_STRETCH, 400);
	m_StatusBar.SetPaneInfo(1, IDS_SAMPLES, SBPS_STRETCH, 250);
	m_StatusBar.SetPaneInfo(2, IDS_TIME, SBPS_STRETCH, 200);
	RepositionBars(AFX_IDW_CONTROLBAR_FIRST, AFX_IDW_CONTROLBAR_LAST, IDS_TIME);

#ifdef DEBUG_CONSOLE
	InitConsoleWindow();
#endif
	m_hCom = INVALID_HANDLE_VALUE;

	m_rx_buf = (unsigned char*)rx_ready_buf;

	m_comset.comid = 0;
	m_comset.baud = 115200;			//921600
	m_comset.parity = 0;
	m_comset.data = 8;
	m_comset.stop = 0;
	m_comset.connect = FALSE;

	memset(&m_cfg, 0, sizeof(m_cfg));
	m_cfg.mode = 0;				// hpf
	m_cfg.osr1 = 3;				// 8
	m_cfg.osr2 = 1;				// 1
	m_cfg.range = 0;			// 
	m_cfg.sr = 0;				// set-reset on
	m_cfg.test_i = 0;
	m_cfg.fifo_en = 0;
	m_cfg.fifo_mode = 0;
	m_cfg.fifo_wmk = 0;
	m_cfg.chipid = 0x00;
	m_cfg.sr_num = 10;
	m_cfg.report_mode = 0;
	m_cfg.max_count = 0x7fffffff;
	m_cfg.id = 0;
	strcpy(m_cfg.name , "qmcXXX");

	m_DpEnSel = 1;
	m_sample_flag = FALSE;
	m_chart_sample = 0;
	m_graphic_flag = FALSE;	
	log_flag = log_button = FALSE;	
	user_button = FALSE;
	m_at_enable = FALSE;
	m_at_i = 1;
	m_user_cmd_flag = FALSE;
	m_user_vlot_flag = FALSE;
	m_ComPause = FALSE;
	memset(&m_odr, 0, sizeof(m_odr));
	user_path.Empty();
	log_path.Empty();

	evb_init_common_ui();
	evb_init_config_ui();
	evb_refresh_ui();
	if(m_dlg_chart == NULL)
	{
		m_dlg_chart = new CSubDialog();
		m_dlg_chart->Create(CSubDialog::IDD, /*this*/GetDesktopWindow());
		m_dlg_chart->ModifyStyle(WS_SYSMENU, 0); // 鍘绘帀鍏抽棴鎸夐挳
		m_dlg_chart->ShowWindow(SW_HIDE);
	}
	m_SerialPort.readReady.connect(this, &CqstevbDlg::evb_rx_func);
//	m_log_Thread = AfxBeginThread(CqstevbDlg::evb_log_thread, NULL, THREAD_PRIORITY_NORMAL, 0, CREATE_SUSPENDED);

	SetIcon(m_hIcon, TRUE);
	//SetIcon(m_hIcon, FALSE);

#if defined(QST_MEASURE_CURRENT)
	m_dev_num = SmartPower_DeviceNums();
	if(m_dev_num > 0)
	{
		if (SmartPower_DeviceOpen_Ext(0, m_dev_sn))
		{
			SmartPower_OutputOn(1.8 , 2);
			QST_PRINTF("Power monitor open ok! %S\r\n", m_dev_sn);
		}
	}
#endif

	return TRUE;
}

void CqstevbDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	CMenu* pSysMenu = GetSystemMenu(FALSE);

	if((nID & 0xFFFF) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else if((nID & 0xFFFF) == IDM_REFRESH_COM)
	{
		evb_refresh_com();
	}
	else if((nID & 0xFFFF) == IDM_GRAPHIC)
	{
		if(m_graphic_flag == FALSE)
		{
			m_graphic_flag = TRUE;
			pSysMenu->CheckMenuItem(IDM_GRAPHIC, MF_CHECKED | MF_BYCOMMAND);
		}
		else
		{
			m_graphic_flag = FALSE;
			pSysMenu->CheckMenuItem(IDM_GRAPHIC, MF_UNCHECKED | MF_BYCOMMAND);
		}
		if(m_graphic_flag)
		{
			m_dlg_chart->ShowWindow(SW_SHOW);
		}
		else
		{
			m_dlg_chart->ShowWindow(SW_HIDE);
		}
	}
	else if((nID & 0xFFFF) == IDM_AUTO_TEST)
	{
		if(m_at_enable == FALSE)
		{
			m_at_enable = TRUE;
			pSysMenu->CheckMenuItem(IDM_AUTO_TEST, MF_CHECKED | MF_BYCOMMAND);
		}
		else
		{
			m_at_enable = FALSE;
			pSysMenu->CheckMenuItem(IDM_AUTO_TEST, MF_UNCHECKED | MF_BYCOMMAND);
		}
	}
	else if((nID & 0xFFFF) == IDM_USER_CMD)
	{
		if(m_user_cmd_flag == FALSE)
		{
			m_user_cmd_flag = TRUE;
			pSysMenu->CheckMenuItem(IDM_USER_CMD, MF_CHECKED | MF_BYCOMMAND);
		}
		else
		{
			m_user_cmd_flag = FALSE;
			pSysMenu->CheckMenuItem(IDM_USER_CMD, MF_UNCHECKED | MF_BYCOMMAND);
		}
		if(m_user_cmd_flag)
		{
			m_edit_cmd.ShowWindow(SW_SHOW);
		}
		else
		{
			m_edit_cmd.ShowWindow(SW_HIDE);
		}
	}
	else if((nID & 0xFFFF) == IDM_USER_VOLT)
	{
		if(m_user_vlot_flag == FALSE)
		{
			m_user_vlot_flag = TRUE;
			pSysMenu->CheckMenuItem(IDM_USER_VOLT, MF_CHECKED | MF_BYCOMMAND);
		}
		else
		{
			m_user_vlot_flag = FALSE;
			pSysMenu->CheckMenuItem(IDM_USER_VOLT, MF_UNCHECKED | MF_BYCOMMAND);
		}
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

void CqstevbDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this);

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		//
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}


HCURSOR CqstevbDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void CqstevbDlg::evb_refresh_ui(void)
{
	if(m_comset.connect)
	{
		m_BtComOpen.EnableWindow(1);
		m_ComId.EnableWindow(0);
		m_ComBaud.EnableWindow(0);
		m_ComParity.EnableWindow(0);
		m_ComData.EnableWindow(0);
		m_ComStop.EnableWindow(0);
		m_BtSampling.EnableWindow(1);
		m_BtCatData.EnableWindow(1);
		m_BtUser1.EnableWindow(0);

		m_BtComOpen.SetWindowTextW(_T("Close"));
	}
	else
	{
		m_BtComOpen.EnableWindow(1);
		m_ComId.EnableWindow(1);
		m_ComBaud.EnableWindow(1);
		m_ComParity.EnableWindow(1);
		m_ComData.EnableWindow(1);
		m_ComStop.EnableWindow(1);
		m_BtComOpen.SetWindowTextW(_T("Open"));
		m_BtSampling.EnableWindow(0);
		m_BtCatData.EnableWindow(0);
		m_BtUser1.EnableWindow(0);

		if(m_dlg_chart && m_graphic_flag)
		{
			m_BtUser1.SetWindowTextW(_T("Show graphic"));
			m_dlg_chart->ShowWindow(SW_HIDE);
			m_graphic_flag = FALSE;
		}
	}

	if(m_sample_flag && m_comset.connect)
	{
		m_edit_id.EnableWindow(0);
		m_edit_cmd.EnableWindow(1);
		m_MagTestI.EnableWindow(0);
		m_MagMode.EnableWindow(0);
		m_MagOsr1.EnableWindow(0);
		m_MagOsr2.EnableWindow(0);
		m_MagRange.EnableWindow(0);
		m_MagSetReset.EnableWindow(0);
		m_MagFifoMode.EnableWindow(0);
		m_MagFifoWmk.EnableWindow(0);
		m_MagReport.EnableWindow(0);
		m_BtSaveLog.EnableWindow(0);
		m_BtZDBL.EnableWindow(0);
		m_BtUser1.EnableWindow(1);
		m_BtSampling.SetWindowTextW(_T("Stop"));
	}
	else if(m_comset.connect)
	{
		m_edit_id.EnableWindow(1);
		m_edit_cmd.EnableWindow(1);
		m_MagTestI.EnableWindow(1);
		m_MagMode.EnableWindow(1);
		m_MagOsr1.EnableWindow(1);
		m_MagOsr2.EnableWindow(1);
		m_MagRange.EnableWindow(1);
		m_MagReport.EnableWindow(1);
		if((m_cfg.chipid==0x10)||(m_cfg.chipid==0x20))
		{
			m_MagSetReset.EnableWindow(1);
		}
		else
		{
			m_MagSetReset.EnableWindow(1);
		}
		if(m_cfg.chipid == 0x80)
		{
			m_MagFifoMode.EnableWindow(0);
			m_MagFifoWmk.EnableWindow(0);
		}
		else
		{
			m_MagFifoMode.EnableWindow(1);
			m_MagFifoWmk.EnableWindow(1);
		}	
		m_BtSaveLog.EnableWindow(1);
		m_BtZDBL.EnableWindow(1);
		m_BtUser1.EnableWindow(0);
		m_BtCatData.EnableWindow(0);
		m_BtSampling.SetWindowTextW(_T("Start"));
	}
	else
	{
		m_edit_id.EnableWindow(0);
		m_edit_cmd.EnableWindow(0);
		m_MagTestI.EnableWindow(0);
		m_MagMode.EnableWindow(0);
		m_MagOsr1.EnableWindow(0);
		m_MagOsr2.EnableWindow(0);
		m_MagRange.EnableWindow(0);
		m_MagSetReset.EnableWindow(0);
		m_MagFifoMode.EnableWindow(0);
		m_MagFifoWmk.EnableWindow(0);
		m_MagReport.EnableWindow(0);
		m_BtSaveLog.EnableWindow(0);
		m_BtZDBL.EnableWindow(0);
		m_BtUser1.EnableWindow(0);
		m_BtCatData.EnableWindow(0);
		m_BtSampling.SetWindowTextW(_T("Start"));
	}
	
	log_button = m_BtSaveLog.GetCheck();
	//GetDlgItem(IDC_STATIC_SENSOR_SELECT)->ShowWindow(SW_SHOW);
	//m_edit_browse.EnableFileBrowseButton(_T("bin"), _T("bin files|*.bin|hex files|*.hex|all file|*.*||"));
}

void CqstevbDlg::evb_init_common_ui(void)
{
	int result = 0;
	CString comStr;
	
	m_comset.connect = FALSE;
	vector<SerialPortInfo> m_portsList = CSerialPortInfo::availablePortInfos();
	TCHAR m_regKeyValue[255];
	m_comset.portNum = m_portsList.size();
	for(int i = 0; i < m_comset.portNum; i++)
	{
#ifdef UNICODE
		int iLength;
		const char * _char = m_portsList[i].portName.c_str();
		iLength = MultiByteToWideChar(CP_ACP, 0, _char, strlen(_char) + 1, NULL, 0);
		MultiByteToWideChar(CP_ACP, 0, _char, strlen(_char) + 1, m_regKeyValue, iLength);
#else
		strcpy_s(m_regKeyValue, 255, m_portsList[i].portName.c_str());
#endif
		m_ComId.AddString(m_regKeyValue);
	}
	m_ComId.SetCurSel(0);
	if(m_comset.portNum > 0)
		m_BtComOpen.SetWindowTextW(_T("Open"));
	else
		m_BtComOpen.SetWindowTextW(_T("Close"));

	for(int i = 0; i < sizeof(com_baudrate)/sizeof(com_baudrate[0]); i++)
	{
		comStr.Empty();
		comStr.Format(L"%ld", com_baudrate[i]);
		result = m_ComBaud.AddString(comStr.GetBuffer());
		if ((result == CB_ERR) || (result == CB_ERRSPACE))
			MessageBox(_T("build baud error!"));
	
		if(com_baudrate[i] == m_comset.baud)
		{
			m_ComBaud.SetCurSel(i);
		}
	}

	TCHAR Parity[][3] = { L"N", L"O", L"E", L"M", L"S" };
	for(int i = 0; i < 5; i++)
	{
		m_ComParity.AddString(Parity[i]);
		if(m_comset.parity == com_parity[i])
			m_ComParity.SetCurSel(i);
	}

	for(int i = 0; i < sizeof(com_databit)/sizeof(com_databit[0]); i++)
	{
		comStr.Empty();
		comStr.Format(L"%d", com_databit[i]);
		m_ComData.AddString(comStr.GetBuffer());
		if(m_comset.data == com_databit[i])
			m_ComData.SetCurSel(i);
	}

	for(int i = 0; i < sizeof(com_stop)/sizeof(com_stop[0]); i++)
	{
		char *stop_str[3] = {"1", "1.5", "2"};
		comStr.Empty();
		comStr.Format(L"%S", stop_str[i]);
		m_ComStop.AddString(comStr.GetBuffer());
		if(m_comset.stop == com_stop[i])
			m_ComStop.SetCurSel(i);
	}
#if 0
	m_edit_font.CreateFont( -13,          // 楂樺害锛堣礋鏁拌〃绀洪€昏緫鍗曚綅锛?
					        -7,         // 瀹藉害
					        0,            // 鏂滃害
					        0,            // 鏂瑰悜
					        400,          // 绮楃粏锛?00涓烘甯革級
							FALSE,        // 鏂滀綋
					        FALSE,        // 涓嬪垝绾?
					        FALSE,        // 鍒犻櫎绾?
					        DEFAULT_CHARSET,
					        OUT_DEFAULT_PRECIS,
					        CLIP_DEFAULT_PRECIS,
							DEFAULT_QUALITY,
					        FF_DONTCARE,
					        _T("瀹嬩綋")
						);
#else
	m_edit_font.CreatePointFont(109, _T("寰蒋闆呴粦"));		//寰蒋闆呴粦
#endif
	m_edit_rx.SetFont(&m_edit_font);
	m_edit_rx.SetLimitText(-1);
	m_edit_id.SetWindowTextW(_T("1#"));
	m_edit_cmd.SetWindowTextW(_T("0"));
	m_edit_volt.SetWindowTextW(_T("1.8"));
	m_DpVolt = 1.8f;

	m_BtSaveLog.SetCheck(FALSE);
	m_BtSaveLog.ShowWindow(SW_SHOW);
	m_BtZDBL.SetCheck(TRUE);	
	m_BtZDBL.ShowWindow(SW_HIDE);
#if !defined(QST_DIGITAL_POWER)
	GetDlgItem(IDC_STATIC_GROUP_DIGITAL_POWER)->ShowWindow(SW_HIDE);
	GetDlgItem(IDC_COMBO_DIGITAL_POWER_EN)->ShowWindow(SW_HIDE);
	GetDlgItem(IDC_EDIT_DIGITAL_POWER_VOLT)->ShowWindow(SW_HIDE);
	GetDlgItem(IDC_BUTTON_DIGITAL_POWER_SET)->ShowWindow(SW_HIDE);
#endif
#if defined(QST_CHART)
	m_BtUser1.ShowWindow(SW_HIDE);	// SW_SHOW
#else
	m_BtUser1.ShowWindow(SW_HIDE);
#endif
	if(m_user_cmd_flag)
		m_edit_cmd.ShowWindow(SW_SHOW);
	else
		m_edit_cmd.ShowWindow(SW_HIDE);
}


void CqstevbDlg::evb_init_config_ui(void)
{
	CString magStr;
	int item_i = 0;

	evb_open_config_file();
	memset(m_cfg.name, 0, sizeof(m_cfg.name));
	if(m_cfg.chipid == 0x90)
		strcpy(m_cfg.name , "6309h");
	else if(m_cfg.chipid == 0x91)
		strcpy(m_cfg.name , "6309v");
	else if (m_cfg.chipid == 0x92)
		strcpy(m_cfg.name, "6309p");
	else if(m_cfg.chipid == 0x10)
		strcpy(m_cfg.name , "6g00v");
	else if(m_cfg.chipid == 0x20)
		strcpy(m_cfg.name , "6g00h");
	else if(m_cfg.chipid == 0x80)
		strcpy(m_cfg.name , "6308");
	else
		strcpy(m_cfg.name , "qmcXXX");

	if((m_cfg.chipid == 0x20)||(m_cfg.chipid == 0x10))
	{
		m_set.odr = mag_mode_odr_maestro;
		m_set.odr_num = sizeof(mag_mode_odr_maestro)/sizeof(mag_mode_odr_maestro[0]);
		m_set.osr1 = mag_osr1_maestro;
		m_set.osr1_num = sizeof(mag_osr1_maestro)/sizeof(mag_osr1_maestro[0]);
		m_set.osr2 = mag_osr2_maestro;
		m_set.osr2_num = sizeof(mag_osr2_maestro)/sizeof(mag_osr2_maestro[0]);
		m_set.range = mag_range_maestro;
		m_set.range_num = sizeof(mag_range_maestro)/sizeof(mag_range_maestro[0]);
		m_set.sr = mag_setreset_maestro;
		m_set.sr_num = sizeof(mag_setreset_maestro)/sizeof(mag_setreset_maestro[0]);

		m_set.fifo_mode = mag_fifo_mode;
		m_set.fifo_mode_num = sizeof(mag_fifo_mode)/sizeof(mag_fifo_mode[0]);
		m_set.fifo_wmk = mag_fifo_level_16;
		m_set.fifo_wmk_num = sizeof(mag_fifo_level_16)/sizeof(mag_fifo_level_16[0]);

		mag_test_item[item_i].test_i = TEST_DATA;
		mag_test_item[item_i++].str = "read data";
		mag_test_item[item_i].test_i = TEST_SELFTEST;
		mag_test_item[item_i++].str = "poll selftest";
#if defined(QST_TEST_SR)
		mag_test_item[item_i].test_i = TEST_SETRESET_SWITCH;
		mag_test_item[item_i++].str = "manual sr";
#endif
		mag_test_item[item_i].test_i = TEST_POLL_ODR;
		mag_test_item[item_i++].str = "poll odr";
		mag_test_item[item_i].test_i = 0xff;
		mag_test_item[item_i++].str = "null";

		m_cfg.mode = 0; 			// hpf
		m_cfg.osr1 = 3; 			// 8
		m_cfg.osr2 = 2; 			// 1
		m_cfg.range = 0;			// 30Gs
		m_cfg.sr = 0;				// 0:sroff 1:seton 2:rston
	}
	else if((m_cfg.chipid == 0x90)||(m_cfg.chipid == 0x91)||(m_cfg.chipid == 0x92))
	{
		m_set.odr = mag_mode_odr_qmc6309x;
		m_set.odr_num = sizeof(mag_mode_odr_qmc6309x)/sizeof(mag_mode_odr_qmc6309x[0]);
		m_set.osr1 = mag_osr1_qmc6309x;
		m_set.osr1_num = sizeof(mag_osr1_qmc6309x)/sizeof(mag_osr1_qmc6309x[0]);
		if (m_cfg.chipid == 0x91)
		{
			m_set.osr2 = mag_osr2_qmc6309v;
			m_set.osr2_num = sizeof(mag_osr2_qmc6309v) / sizeof(mag_osr2_qmc6309v[0]);
			m_set.fifo_wmk = mag_fifo_level_16;
			m_set.fifo_wmk_num = sizeof(mag_fifo_level_16)/sizeof(mag_fifo_level_16[0]);
		}
		else
		{
			m_set.osr2 = mag_osr2_qmc6309x;
			m_set.osr2_num = sizeof(mag_osr2_qmc6309x) / sizeof(mag_osr2_qmc6309x[0]);
			m_set.fifo_wmk = mag_fifo_level_8;
			m_set.fifo_wmk_num = sizeof(mag_fifo_level_8)/sizeof(mag_fifo_level_8[0]);
		}
		m_set.range = mag_range_qmc6309x;
		m_set.range_num = sizeof(mag_range_qmc6309x)/sizeof(mag_range_qmc6309x[0]);
		m_set.sr = mag_setreset_qmc6309x;
		m_set.sr_num = sizeof(mag_setreset_qmc6309x)/sizeof(mag_setreset_qmc6309x[0]);

		m_set.fifo_mode = mag_fifo_mode;
		m_set.fifo_mode_num = sizeof(mag_fifo_mode)/sizeof(mag_fifo_mode[0]);



		mag_test_item[item_i].test_i = TEST_DATA;
		mag_test_item[item_i++].str = "read data";
		mag_test_item[item_i].test_i = TEST_SELFTEST;
		mag_test_item[item_i++].str = "read selftest";
		mag_test_item[item_i].test_i = TEST_POLL_ODR;
		mag_test_item[item_i++].str = "calc odr";
#if defined(QST_MEASURE_CURRENT)
		mag_test_item[item_i].test_i = TEST_WORK_CURRENT;
		mag_test_item[item_i++].str = "work current";
#endif
		mag_test_item[item_i].test_i = 0xff;
		mag_test_item[item_i++].str = "null";


		m_cfg.mode = 0;				// hpf
		m_cfg.osr1 = 3;				// 8
		m_cfg.osr2 = 2;				// 4
		m_cfg.range = 0;			// 32Gs
		m_cfg.sr = 0;				// set-reset on
	}
	else //if(m_cfg.chipid == 0x80)
	{
		m_set.odr = mag_mode_odr_qmc6308;
		m_set.odr_num = sizeof(mag_mode_odr_qmc6308)/sizeof(mag_mode_odr_qmc6308[0]);
		m_set.osr1 = mag_osr1_qmc6308;
		m_set.osr1_num = sizeof(mag_osr1_qmc6308)/sizeof(mag_osr1_qmc6308[0]);
		m_set.osr2 = mag_osr2_qmc6308;
		m_set.osr2_num = sizeof(mag_osr2_qmc6308)/sizeof(mag_osr2_qmc6308[0]);
		m_set.range = mag_range_qmc6308;
		m_set.range_num = sizeof(mag_range_qmc6308)/sizeof(mag_range_qmc6308[0]);
		m_set.sr = mag_setreset_qmc6308;
		m_set.sr_num = sizeof(mag_setreset_qmc6308)/sizeof(mag_setreset_qmc6308[0]);

		mag_test_item[item_i].test_i = TEST_DATA;
		mag_test_item[item_i++].str = "read data";
		mag_test_item[item_i].test_i = TEST_SELFTEST;
		mag_test_item[item_i++].str = "poll selftest";
		//mag_test_item[item_i].test_i = TEST_POLL_ODR;
		//mag_test_item[item_i++].str = "poll odr";
		mag_test_item[item_i].test_i = 0xff;
		mag_test_item[item_i++].str = "null";

		m_set.fifo_mode = NULL;
		m_set.fifo_mode_num = 0;
		m_set.fifo_wmk = NULL;
		m_set.fifo_wmk_num = 0;

		m_cfg.mode = 0;
		m_cfg.osr1 = 3;
		m_cfg.osr2 = 2;
		m_cfg.range = 0;
		m_cfg.sr = 0;
	}

	m_MagMode.Clear();
	m_MagOsr1.Clear();
	m_MagOsr2.Clear();
	m_MagRange.Clear();
	m_MagSetReset.Clear();
	m_MagFifoMode.Clear();
	m_MagFifoWmk.Clear();
	m_MagReport.Clear();
	m_MagTestI.Clear();
	m_MagMode.ResetContent();
	m_MagOsr1.ResetContent();
	m_MagOsr2.ResetContent();
	m_MagRange.ResetContent();
	m_MagSetReset.ResetContent();
	m_MagFifoMode.ResetContent();
	m_MagFifoWmk.ResetContent();
	m_MagReport.ResetContent();
	m_MagTestI.ResetContent();
	m_DpEn.ResetContent();

	if(m_set.odr)
	{
		for(int i = 0; i < m_set.odr_num; i++)
		{
			magStr.Empty();
			magStr.Format(L"%S", m_set.odr[i]);
			m_MagMode.AddString(magStr.GetBuffer());
		}	
		m_MagMode.SetCurSel(m_cfg.mode);
	}
	if(m_set.osr1)
	{
		for(int i = 0; i < m_set.osr1_num; i++)
		{
			magStr.Empty();
			magStr.Format(L"%S", m_set.osr1[i]);
			m_MagOsr1.AddString(magStr.GetBuffer());
		}	
		m_MagOsr1.SetCurSel(m_cfg.osr1);
	}
	if(m_set.osr2)
	{
		for(int i = 0; i < m_set.osr2_num; i++)
		{
			magStr.Empty();
			magStr.Format(L"%S", m_set.osr2[i]);
			m_MagOsr2.AddString(magStr.GetBuffer());
		}	
		m_MagOsr2.SetCurSel(m_cfg.osr2);
	}
	if(m_set.range)
	{
		for(int i = 0; i < m_set.range_num; i++)
		{
			magStr.Empty();
			magStr.Format(L"%S", m_set.range[i]);
			m_MagRange.AddString(magStr.GetBuffer());
		}	
		m_MagRange.SetCurSel(m_cfg.range);
	}
	if(m_set.sr)
	{
		for(int i = 0; i < m_set.sr_num; i++)
		{
			magStr.Empty();
			magStr.Format(L"%S", m_set.sr[i]);
			m_MagSetReset.AddString(magStr.GetBuffer());
		}	
		m_MagSetReset.SetCurSel(m_cfg.sr);
	}

	if(m_set.fifo_mode)
	{
		for(int i = 0; i < m_set.fifo_mode_num; i++)
		{
			magStr.Empty();
			magStr.Format(L"%S", m_set.fifo_mode[i]);
			m_MagFifoMode.AddString(magStr.GetBuffer());
		}	
		m_MagFifoMode.SetCurSel(m_cfg.fifo_mode);
	}
	if(m_set.fifo_wmk)
	{		
		for(int i = 0; i < m_set.fifo_wmk_num; i++)
		{
			magStr.Empty();
			magStr.Format(L"%S", m_set.fifo_wmk[i]);
			m_MagFifoWmk.AddString(magStr.GetBuffer());
		}	
		m_MagFifoWmk.SetCurSel(m_cfg.fifo_wmk);
	}

	magStr.Empty();
	magStr.Format(L"%S", "polling");
	m_MagReport.AddString(magStr.GetBuffer());
	if (strstr(m_cfg_info, "I3C"))
	{
		magStr.Empty();
		magStr.Format(L"%S", "ibi");
		m_MagReport.AddString(magStr.GetBuffer());
	}
	else
	{
		m_cfg.report_mode = 0;
	}
	m_MagReport.SetCurSel(m_cfg.report_mode);

	for(int i = 0; i < sizeof(mag_test_item)/sizeof(mag_test_item[0]); i++)
	{
		if(mag_test_item[i].test_i == 0xff)
		{
			break;
		}
		magStr.Empty();
		magStr.Format(L"%S", mag_test_item[i].str);
		m_MagTestI.AddString(magStr.GetBuffer());
	}
	m_MagTestI.SetCurSel(m_cfg.test_i);

	m_DpEn.AddString(L"off");
	m_DpEn.AddString(L"on");
	m_DpEn.SetCurSel(m_DpEnSel);

	evb_refresh_ui();
}

void CqstevbDlg::evb_rx_func(void)
{
	if (m_ComPause)
	{
		m_SerialPort.readAllData((char*)m_rx_buf);
		return;
	}

	int iLen = m_SerialPort.readAllData((char *)m_rx_buf);
	if((iLen > 0) && (iLen<MAX_RX_READY_BUF))
	{
		m_rx_buf[iLen] = '\0';
		//QST_PRINTF("%s",m_rx_buf);
		evb_rx_display_buf(m_rx_buf, iLen);
		evb_rx_decode_ascii_buf(m_rx_buf, iLen);
	}
}

void CqstevbDlg::evb_rx_display_buf(unsigned char *buf, unsigned int len)
{
	m_string_rx.Empty();
	m_string_rx.Format(_T("%S"), buf);		// buf
//	m_string_rx += rx_str;
//	m_edit_rx.SetSel(-1, -1, FALSE);
//	m_edit_rx.ReplaceSel(m_string_rx);
	m_edit_rx.ReplaceSel(m_string_rx);
	m_edit_rx.LineScroll(m_edit_rx.GetLineCount());

//	m_string_rx.Empty();
//	m_edit_rx.SetFocus();
}

void CqstevbDlg::evb_rx_decode_ascii_buf(unsigned char *ptr, unsigned int len)
{
	unsigned int i=0;

	while(i < len)
	{
		m_asi_decode.buf[m_asi_decode.index++] = ptr[i];			
		m_asi_decode.index = (m_asi_decode.index % EVB_ASCII_BUF_LEN);
		if(ptr[i] == '\n')
		{
			m_asi_decode.buf[m_asi_decode.index++] = '\0';
			m_asi_decode.index = (m_asi_decode.index % EVB_ASCII_BUF_LEN);
			if( (strstr((char*)m_asi_decode.buf, "M-")) || (strstr((char*)m_asi_decode.buf, "D-")) )
			{
				if((m_sample_flag)&&((m_cfg.test_i == TEST_DATA) || (m_cfg.test_i == TEST_SETRESET_SWITCH)))
				{
					int num = 0;

					num = sscanf((char*)m_asi_decode.buf, "%f,%f,%f,M-%d", &m_data.raw[0], &m_data.raw[1], &m_data.raw[2], &m_odr.samples2);
					if(num != 4)
					{
						int accuracy = 0;
						int temp = 0;
						float out[20];
						// num = sscanf((char*)m_asi_decode.buf, "%d,E,%f,%f,%f,M,%f,%f,%f,%d,MC,%f,%f,%f,I,%f,%f,%f,%f,%f,%f,D-%d\r\n", 
						// 			&accuracy, &out[0], &out[1], &out[2], &temp, &out[3], &out[4], &out[5], 
						// 			&out[6], &out[7], &out[8], &out[9], &out[10], &out[11], &out[12], &out[13], &out[14], &m_odr.samples2);
						num = sscanf((char*)m_asi_decode.buf, "D-%d,E%d,%f,%f,%f\r\n", &m_odr.samples2, &accuracy, &out[0], &out[1], &out[2]);
					}
					if((num == 4) || (num == 5))
					{
						//QST_PRINTF("sample=%d	%f,%f,%f\r\n", m_odr.samples2, m_data.raw[0], m_data.raw[1], m_data.raw[2]);
						if(log_button)
						{
							evb_write_log_file((char*)m_asi_decode.buf, strlen((char*)m_asi_decode.buf));
						}
						if(user_button)
						{
							if(user_path.IsEmpty())
							{
								SYSTEMTIME st;
								GetLocalTime(&st);

								user_path.Format(L".\\user_log\\%s-%S[%02x %02x %02x]-%02d%02d%02d.csv", m_id_info.GetBuffer(), m_cfg.name,m_cfg.v_id, m_cfg.w_id,m_cfg.d_id,st.wHour, st.wMinute, st.wSecond);
							}
							user_button = FALSE;
							evb_write_user_file(user_path, (char*)m_asi_decode.buf, strlen((char *)m_asi_decode.buf));
						}
#if defined(QST_CHART)
						if(m_dlg_chart && (m_odr.odr> 0.5f))
						{
							if(m_graphic_flag)
							{
								m_dlg_chart->dlg_chart_set_data(m_data.raw);
								if(m_odr.odr <= 20)
								{
									m_dlg_chart->dlg_chart_update();
								}
								else
								{
									m_chart_sample++;
									if( (m_odr.odr)&&(m_chart_sample > (unsigned int)(200/(1000/m_odr.odr))) )
									{
										m_chart_sample = 0;
										m_dlg_chart->dlg_chart_update();
									}
								}
							}
						}
#endif
					}
					else
					{
						QST_PRINTF("err num=%d\r\n", num);
					}
				}
				else if(m_cfg.test_i == TEST_SELFTEST)
				{
					int raw[3];
					int num = sscanf((char*)m_asi_decode.buf, "%d,%d,%d,M-%d", &raw[0], &raw[1], &raw[2], &m_odr.samples2);
					if(num == 4)
					{
						//QST_PRINTF("sample=%d %f,%f,%f\r\n", m_odr.samples2, raw[0], raw[1], raw[2]);
						if(log_button)
						{
							evb_write_log_file((char*)m_asi_decode.buf, strlen((char*)m_asi_decode.buf));
						}
					}
				}
				else if(m_cfg.test_i == TEST_POLL_ODR)
				{					
					sscanf((char*)m_asi_decode.buf, "M-%d", &m_odr.samples2);
				}

				m_asi_decode.index = 0;
				memset(m_asi_decode.buf, 0 , sizeof(m_asi_decode.buf));
			}
#if 0
			else if(strstr((char*)m_asi_decode.buf, "$:$"))
			{
				memset(m_cfg_info, 0, sizeof(m_cfg_info));
				memcpy(m_cfg_info, &m_asi_decode.buf[4], m_asi_decode.buf[3]);
				m_misc = 0x00;
				sscanf_s(m_cfg_info, "0x%02x 0x%02x 0x%02x 0x%04x", &m_misc, &m_cfg.v_id, &m_cfg.w_id, &m_cfg.d_id);
				QST_PRINTF("$receice info$: 0x%02x 0x%02x 0x%02x 0x%04x\r\n", m_misc, m_cfg.v_id, m_cfg.w_id, m_cfg.d_id);
				if(m_misc != m_cfg.chipid)
				{
					m_cfg.chipid = m_misc;
					evb_init_config_ui();
				}
				evb_open_log_file();
				m_asi_decode.index = 0;
				memset(m_asi_decode.buf, 0 , sizeof(m_asi_decode.buf));
			}
#endif
			else if((strstr((char*)m_asi_decode.buf, "$:i")) || (strstr((char*)m_asi_decode.buf, "$:$")))
			{
				//int len = (int)m_asi_decode.buf[3];
				int data[4];

				memset(m_cfg_info, 0, sizeof(m_cfg_info));
				memcpy(m_cfg_info, &m_asi_decode.buf[4], m_asi_decode.buf[3]);
				int ret = sscanf_s(m_cfg_info, "0x%02x 0x%02x 0x%02x 0x%04x", &data[0], &data[1], &data[2], &data[3]);
				QST_PRINTF("@receice info@:ret=%d %s\r\n", ret, m_cfg_info);
				//QST_PRINTF("@receice info@ chipid: 0x%02x 0x%02x 0x%02x 0x%04x\r\n", data[0], data[1], data[2], data[3]);
				if( (ret==4) && ((data[0]==0x80)||(data[0]==0x90)||(data[0]==0x91)||(data[0]==0x92)||(data[0]==0x20)) )
				{
					m_cfg.v_id = (unsigned char)data[1];
					m_cfg.w_id = (unsigned char)data[2];
					m_cfg.d_id = (unsigned short)data[3];
					if(data[0] != m_cfg.chipid)
					{
						m_cfg.chipid = (unsigned char)data[0];
						evb_init_config_ui();
					}
				}
				if(strstr((char*)m_asi_decode.buf, "$:$"))
				{
					evb_open_log_file();
				}
				m_asi_decode.index = 0;
				memset(m_asi_decode.buf, 0 , sizeof(m_asi_decode.buf));
			}
			else if(strstr((char*)m_asi_decode.buf, "$:w"))
			{
				if(user_path.IsEmpty())
				{
					SYSTEMTIME st;
					GetLocalTime(&st);
					user_path.Format(L".\\odr\\%s_%S_[%02x %02x %04x]_%04d%02d%02d%02d%02d%02d.txt", m_id_info.GetBuffer(),  m_cfg.report_mode?"ibi":"polling",
																									m_cfg.chipid, m_cfg.w_id, m_cfg.d_id, st.wYear,st.wMonth,st.wDay,st.wHour,st.wMinute,st.wSecond);
				}
				memset(m_asi_decode.buf, 0, sizeof(m_asi_decode.buf));
				int len = sprintf((char *)m_asi_decode.buf, "%s,%.2f\r\n", m_cfg_info, m_odr.odr);
				evb_write_user_file(user_path, (char *)m_asi_decode.buf, len);
				QST_PRINTF("config[%s] odr[%f] write file\r\n", m_cfg_info, m_odr.odr);
				memset(&m_odr, 0, sizeof(evb_sensor_odr));
				m_asi_decode.index = 0;				
				memset(m_asi_decode.buf, 0 , sizeof(m_asi_decode.buf));
			}
			m_asi_decode.index = 0;
		}

		i++;
	}
}

void CqstevbDlg::evb_open_log_file(void)
{
	if(log_button)
	{
		if(log_flag == FALSE)
		{
			SYSTEMTIME st;	 
			int len = 0;

			GetLocalTime(&st);
			log_path.Empty();
			evb_create_folder((char*)log_folder);
			if(m_cfg.name[0] != 0)
			{
				if(m_cfg.test_i == TEST_SELFTEST)
				{
					log_path.Format(L"%S\\%s_%S_%04d%02d%02d%02d%02d%02d.csv", log_folder,m_id_info.GetBuffer(),m_cfg.name,st.wYear,st.wMonth,st.wDay,st.wHour,st.wMinute,st.wSecond);
				}
				else if(m_at_enable)
				{
					log_path.Format(L"%S\\%d_%03d_%S@%02x@%04x_%S_%S_osr%S%S_%S_%04d%02d%02d%02d%02d%02d.csv", log_folder,m_cfg.id,m_at_i/*mag_at_item[0]*/,m_cfg.name,m_cfg.w_id,m_cfg.d_id,
																										m_set.range[m_cfg.range],m_set.odr[m_cfg.mode],m_set.osr1[m_cfg.osr1],m_set.osr2[m_cfg.osr2],m_set.sr[m_cfg.sr],
																										st.wYear,st.wMonth,st.wDay,st.wHour, st.wMinute, st.wSecond);
				}
				else
				{
					log_path.Format(L"%S\\%s_%S@%02x@%04x_%S_%S_osr%S%S_%S_%04d%02d%02d%02d%02d%02d.csv", log_folder, m_id_info.GetBuffer(), m_cfg.name, m_cfg.w_id, m_cfg.d_id,
																										m_set.range[m_cfg.range], m_set.odr[m_cfg.mode], m_set.osr1[m_cfg.osr1], m_set.osr2[m_cfg.osr2],m_set.sr[m_cfg.sr],
																										st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
				}
				log_flag = log_file.Open(log_path.GetString(), CFile::modeCreate | CFile::modeWrite);		//  | CFile::shareDenyNone
				QST_PRINTF("log file create ret=%d	name:%S\n", log_flag, log_path.GetBuffer());
			}
		}
	}
}

void CqstevbDlg::evb_write_log_file(char *str_buf, unsigned int len)
{
	if(log_flag == TRUE)
	{
#if defined(QST_FS_CACHE)
		log_file.Write(str_buf, len);
#else
		log_file.SeekToEnd();
		log_file.Write(str_buf, len);
		log_file.Close();
		log_flag = FALSE;
#endif
	}
}

void CqstevbDlg::evb_close_log_file(void)
{
	if(log_flag == TRUE)
	{
		log_file.Close();
		log_flag = FALSE;
	}
}

void CqstevbDlg::evb_write_user_file(CString path, char *str_buf, unsigned int len)
{
	BOOL evb_user_flag = FALSE;

	if(path.IsEmpty() || (len <= 0) || (str_buf == NULL))
	{
		QST_PRINTF("evb_write_user_file fail! para error!\n");
		return;
	}

	if((len > 0) && (str_buf != NULL))
	{
		evb_user_flag = user_file.Open(path.GetString(), CFile::modeNoTruncate | CFile::modeCreate | CFile::modeWrite); //|CFile::shareDenyNone
		QST_PRINTF("evb_write_user_file len=%d flag=%d name:%S\n", len, evb_user_flag, path.GetBuffer());
	}

	if(evb_user_flag == TRUE)
	{
		user_file.SeekToEnd();
		user_file.Write(str_buf, len);
		user_file.Close();
		evb_user_flag = FALSE;
	}
}

void CqstevbDlg::evb_open_config_file(void)
{
	m_csv_at.clear();
	if((m_cfg.chipid == 0x20)||(m_cfg.chipid == 0x10))
		m_csv_at = CSVReader("./mag_config_tmr.csv");
	else if((m_cfg.chipid == 0x90)||(m_cfg.chipid == 0x92))
		m_csv_at = CSVReader("./mag_config_qmc6309.csv");	
	else if(m_cfg.chipid == 0x91)
		m_csv_at = CSVReader("./mag_config_qmc6309v.csv");
	else if(m_cfg.chipid == 0x80)
		m_csv_at = CSVReader("./mag_config_qmc6308.csv");
		
	m_at_max = m_csv_at.rowsNum();
	QST_PRINTF("row:%d  col:%d \r\n", m_csv_at.rowsNum(), m_csv_at.colsNum());
}

void CqstevbDlg::evb_tx_func(unsigned char *buf, int len)
{
	if(m_SerialPort.isOpened())
	{
		m_SerialPort.writeData((char*)buf, len);
	}
	else
	{
		AfxMessageBox(_T("Please open com!"));
	}
}

void CqstevbDlg::evb_create_folder(char * name)
{
	int len = 0;
	CString folder = L".\\log";

	folder.Empty();
	folder.Format(L"%S", name);
	if(!PathIsDirectory(folder))
	{
		if(!CreateDirectory(folder, NULL ) ) 
		{
			QST_PRINTF("create folder fail!(%S)", folder.GetBuffer());
		}
	}
}

int CqstevbDlg::evb_write_reg(unsigned char slave, unsigned char reg, unsigned char value)
{
	unsigned char cmd[32];

	memset(cmd, 0, sizeof(cmd));
	int txlen = sprintf((char *)cmd, "w 0x%02x 0x%02x 0x%02x", slave, reg, value);
	evb_tx_func(cmd, txlen);

	return 1;
}

void CqstevbDlg::qst_evb_process_cmd(LPWSTR cmd)
{
    STARTUPINFO				si;
    PROCESS_INFORMATION		pi;

    ZeroMemory(&si, sizeof(si));
	ZeroMemory(&pi, sizeof(pi));
    si.cb = sizeof(si);
	si.dwFlags |= STARTF_USESHOWWINDOW;
	si.wShowWindow = SW_SHOWNORMAL;		// SW_HIDE, SW_MINIMIZE, SW_SHOWNOACTIVATE

    // 鍚姩涓€涓柊鐨刢md.exe杩涚▼骞惰繍琛屽懡浠?"your_command"
    if (!CreateProcessW(
						L".\\digital_power.exe",  
						cmd, // 鍛戒护琛屽弬鏁?
						NULL,             // 瀹夊叏灞炴€?
						NULL,             // 瀹夊叏灞炴€?
						FALSE,            // 缁ф壙handles
						0,                // 鍒涘缓鏍囧織
						NULL,             // 鏂扮殑鐜
						NULL,             // 褰撳墠鐩綍
						&si,              // 鍚姩淇℃伅
						&pi)              // 杩涚▼淇℃伅
    ) 
    {
        // 鍒涘缓澶辫触
        return;
    }
 
    // 绛夊緟CMD鎵ц瀹屾瘯
    WaitForSingleObject(pi.hProcess, INFINITE);
    // 鍏抽棴杩涚▼鍜岀嚎绋嬪彞鏌?
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);
}

void CqstevbDlg::evb_calc_sample_odr(unsigned int sample)
{
	if(sample > 0)
	{
		if(m_odr.samples1 == 0)
		{
			m_odr.tm_start = GetTickCount();
			m_odr.samples1 = sample;
			m_odr.samples_last = sample;
		}
		else
		{
			m_odr.tm_now = GetTickCount();
			m_odr.samples2 = sample;
			if(m_odr.tm_now > m_odr.tm_start)
			{
				if((m_odr.samples2 > m_odr.samples_last)&&(m_odr.samples2 > m_odr.samples1))
				{
					m_odr.odr = (float)(1000.f * (m_odr.samples2-m_odr.samples1) / (m_odr.tm_now-m_odr.tm_start));
					m_odr.samples_last = m_odr.samples2;
				}
			}
			else
			{
				QST_PRINTF("evb_calc_sample_odr error! (%d-%d)/(%lld-%lld)\r\n", m_odr.samples2,m_odr.samples1,m_odr.tm_now,m_odr.tm_start);
				memset(&m_odr, 0, sizeof(m_odr));
			}
		}
	}
}

BOOL CqstevbDlg::evb_check_digital(CString &str)
{
	#define IS_DIGITAL(c)	((c>='0')&&(c<='9'))
	#define IS_DOT(c)		(c=='.')

	char buff_out[32];
	int len = str.GetLength();
	int dot_num = 0;

	memset(buff_out, 0, sizeof(buff_out));
	WideCharToMultiByte(CP_ACP, 0, str, len,  buff_out, len, NULL, NULL);

	if(IS_DIGITAL(buff_out[0]))
	{
		char *p = &buff_out[1];
		while(*p)
		{
			if(IS_DIGITAL(*p))
			{
			}
			else if(IS_DOT(*p))
			{
				dot_num++;
				if(dot_num >= 2)
				{
					return FALSE;
				}
			}
			else
			{
				return FALSE;
			}
			p++;
		}
		
		return TRUE;
	}
	else
	{
		return FALSE;
	}


    return TRUE; // 鎵€鏈夊瓧绗﹂兘閫氳繃浜嗘鏌ワ紝杩斿洖true
}

void CqstevbDlg::evb_refresh_com(void)
{
	m_comset.connect = FALSE;
	m_ComId.ResetContent();
	vector<SerialPortInfo> m_portsList = CSerialPortInfo::availablePortInfos();
	TCHAR m_regKeyValue[255];
	m_comset.portNum = m_portsList.size();
	for (int i = 0; i < m_comset.portNum; i++)
	{
#ifdef UNICODE
		int iLength;
		const char * _char = m_portsList[i].portName.c_str();
		iLength = MultiByteToWideChar(CP_ACP, 0, _char, strlen(_char) + 1, NULL, 0);
		MultiByteToWideChar(CP_ACP, 0, _char, strlen(_char) + 1, m_regKeyValue, iLength);
#else
		strcpy_s(m_regKeyValue, 255, m_portsList[i].portName.c_str());
#endif
		m_ComId.AddString(m_regKeyValue);
	}
	m_ComId.SetCurSel(0);
	if (m_comset.portNum > 0)
		m_BtComOpen.SetWindowTextW(_T("Open"));
	else
		m_BtComOpen.SetWindowTextW(_T("Close"));
}

void CqstevbDlg::OnBnClickedComOpen()
{
	if(m_comset.portNum <= 0)
	{
		evb_refresh_com();
	}
	else if(m_comset.connect == FALSE)
	{
		CString port_name;

		port_name.Empty();
		m_comset.comid = m_ComId.GetCurSel();;
		m_ComId.GetWindowTextW(port_name);
#ifdef UNICODE
		m_comset.portName = CW2A(port_name.GetString());
#else
		m_comset.portName = port_name.GetBuffer();
#endif
		m_SerialPort.setMinByteReadNotify(1);
		m_SerialPort.init(m_comset.portName, m_comset.baud, itas109::Parity(m_comset.parity), itas109::DataBits(m_comset.data), itas109::StopBits(m_comset.stop),itas109::FlowControl(com_flow[0]), MAX_RX_FRAME_BUF);
		m_comset.connect = m_SerialPort.open();

		if(m_comset.connect)
		{
			m_SerialPort.clearError();
			m_SerialPort.readAllData((char*)m_rx_buf);
			memset(m_rx_buf, 0, MAX_RX_READY_BUF);
			m_string_rx.Empty();
			m_edit_rx.Clear();
			m_edit_rx.SetWindowTextW(m_string_rx);
			m_at_i = 1;

			QST_PRINTF("\n*****\nport:%S %d,%d,%d,%d\n", port_name,m_comset.baud,m_comset.parity,m_comset.data,m_comset.stop);
			Sleep(10);
			evb_tx_func((unsigned char*)"rst", 3);
			evb_create_folder(".\\log");
			evb_create_folder(".\\fifo_log");
			evb_create_folder(".\\user_log");
			evb_create_folder(".\\odr");
			evb_create_folder(".\\selftest");			
			evb_create_folder(".\\current");
		}
		else
		{
			AfxMessageBox(_T("open COM fail!"), MB_OK, MB_ICONERROR);
			return;
		}
	}
	else
	{
#if defined(QST_MEASURE_CURRENT)
		MeasureThreadStop();
#endif
		evb_close_log_file();
		KillTimer(EVB_TIMER_ID_1);
		KillTimer(EVB_TIMER_ID_2);
		KillTimer(EVB_TIMER_ID_3);
		KillTimer(EVB_TIMER_ID_4);

		evb_tx_func((unsigned char*)"rst", 3);
		m_comset.connect = FALSE;
		m_sample_flag = FALSE;
		m_at_i = 1;
		m_SerialPort.close();
	}

	m_BtComOpen.EnableWindow(0);
	SetTimer(EVB_TIMER_ID_2, 300, NULL);
}

void CqstevbDlg::OnCbnSelchangeComboComId()
{
	int index;

	index  = m_ComId.GetCurSel();
	if(index != CB_ERR)
	{
		CString port_name;
		m_comset.comid = index;
		m_ComId.GetWindowTextW(port_name);
#ifdef UNICODE
		m_comset.portName = CW2A(port_name.GetString());
#else
		m_comset.portName = port_name.GetBuffer();
#endif
		QST_PRINTF("comid = %d\n", m_comset.comid);
	}
}

void CqstevbDlg::OnCbnSelchangeComboComBrud()
{
	int index;

	index = m_ComBaud.GetCurSel();
	if(index != CB_ERR)
	{
		m_comset.baud = com_baudrate[index];
		QST_PRINTF("baud = %d\n", m_comset.baud);
	}
}

void CqstevbDlg::OnCbnSelchangeComboComParity()
{
	int index;

	index = m_ComParity.GetCurSel();
	if(index != CB_ERR)
	{
		m_comset.parity = index;
		QST_PRINTF("parity = %d\n", m_comset.parity);
	}
}

void CqstevbDlg::OnCbnSelchangeComboComData()
{
	int index;

	index = m_ComData.GetCurSel();
	if(index != CB_ERR)
	{
		m_comset.data = index;
		QST_PRINTF("data = %d\n", m_comset.data);
	}
}

void CqstevbDlg::OnCbnSelchangeComboComStop()
{
	int index;

	index = m_ComStop.GetCurSel();
	if(index != CB_ERR)
	{
		m_comset.stop = com_stop[index];
		QST_PRINTF("stop = %d\n", m_comset.stop);
	}
}

void CqstevbDlg::OnTimer(UINT_PTR nIDEvent)
{
	if(nIDEvent == EVB_TIMER_ID_1)
	{
		SYSTEMTIME			st;
		CString				m_StrStatus;

		evb_calc_sample_odr(m_odr.samples2);
		m_StrStatus.Empty();
		m_StrStatus.Format(L"[%S]", m_cfg_info);
		m_StatusBar.SetPaneText(0, m_StrStatus, TRUE);

		m_StrStatus.Empty();
		m_StrStatus.Format(L"%S samples[%d] rate[%.2f Hz]", m_cfg.name, m_odr.samples2, m_odr.odr);
		m_StatusBar.SetPaneText(1, m_StrStatus, TRUE);

		GetLocalTime(&st);
		m_StrStatus.Empty();
		m_StrStatus.Format(L"%04d/%02d/%02d %02d:%02d:%02d", st.wYear,st.wMonth,st.wDay,st.wHour,st.wMinute,st.wSecond);
		m_StatusBar.SetPaneText(2, m_StrStatus, TRUE);
		//QST_PRINTF("samples[%d] rate[%.2f(hz)]\n", m_odr.samples2, m_odr.odr);
		if(m_at_enable)
		{
			if(m_odr.samples2 >= (unsigned int)mag_at_item[6])
			{
				KillTimer(EVB_TIMER_ID_1);
				KillTimer(EVB_TIMER_ID_2);
				KillTimer(EVB_TIMER_ID_3);
				OnBnClickedButtonSampling();	// stop
				SetTimer(EVB_TIMER_ID_3, 5000, NULL);
			}
		}
	}
	else if(nIDEvent == EVB_TIMER_ID_2)
	{
		KillTimer(EVB_TIMER_ID_2);
		evb_refresh_ui();
	}
	else if(nIDEvent == EVB_TIMER_ID_3)
	{
		if(m_at_enable)
		{
			if(m_cfg.v_id != 0x00)
			{
				KillTimer(EVB_TIMER_ID_3);
				m_at_i++;
				OnBnClickedButtonSampling();
			}
		}
		else
		{
			KillTimer(EVB_TIMER_ID_3);	
		}
	}
#if defined(QST_MEASURE_CURRENT)
	else if(nIDEvent == EVB_TIMER_ID_4)
	{
		KillTimer(EVB_TIMER_ID_4);

		if(m_cfg.test_i == TEST_WORK_CURRENT)
		{
			unsigned char cmd[32];
			int txlen = 0;

			evb_tx_func((unsigned char*)"rst", 3);
			MeasureThreadStop();
			Sleep(500);

			while(m_at_i < m_at_max)
			{
				char *str;
				
				QST_PRINTF("row[%d]	", m_at_i);
				for(int i=0; i<(sizeof(mag_at_item)/sizeof(mag_at_item[0])); i++)
				{
					str = (char *)m_csv_at.getItem(m_at_i, i);
					mag_at_item[i] = atoi(str);
					//QST_PRINTF(" v[%d]=%d", i, mag_at_item[i]);
				}
				//QST_PRINTF("\r\n");
				if((mag_at_item[4]==0))		// osr2 = 0
				{
					QST_PRINTF("index[%d] odr[%d] osr[%d %d]\r\n", mag_at_item[0], mag_at_item[2], mag_at_item[3], mag_at_item[4]);
					break;
				}
				m_at_i++;
			}

			if(m_at_i >= m_at_max)
			{
				m_at_i = 1;
				AfxMessageBox(_T("Current Test Finish!"), MB_OK, MB_ICONINFORMATION);
				return;
			}
			else
			{
				m_at_i++;
				m_cfg.id = mag_at_item[0];
				//m_cfg.test_i = mag_at_item[1];
				m_cfg.mode = mag_at_item[2];
				m_cfg.osr1 = mag_at_item[3];
				m_cfg.osr2 = mag_at_item[4];
				m_cfg.delay = mag_at_item[5];
				m_cfg.max_count = mag_at_item[6];

				m_MagMode.SetCurSel(m_cfg.mode);
				m_MagOsr1.SetCurSel(m_cfg.osr1);
				m_MagOsr2.SetCurSel(m_cfg.osr2);
				m_MagRange.SetCurSel(m_cfg.range);
				m_MagSetReset.SetCurSel(m_cfg.sr);
				m_MagFifoMode.SetCurSel(m_cfg.fifo_mode);
				m_MagFifoWmk.SetCurSel(m_cfg.fifo_wmk);

				m_SerialPort.clearError();
				m_SerialPort.readAllData((char*)m_rx_buf);
				memset(m_rx_buf, 0, MAX_RX_READY_BUF);
				m_string_rx.Empty();
				m_edit_rx.Clear();
				m_edit_rx.SetWindowTextW(m_string_rx);
				//user_path.Empty();
				//memset(&m_odr, 0, sizeof(m_odr));
				m_sample_flag = TRUE;

				memset(cmd, 0, sizeof(cmd));
				txlen = sprintf((char *)cmd, "ma,%d,%d,%d,%d,%d", /*m_cfg.test_i*/TEST_DATA, m_cfg.fifo_en, (m_cfg.fifo_mode<<6), m_cfg.fifo_wmk, m_cfg.max_count);
				evb_tx_func(cmd, txlen);
				Sleep(UART_WRITE_DELAY);
				memset(cmd, 0, sizeof(cmd));
				txlen = sprintf((char *)cmd, "mb,%d,%d,%d,%d,%d", m_cfg.mode, m_cfg.range, m_cfg.osr1, m_cfg.osr2, m_cfg.sr);
				evb_tx_func(cmd, txlen);
				Sleep(UART_WRITE_DELAY);
				memset(cmd, 0, sizeof(cmd));
				txlen = sprintf((char *)cmd, "start,%d,%d", 0xffff, m_cfg.id);
				evb_tx_func(cmd, txlen);
				Sleep(UART_WRITE_DELAY);

				MeasureThreadStart();

				SetTimer(EVB_TIMER_ID_4, 10000, NULL);
			}
		}
	}
#endif

	CDialogEx::OnTimer(nIDEvent);
}

void CqstevbDlg::OnClose()
{
	//DWORD dwExitCode;
#ifdef DEBUG_CONSOLE
	CloseConsoleWindow();
#endif
	if(m_SerialPort.isOpened())
	{
		int iLen = m_SerialPort.readAllData((char *)m_rx_buf);
		m_SerialPort.close();
	}
	evb_close_log_file();

	KillTimer(EVB_TIMER_ID_1);
	KillTimer(EVB_TIMER_ID_2);

	if(m_dlg_chart)
	{
		delete m_dlg_chart;
		m_dlg_chart = NULL;
	}
#if defined(QST_MEASURE_CURRENT)
	if(m_dev_num)
	{
		MeasureThreadStop();
		SmartPower_OutputOff(1);
		if(SmartPower_DeviceClose())
		{
			m_dev_num = 0;
			QST_PRINTF("Power monitor close ok!\r\n");
			//MeasureThreadStop();
		}
	}
#endif

	CDialogEx::OnClose();
}

#if 0
HBRUSH CqstevbDlg::OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor)
{
	HBRUSH hbr = __super::OnCtlColor(pDC, pWnd, nCtlColor);

	if (nCtlColor == CTLCOLOR_STATIC)	//
	{
	//	m_font1.CreatePointFont(200, _T("Arial"), pDC);

		//LOGFONT lf = { 25 }; //
		//wcscpy(lf.lfFaceName, _T("Arial"));
		//lf.lfCharSet = GB2312_CHARSET;	//134; // #define GB2312_CHARSET 134
		//m_font1.CreateFontIndirect(&lf);

		//pDC->SelectObject(&m_font1);

		switch (pWnd->GetDlgCtrlID())
		{
		case IDC_EDIT_RX:
			pDC->SetTextColor(RGB(0, 135, 189));
			pDC->SetBkMode(TRANSPARENT); //
			break;
//		case IDC_STATIC_SENSOR_OUTPUT:
//			pDC->SetTextColor(RGB(255, 255, 0));
//			pDC->SetBkColor(RGB(255, 255, 255));
//			pDC->SelectObject(&sensorFont);
//			pDC->SetBkMode(TRANSPARENT);
//			return (HBRUSH)::GetStockObject(BLACK_BRUSH);
//			break;
		default:
			break;
		}
	}
	// TODO:  
	return hbr;
}
#endif

void CqstevbDlg::OnCbnSelchangeComboMode()
{
	int index;

	index  = m_MagMode.GetCurSel();
	if(index != CB_ERR)
	{
		if((m_cfg.chipid == 0x10)||(m_cfg.chipid == 0x20))
		{
			if(strcmp(m_set.odr[index], "1000Hz") == 0)
			{
				if((strcmp(m_set.osr1[m_cfg.osr1], "16") == 0) || (strcmp(m_set.osr1[m_cfg.osr1], "32") == 0))
				{			
					AfxMessageBox(_T("1000Hz can not support OSR1(16 or 32)"), MB_OK, MB_ICONERROR);
					m_cfg.osr1 = 0;
					m_MagOsr1.SetCurSel(m_cfg.osr1);
				}
				m_cfg.mode = index;
			}
			else if(strcmp(m_set.odr[index], "400Hz") == 0)
			{
				if(strcmp(m_set.osr1[m_cfg.osr1], "32") == 0)
				{			
					AfxMessageBox(_T("400Hz can not support OSR1(32)"), MB_OK, MB_ICONERROR);
					m_cfg.osr1 = 0;
					m_MagOsr1.SetCurSel(m_cfg.osr1);
				}
				m_cfg.mode = index;
			}
			else
			{
				m_cfg.mode = index;
			}
		}
		else
		{
			m_cfg.mode = index;
		}
	}
}

void CqstevbDlg::OnCbnSelchangeComboOsr1()
{
	int index;

	index  = m_MagOsr1.GetCurSel();
	if(index != CB_ERR)
	{
		if((m_cfg.chipid == 0x10)||(m_cfg.chipid == 0x20))
		{
			if(strcmp(m_set.odr[m_cfg.mode], "1000Hz") == 0)
			{
				if((strcmp(m_set.osr1[index], "16") == 0) || (strcmp(m_set.osr1[index], "32") == 0))
				{			
					AfxMessageBox(_T("1000Hz can not support OSR1(16 or 32)"), MB_OK, MB_ICONERROR);
					m_MagOsr1.SetCurSel(m_cfg.osr1);
				}
				else
				{
					m_cfg.osr1 = index;
				}
			}
			else if(strcmp(m_set.odr[m_cfg.mode], "400Hz") == 0)
			{
				if(strcmp(m_set.osr1[index], "32") == 0)
				{			
					AfxMessageBox(_T("400Hz can not support OSR1(32)"), MB_OK, MB_ICONERROR);
					m_MagOsr1.SetCurSel(m_cfg.osr1);
				}
				else
				{
					m_cfg.osr1 = index;
				}
			}
			else
			{
				m_cfg.osr1 = index;
			}
		}
		else
		{
			m_cfg.osr1 = index;
		}
	}
}

void CqstevbDlg::OnCbnSelchangeComboOsr2()
{
	int index;

	index  = m_MagOsr2.GetCurSel();
	if(index != CB_ERR)
	{
		m_cfg.osr2 = index;
	}
}

void CqstevbDlg::OnCbnSelchangeComboRange()
{
	int index;

	index  = m_MagRange.GetCurSel();
	if(index != CB_ERR)
	{
		m_cfg.range = index;
	}
}

void CqstevbDlg::OnSelchangeComboSetReset()
{
	int index;

	index  = m_MagSetReset.GetCurSel();
	if(index != CB_ERR)
	{
		m_cfg.sr = index;
	}
}

void CqstevbDlg::OnBnClickedButtonSampling()
{
	unsigned char cmd[32];
	char volt_str[16];
	int txlen = 0;

	log_button = m_BtSaveLog.GetCheck();

	if(!m_sample_flag)
	{
		if(m_cfg.v_id == 0x00)
		{
			QST_PRINTF("v_id=0x00 return\r\n");
			return;
		}
		m_edit_id.GetWindowTextW(m_id_info);
		if(m_comset.connect == FALSE)
		{
			AfxMessageBox(_T("Please open com!"));
			return;
		}
		if(m_id_info.GetLength() == 0)
		{
			AfxMessageBox(_T("Please input test ID!"), MB_OK, MB_ICONERROR);
			return;
		}

		if(m_cfg.fifo_wmk > 0)
		{
			m_cfg.fifo_en = 1;
		}
		else
		{
			m_cfg.fifo_en = 0;
		}

		if(m_cfg.fifo_en)
		{
			if((m_cfg.fifo_wmk < 1) || (m_cfg.fifo_wmk >= m_set.fifo_wmk_num))
			{
				AfxMessageBox(_T("FIFO WKM error"), MB_OK, MB_ICONERROR);
				return;
			}
		}

		memset(volt_str, 0, sizeof(volt_str));
#if defined(QST_DIGITAL_POWER)
		CString volt;
		m_edit_volt.GetWindowTextW(volt);
		if(evb_check_digital(volt))
		{
			m_DpVolt = (float)(_tstof(volt));
		}
		else
		{
			AfxMessageBox(_T("voltage input error!"), MB_OK, MB_ICONERROR);
			return;
		}
		if(m_at_enable) 			
			sprintf(volt_str, "-%.1f", m_DpVolt);
		else
			sprintf(volt_str, "\\%.1f", m_DpVolt);
		char *pos = volt_str;
		while(pos)
		{
			pos = strchr(pos, '.');
			if(pos)
			{
				*pos = 'p';
				pos = pos + 1;
			}
		}
#else
		if(m_user_vlot_flag)
			sprintf(volt_str, "3p3");
		else
			sprintf(volt_str, "1p8");
#endif
		memset(log_folder, 0, sizeof(log_folder));
		if(m_cfg.test_i == TEST_DATA)
		{
			if(m_cfg.fifo_en)
			{
				if(m_at_enable)
					sprintf((char *)log_folder, ".\\fifo_log\\%S_%s_%s", m_id_info.GetBuffer(), (m_cfg.report_mode?"ibi":"poll"), volt_str);
				else
					sprintf((char *)log_folder, ".\\fifo_log\\%s", volt_str);
			}
			else
			{
				if(m_at_enable)
					sprintf((char *)log_folder, ".\\log\\%S_%s_%s", m_id_info.GetBuffer(), (m_cfg.report_mode?"ibi":"poll"), volt_str);
				else
					sprintf((char *)log_folder, ".\\log\\%s", volt_str);
			}
			if(m_at_enable == FALSE)
			{
				evb_open_log_file();
			}
		}
		else if(m_cfg.test_i == TEST_SELFTEST)
		{
			sprintf((char *)log_folder, ".\\selftest\\%s", volt_str);
			evb_open_log_file();
		}
		else if(m_cfg.test_i == TEST_POLL_ODR)
		{
			sprintf((char *)log_folder, ".\\odr");
		}
		else if(m_cfg.test_i == TEST_WORK_CURRENT)
		{
			sprintf((char *)log_folder, ".\\current\\%s", volt_str);
			m_at_i = 1;
			SetTimer(EVB_TIMER_ID_4, 200, NULL);
			return;
		}

		if( (m_at_enable)&&(m_at_max > 1)&&((m_cfg.test_i == TEST_DATA)) )
		{
			if(m_at_i >= m_at_max)
			{
				m_at_i = 1;
				AfxMessageBox(_T("Auto Test Finish!"), MB_OK, MB_ICONINFORMATION);
				return;
			}
			else
			{
				char *str;
				for(int i=0; i<(sizeof(mag_at_item)/sizeof(mag_at_item[0])); i++)
				{
					str = (char *)m_csv_at.getItem(m_at_i, i);
					mag_at_item[i] = atoi(str);
					QST_PRINTF(" v[%d]=%d", i, mag_at_item[i]);
				}
				QST_PRINTF("\r\n");
			}

			m_cfg.id = mag_at_item[0];
			//m_cfg.test_i = mag_at_item[1];
			m_cfg.mode = mag_at_item[2];
			m_cfg.osr1 = mag_at_item[3];
			m_cfg.osr2 = mag_at_item[4];
			m_cfg.delay = mag_at_item[5];
			m_cfg.max_count = mag_at_item[6];

			m_MagMode.SetCurSel(m_cfg.mode);
			m_MagOsr1.SetCurSel(m_cfg.osr1);
			m_MagOsr2.SetCurSel(m_cfg.osr2);
			m_MagRange.SetCurSel(m_cfg.range);
			m_MagSetReset.SetCurSel(m_cfg.sr);
			m_MagFifoMode.SetCurSel(m_cfg.fifo_mode);
			m_MagFifoWmk.SetCurSel(m_cfg.fifo_wmk);
		}

		m_SerialPort.clearError();
		m_SerialPort.readAllData((char*)m_rx_buf);
		memset(m_rx_buf, 0, MAX_RX_READY_BUF);
		m_string_rx.Empty();
		m_edit_rx.Clear();
		m_edit_rx.SetWindowTextW(m_string_rx);
		user_path.Empty();
		memset(&m_odr, 0, sizeof(m_odr));
		m_sample_flag = TRUE;

		if(m_user_cmd_flag)
		{
			CString str_cmd;
			char cmd[16];
			int txlen = 0;

			memset(cmd, 0, sizeof(cmd));
			m_edit_cmd.GetWindowTextW(str_cmd);
			txlen = sprintf((char *)cmd, "%S", str_cmd.GetBuffer());
			sscanf_s(cmd, "%d", &m_cfg.delay);
			QST_PRINTF("user send cmd = %s delay=%d\r\n", cmd, m_cfg.delay);
		}
		else
		{
			m_cfg.delay = 0;
		}

		memset(cmd, 0, sizeof(cmd));
		txlen = sprintf((char *)cmd, "ma,%d,%d,%d,%d,%d", m_cfg.test_i, m_cfg.fifo_en, (m_cfg.fifo_mode<<6), m_cfg.fifo_wmk, m_cfg.max_count);
		evb_tx_func(cmd, txlen);
		Sleep(UART_WRITE_DELAY);
		memset(cmd, 0, sizeof(cmd));
		txlen = sprintf((char *)cmd, "mb,%d,%d,%d,%d,%d", m_cfg.mode, m_cfg.range, m_cfg.osr1, m_cfg.osr2, m_cfg.sr);
		evb_tx_func(cmd, txlen);
		Sleep(UART_WRITE_DELAY);
		memset(cmd, 0, sizeof(cmd));
		txlen = sprintf((char *)cmd, "start,%d,%d,%d", m_cfg.delay,m_cfg.id,m_cfg.report_mode);
		evb_tx_func(cmd, txlen);
		Sleep(UART_WRITE_DELAY);

		SetTimer(EVB_TIMER_ID_1, 300, NULL);
	}
	else
	{
		KillTimer(EVB_TIMER_ID_1);
		KillTimer(EVB_TIMER_ID_2);
		//m_dlg_chart->ShowWindow(SW_HIDE);
		evb_close_log_file();
		m_ComPause = TRUE;
		m_SerialPort.readAllData((char*)m_rx_buf);
		m_SerialPort.clearError();
		evb_tx_func((unsigned char*)"rst", 3);
		Sleep(UART_WRITE_DELAY);
		//evb_tx_func((unsigned char*)"rst", 3);
		//Sleep(UART_WRITE_DELAY);
		//evb_tx_func((unsigned char*)"rst", 3);
		//evb_tx_func((unsigned char*)"rst", 3);
		//evb_tx_func((unsigned char*)"rst", 3);
		m_ComPause = FALSE;
		memset(m_rx_buf, 0, MAX_RX_READY_BUF);
		m_sample_flag = FALSE;
		m_cfg.v_id = m_cfg.w_id = 0x00;
		m_cfg.d_id = 0;
		memset(&m_odr, 0, sizeof(m_odr));
	}

	m_BtSampling.EnableWindow(0);
	SetTimer(EVB_TIMER_ID_2, 300, NULL);
}

void CqstevbDlg::OnBnClickedButtonUser1()
{
	// TODO:
	//CSubDialog	*dlg = new CSubDialog();

	if(m_dlg_chart)
	{
		if(m_graphic_flag == FALSE)
		{
			m_dlg_chart->ShowWindow(SW_SHOW);
			m_graphic_flag = TRUE;
			m_BtUser1.SetWindowTextW(_T("Hide graphic"));
		}
		else
		{
			m_dlg_chart->ShowWindow(SW_HIDE);
			m_graphic_flag = FALSE;
			m_BtUser1.SetWindowTextW(_T("Show graphic"));
		}
	}
}

#if defined(QST_MEASURE_CURRENT)
UINT CqstevbDlg::MeasureThreadWork(LPVOID pParam)
{
#define MEASURE_FREQ		1200
	// 鑾峰彇楂樼簿搴﹁鏃跺櫒棰戠巼
	LARGE_INTEGER freq;	
	LARGE_INTEGER lastTime, currentTime;
	CqstevbDlg* pDlg = (CqstevbDlg*)pParam;

	QueryPerformanceFrequency(&freq);
	LONGLONG targetTicks = (LONGLONG)(freq.QuadPart / (MEASURE_FREQ));

	pDlg->m_thread_count = 0;
	QueryPerformanceCounter(&lastTime);
	SmartPower_GetStatus(pDlg->m_dev_value);
	QueryPerformanceCounter(&currentTime);
	QST_PRINTF("%lld [%lld] %lld\r\n", freq.QuadPart, (currentTime.QuadPart-lastTime.QuadPart), targetTicks);
	pDlg->m_dev_avg_value[0] = pDlg->m_dev_value[0];
	pDlg->m_dev_avg_value[1] = pDlg->m_dev_value[1];

	while(pDlg->m_thread_run && pDlg->m_dev_num)
	{
		QueryPerformanceCounter(&lastTime);
		QueryPerformanceCounter(&currentTime);

		while((currentTime.QuadPart - lastTime.QuadPart) < targetTicks)
		{
			QueryPerformanceCounter(&currentTime);
		}

		//QST_PRINTF("MeasureThreadWork timeout %d\r\n", pDlg->m_thread_count++);

		SmartPower_GetStatus(pDlg->m_dev_value);
		QST_PRINTF("%d\r\n", (int)(pDlg->m_dev_value[1]*1000000));
		pDlg->m_dev_avg_value[0] = (pDlg->m_dev_value[0] + pDlg->m_dev_avg_value[0])/2.0f;
		pDlg->m_dev_avg_value[1] = (pDlg->m_dev_value[1] + pDlg->m_dev_avg_value[1])/2.0f;

		pDlg->m_thread_count++;
		if((pDlg->m_thread_count % (MEASURE_FREQ/4)) == 0)
		{
			QST_PRINTF("[%d][vol:%f	curr:%f]\r\n",pDlg->m_thread_count, pDlg->m_dev_avg_value[0],pDlg->m_dev_avg_value[1]);
		}

	}

	return 0;
}

void CqstevbDlg::MeasureThreadStart()
{
	// 鍒涘缓宸ヤ綔绾跨▼
	if((m_pThread == NULL) && (m_dev_num > 0))
	{
		m_pThread = AfxBeginThread(&MeasureThreadWork, this, THREAD_PRIORITY_NORMAL);
	}
	if (m_pThread)
	{
		m_thread_run = TRUE;
		m_thread_count = 0;
		memset(m_dev_value, 0, sizeof(m_dev_value));
		memset(m_dev_avg_value, 0, sizeof(m_dev_avg_value));
	}
}

void CqstevbDlg::MeasureThreadStop()
{
	m_thread_run = FALSE;
	if (m_pThread)
	{
		//WaitForSingleObject(m_pThread->m_hThread, INFINITE);
		m_pThread = NULL;
	}
}
#endif

BOOL CqstevbDlg::PreTranslateMessage(MSG* pMsg)
{
	if (pMsg->message == WM_KEYDOWN)
	{
		CWnd *pCtrl = CWnd::GetFocus();
		int focusId = pCtrl->GetDlgCtrlID();

		if(focusId == IDC_EDIT_DIGITAL_POWER_VOLT)
		{
			CDialog::PreTranslateMessage(pMsg);
			return TRUE;
		}

		switch (pMsg->wParam)
		{
		case 'd':
		case 'D':
			if(m_cfg.test_i == TEST_DATA)
			{
				QST_PRINTF("user key d press!(cat data)\r\n");
				user_button = 1;
			}
			break;
		case VK_RETURN:
			if(focusId == IDC_EDIT_TEST_ID)
			{
				QST_PRINTF("edit id focus!\r\n");
				OnBnClickedButtonSampling();
			}
#if 0
			else if(focusId == IDC_EDIT_CUSTOM_CMD)
			{			
				CString str_cmd;
				unsigned char cmd[32];
				int txlen = 0;

				memset(cmd, 0, sizeof(cmd));
				m_edit_cmd.GetWindowTextW(str_cmd);
				txlen = sprintf((char *)cmd, "%S", str_cmd.GetBuffer());
				evb_tx_func(cmd, txlen);
				QST_PRINTF("user send cmd = %s\r\n", cmd);
				if((m_sample_flag) && (strcmp((char*)cmd, "rst") == 0) )
				{
					OnBnClickedButtonSampling();
				}
			}
#endif
			return TRUE;
			break;
		default:
			break;
		}
	}
	else
	{
		return FALSE;
	}
	CDialog::PreTranslateMessage(pMsg);

	return TRUE;
}

void CqstevbDlg::OnCbnSelchangeComboFifoMode()
{
	int index;

	index  = m_MagFifoMode.GetCurSel();
	if(index != CB_ERR)
	{
		m_cfg.fifo_mode = index;
	}
}

void CqstevbDlg::OnCbnSelchangeComboFifoWmk()
{
	int index;

	index  = m_MagFifoWmk.GetCurSel();
	if(index != CB_ERR)
	{
		m_cfg.fifo_wmk = index;
	}
}

void CqstevbDlg::OnCbnSelchangeComboTestI()
{
	int index;

	index  = m_MagTestI.GetCurSel();
	if(index != CB_ERR)
	{
		m_cfg.test_i = mag_test_item[index].test_i;
#if defined(QST_TEST_SR)
		if(m_cfg.test_i == TEST_SETRESET_SWITCH)
			m_cfg.delay = 800;
		else
			m_cfg.delay = 0;
#endif
	}
	QST_PRINTF("select %d test item: %d\r\n", index, m_cfg.test_i);
}

void CqstevbDlg::OnBnClickedButtonSetReset()
{
	// TODO: 鍦ㄦ娣诲姞鎺т欢閫氱煡澶勭悊绋嬪簭浠ｇ爜
	unsigned char cmd[32];
	int txlen = 0;
	
	memset(cmd, 0, sizeof(cmd));
	txlen = sprintf((char *)cmd, "set,%d", m_cfg.sr_num);
	
	evb_tx_func(cmd, txlen);
}

void CqstevbDlg::OnBnClickedButtonUser()
{
	// TODO: 鍦ㄦ娣诲姞鎺т欢閫氱煡澶勭悊绋嬪簭浠ｇ爜
	if(m_sample_flag)
	{
		user_button = TRUE;		
		QST_PRINTF("user key press(cat data)!\r\n");
	}
}

void CqstevbDlg::OnCbnSelchangeComboDigitalPowerEn()
{
	// TODO: 鍦ㄦ娣诲姞鎺т欢閫氱煡澶勭悊绋嬪簭浠ｇ爜
	int index;

	index  = m_DpEn.GetCurSel();
	if(index != CB_ERR)
	{
		m_DpEnSel = index;
	}
	QST_PRINTF("Digital Power Sel: %d\r\n", m_DpEnSel);
}

void CqstevbDlg::OnBnClickedButtonDigitalPowerSet()
{
	// TODO: 鍦ㄦ娣诲姞鎺т欢閫氱煡澶勭悊绋嬪簭浠ｇ爜
	CString cmd;
	CString volt;

	m_edit_volt.GetWindowTextW(volt);
	if(evb_check_digital(volt))
	{
		m_DpVolt = (float)(_tstof(volt));
		QST_PRINTF("digital_power set Volt=%.2f\r\n", m_DpVolt);

		cmd.Empty();
		cmd.Format(L"digital_power ctrl 1 %f %d", m_DpVolt, m_DpEnSel);
		qst_evb_process_cmd(cmd.GetBuffer());
	}
	else
	{
		AfxMessageBox(_T("voltage input error!"), MB_OK, MB_ICONERROR);
	}
}

void CqstevbDlg::OnCbnSelchangeComboFifoReportMode()
{
	// TODO: 鍦ㄦ娣诲姞鎺т欢閫氱煡澶勭悊绋嬪簭浠ｇ爜
	int index;

	index  = m_MagReport.GetCurSel();
	if(index != CB_ERR)
	{
		m_cfg.report_mode = index;
	}
}
