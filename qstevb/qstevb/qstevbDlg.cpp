
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

//#include "smartpower.h"
#include "QPlotRunner.h"

#define WM_FLASH_COMPLETE			(WM_USER + 100)
#define WM_TIMER_CBK				(WM_USER + 101)

#define UART_WRITE_DELAY			30	//100	//20	// uint millsecond
#define MAX_RX_FRAME_BUF			1024*2
#define MAX_RX_READY_BUF			MAX_RX_FRAME_BUF

static char rx_ready_buf[MAX_RX_READY_BUF] = {0};
static char	m_cfg_info[80] = {0};
static char	m_custom_info[80] = {0};
//static int mag_cfg_item[7] = {0,0,0,0,0,0,0};

static const long com_baudrate[] = {4800,9600,14400,19200,38400,56000,57600,115200,256000,460800,512000,921600};
static const int com_databit[] = {8, 7, 6, 5};		// bit
static const int com_parity[] = {0, 1, 2, 3, 4};	// none; odd; even; mark; Space
static const int com_stop[] = {0, 1, 2};			// one, oneandhalf, two
static const int com_flow[] = {0, 1, 2};			// none, hardware, software
static const UINT indicators[] = {IDS_VERSION, IDS_SAMPLES, IDS_TIME};

static const short mag_svvt_array[] = {1000, 2000, 4000};

static const char* mag_sensor_type[] = { "qmc6309h", "qmc6309v", "qmc6g00h", "qmc6308" };

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

static const char* mag_setreset_maestro[] = {"sr0", "s1", "r1"};
static const char* mag_setreset_qmc6309x[] = {"sr1", "s1", "r1", "sr0"};
static const char* mag_setreset_qmc6308[] = {"sr1", "s1", "sr0"};

static const char* mag_report_poll_ibi[] = { "poll", "ibi" };
static const char* mag_report_poll[] = { "poll" };

static const char* mag_zdbl[] = {"zd0", "zd1"};

static const char* mag_fifo_mode[] = {"bypass", "fifo", "stream",};

static const char* mag_fifo_level_16[] = {"0","1","2","3","4","5","6","7","8","9","10","11","12","13","14","15","16"};
static const char* mag_fifo_level_8[] = {"0","1","2","3","4","5","6","7","8"};

// cmd thread
struct CmdThreadParam
{
	HANDLE hProcess;   // 要等待的进程句柄
	HWND   hWnd;       // 接收完成消息的窗口句柄
};
// cmd thread

static struct
{
	int		test_i;
	char	*str;
} mag_test_item[8];

static CSubDialog *m_dlg_chart = NULL;

#ifdef _DEBUG
#define new DEBUG_NEW
#define DEBUG_CONSOLE
#endif
//#define DEBUG_CONSOLE

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

	DDX_Control(pDX, IDC_COMBO_ID, m_MagId);
	DDX_Control(pDX, IDC_COMBO_SENSOR_TYPE, m_MagSensorId);
	DDX_Control(pDX, IDC_COMBO_MODE, m_MagMode);
	DDX_Control(pDX, IDC_COMBO_OSR1, m_MagOsr1);
	DDX_Control(pDX, IDC_COMBO_OSR2, m_MagOsr2);
	DDX_Control(pDX, IDC_COMBO_RANGE, m_MagRange);
	DDX_Control(pDX, IDC_COMBO_SET_RESET, m_MagSetReset);
	DDX_Control(pDX, IDC_COMBO_FIFO_MODE, m_MagFifoMode);
	DDX_Control(pDX, IDC_COMBO_FIFO_WMK, m_MagFifoWmk);
	DDX_Control(pDX, IDC_COMBO_TEST_I, m_MagTestI);
	DDX_Control(pDX, IDC_COMBO_FIFO_REPORT_MODE, m_MagReport);
	
	//DDX_Control(pDX, IDC_COMBO_DIGITAL_POWER_EN, m_DpEn);
	

	DDX_Control(pDX, IDC_EDIT_RX, m_edit_rx);
	DDX_Control(pDX, IDC_EDIT_TEST_ID, m_edit_id);
	DDX_Control(pDX, IDC_EDIT_CUSTOM_CMD, m_edit_cmd);
	//DDX_Control(pDX, IDC_EDIT_DIGITAL_POWER_VOLT, m_edit_volt);

	DDX_Control(pDX, IDC_CHECK_SAVE_LOG, m_BtSaveLog);
	DDX_Control(pDX, IDC_CHECK_OUT_PIC, m_BtOutPic);
	DDX_Control(pDX, IDC_BUTTON_SAMPLING, m_BtSampling);
	DDX_Control(pDX, IDC_BUTTON_IMU_CALI, m_BtUser1);	
	DDX_Control(pDX, IDC_BUTTON_USER, m_BtCatData);
}

BEGIN_MESSAGE_MAP(CqstevbDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON_COM_OPEN, &CqstevbDlg::OnBnClickedOpen)
	ON_CBN_SELCHANGE(IDC_COMBO_COM_ID, &CqstevbDlg::OnCbnSelchangeComboComId)
	ON_CBN_SELCHANGE(IDC_COMBO_COM_BRUD, &CqstevbDlg::OnCbnSelchangeComboComBrud)
	ON_CBN_SELCHANGE(IDC_COMBO_COM_JYW, &CqstevbDlg::OnCbnSelchangeComboComParity)
	ON_CBN_SELCHANGE(IDC_COMBO_COM_SJW, &CqstevbDlg::OnCbnSelchangeComboComData)
	ON_CBN_SELCHANGE(IDC_COMBO_COM_TZW, &CqstevbDlg::OnCbnSelchangeComboComStop)
	ON_WM_TIMER()
	ON_WM_CLOSE()
	ON_WM_CTLCOLOR()
	ON_CBN_SELCHANGE(IDC_COMBO_MODE, &CqstevbDlg::OnCbnSelchangeComboMode)
	ON_CBN_SELCHANGE(IDC_COMBO_SENSOR_TYPE, &CqstevbDlg::OnCbnSelchangeComboSensorType)
	ON_CBN_SELCHANGE(IDC_COMBO_OSR1, &CqstevbDlg::OnCbnSelchangeComboOsr1)
	ON_CBN_SELCHANGE(IDC_COMBO_OSR2, &CqstevbDlg::OnCbnSelchangeComboOsr2)
	ON_CBN_SELCHANGE(IDC_COMBO_RANGE, &CqstevbDlg::OnCbnSelchangeComboRange)
	ON_BN_CLICKED(IDC_BUTTON_SAMPLING, &CqstevbDlg::OnBnClickedButtonSampling)
	ON_BN_CLICKED(IDC_BUTTON_IMU_CALI, &CqstevbDlg::OnBnClickedButtonImuCali)
	ON_CBN_SELCHANGE(IDC_COMBO_SET_RESET, &CqstevbDlg::OnSelchangeComboSetReset)
	ON_CBN_SELCHANGE(IDC_COMBO_FIFO_MODE, &CqstevbDlg::OnCbnSelchangeComboFifoMode)
	ON_CBN_SELCHANGE(IDC_COMBO_FIFO_WMK, &CqstevbDlg::OnCbnSelchangeComboFifoWmk)
	ON_CBN_SELCHANGE(IDC_COMBO_TEST_I, &CqstevbDlg::OnCbnSelchangeComboTestI)
	ON_BN_CLICKED(IDC_BUTTON_USER, &CqstevbDlg::OnBnClickedButtonUser)
	ON_CBN_SELCHANGE(IDC_COMBO_FIFO_REPORT_MODE, &CqstevbDlg::OnCbnSelchangeComboFifoReportMode)
	ON_MESSAGE(WM_FLASH_COMPLETE, OnExeComplete)
	ON_CBN_SELCHANGE(IDC_COMBO_ID, &CqstevbDlg::OnCbnSelchangeComboId)
	ON_BN_CLICKED(IDC_CHECK_OUT_PIC, &CqstevbDlg::OnBnClickedCheckOutPic)
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
#if defined(QST_CHART)
			strCutom.LoadString(IDS_STRING_GPAPHIC);
			pSysMenu->AppendMenu(MF_STRING, IDM_GRAPHIC, strCutom);
#endif
			strCutom.LoadString(IDS_STRING_AUTO_TEST);
			pSysMenu->AppendMenu(MF_STRING, IDM_AUTO_TEST, strCutom);

			strCutom.LoadString(IDS_STRING_CUSTOM_CMD);
			pSysMenu->AppendMenu(MF_STRING, IDM_USER_CMD, strCutom);

			strCutom.LoadString(IDS_STRING_CUSTOM_UPDATE);
			pSysMenu->AppendMenu(MF_STRING, IDM_USER_UPDATE, strCutom);

			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
		//	pSysMenu->CheckMenuItem(IDM_AUTO_TEST, MF_CHECKED | MF_BYCOMMAND);
	}

	m_StatusBar.Create(this);
	m_StatusBar.SetIndicators(indicators, 4);
	m_StatusBar.SetPaneInfo(0, IDS_VERSION, SBPS_STRETCH, 300);
	m_StatusBar.SetPaneInfo(1, IDS_SAMPLES, SBPS_STRETCH, 350);
	m_StatusBar.SetPaneInfo(2, IDS_STRING_CUSTOM_UPDATE, SBPS_STRETCH, 200);
	m_StatusBar.SetPaneInfo(3, IDS_TIME, SBPS_STRETCH, 150);
	
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

	m_usbdev_detect = FALSE;
	m_usbdev_type = DEVICE_NONE;
	m_dev_connect = FALSE;
	//m_hTimerQueue = NULL;
	//m_hTimer = NULL;

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
	m_cfg.chipid = EVB_INIT_CHIPID;
	m_cfg.v_id = 0x00;
	m_cfg.w_id = 0x00;
	m_cfg.d_id = 0x0000;
	m_cfg.l_id = 0x0000;
	//m_cfg.sr_num = 10;
	m_cfg.report_mode = 0;
	m_cfg.max_count = 0x7fffffff;
	m_cfg.id = 0;
	m_cfg.id_max = QST_SENSOR_NUM;
	m_cfg.at_test = 0;
	m_cfg.cfg_i = 1;
	m_cfg.cfg_max = 1;
	m_cfg.odr = 0;
	strcpy(m_cfg.name , "qmcXXX");
	m_cfg.mat[0][0] = m_cfg.mat[1][1] = m_cfg.mat[2][2] = 1.0f;

	m_sample_flag = FALSE;
	m_chart_sample = 0;
	m_graphic_flag = FALSE;	
	log_flag = log_button = FALSE;	
	log_each_flag = TRUE;
	log_temp = FALSE;
	user_button = FALSE;

	m_user_cmd_flag = FALSE;
	memset(&m_odr, 0, sizeof(m_odr));
	user_path.Empty();
	log_path.Empty();

	evb_init_common_ui();
	evb_init_config_ui();
	//evb_refresh_ui();
	qst_imu_cali_init(m_cfg.bias_a, m_cfg.bias_g);
	if(m_dlg_chart == NULL)
	{
		m_dlg_chart = new CSubDialog();
		m_dlg_chart->Create(CSubDialog::IDD, /*this*/GetDesktopWindow());
		m_dlg_chart->ModifyStyle(WS_SYSMENU, 0);
		m_dlg_chart->ShowWindow(SW_HIDE);
	}
	m_SerialPort.readReady.connect(this, &CqstevbDlg::evb_rx_func);

	SetIcon(m_hIcon, TRUE);

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
		if(m_cfg.at_test == 0)
		{
			m_cfg.at_test = 1;
			m_cfg.cfg_i = 1;
			m_cfg.id = 0;
			pSysMenu->CheckMenuItem(IDM_AUTO_TEST, MF_CHECKED | MF_BYCOMMAND);
		}
		else
		{
			m_cfg.at_test = 0;
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
	else if((nID & 0xFFFF) == IDM_USER_UPDATE)
	{
		evb_open_bin_path();
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

UINT ProcessWaitThread(LPVOID pParam)
{
	CmdThreadParam* p = (CmdThreadParam*)pParam;
	WaitForSingleObject(p->hProcess, INFINITE);

	DWORD dwExitCode = 0;
	GetExitCodeProcess(p->hProcess, &dwExitCode);
	CloseHandle(p->hProcess);

	::PostMessage(p->hWnd, WM_FLASH_COMPLETE, (WPARAM)dwExitCode, 0);

	delete p;   // 清理堆内存
	return 0;
}

LRESULT CqstevbDlg::OnExeComplete(WPARAM wParam, LPARAM lParam)
{
	DWORD dwExitCode = (DWORD)wParam;
	if (dwExitCode == 0)
		AfxMessageBox(_T("烧录成功！"));
	else
		AfxMessageBox(_T("烧录失败，退出码: ") + CString(std::to_wstring(dwExitCode).c_str()));

	// 这里可以添加后续操作，比如启用按钮、更新界面等
	return 0;
}

void CqstevbDlg::evb_open_bin_path(void)
{
	CFileDialog dlg(TRUE, NULL, NULL, OFN_HIDEREADONLY, _T("所有文件 (*.*)|*.bin||"), this);
	dlg.m_ofn.lpstrTitle = _T("Select bin file");

	if (dlg.DoModal() == IDOK)
	{
		CString cmd_l;
		STARTUPINFO				si;
		PROCESS_INFORMATION		pi;

		m_BinPath = dlg.GetPathName();
		m_BinPath.Replace(_T('\\'), _T('/'));   // 替换为正斜杠
		QST_PRINTF("select file %S\r\n", (LPCTSTR)m_BinPath);

		CString strWorkDir = _T(".\\firmware\\openocd\\bin");
		CString strExePath = _T(".\\firmware\\openocd\\bin\\openocd.exe");
		cmd_l.Format(L"\"%s\" -f ..\\openocd\\scripts\\interface\\stlink.cfg -f ..\\openocd\\scripts\\target\\stm32f4x.cfg -c \"program \"%s\" 0x08000000 verify reset exit\"", (LPCTSTR)strExePath, (LPCTSTR)m_BinPath);

		ZeroMemory(&si, sizeof(si));
		ZeroMemory(&pi, sizeof(pi));
		si.cb = sizeof(si);
		si.dwFlags |= STARTF_USESHOWWINDOW;
		si.wShowWindow = SW_SHOWNORMAL;		// SW_HIDE, SW_MINIMIZE, SW_SHOWNOACTIVAT

		BOOL bSuccess = CreateProcessW(
			NULL,
			cmd_l.GetBuffer(), // 命令行参数
			NULL,             // 安全属性
			NULL,             // 安全属性
			FALSE,            // 继承handles
			0,                // 创建标志
			NULL,             // 新的环境
			strWorkDir,            // 当前目录
			&si,              // 启动信息
			&pi);              // 进程信息

		cmd_l.ReleaseBuffer();
		if (bSuccess)
		{
			//CloseHandle(pi.hProcess);
			//CloseHandle(pi.hThread);
			QST_PRINTF("cmd done\r\n");
		}
		else
		{
			AfxMessageBox(_T("启动烧录进程失败！"));
			return;
		}

		CmdThreadParam* pParam = new CmdThreadParam;
		pParam->hProcess = pi.hProcess; // 进程句柄
		pParam->hWnd = m_hWnd;          // 当前窗口句柄

		CWinThread* pThread = AfxBeginThread(ProcessWaitThread, pParam, THREAD_PRIORITY_NORMAL, 0, CREATE_SUSPENDED);
		if (pThread == NULL)
		{
			CloseHandle(pi.hProcess);
			CloseHandle(pi.hThread);
			delete pParam;
			AfxMessageBox(_T("创建监控线程失败！"));
			return;
		}
		CloseHandle(pi.hThread);
		pThread->ResumeThread();
		// AfxMessageBox(_T("正在烧录，请稍候..."));
	}

}

void CqstevbDlg::evb_refresh_ui(void)
{
	if(m_dev_connect)
	{
		m_BtComOpen.EnableWindow(1);
		m_ComId.EnableWindow(0);
		m_ComBaud.EnableWindow(0);
		m_ComParity.EnableWindow(0);
		m_ComData.EnableWindow(0);
		m_ComStop.EnableWindow(0);
		m_BtSampling.EnableWindow(1);
		m_BtCatData.EnableWindow(1);
		m_MagSensorId.EnableWindow(1);

		m_BtComOpen.SetWindowTextW(_T("关闭"));
	}
	else
	{
		m_BtComOpen.EnableWindow(1);
		m_ComId.EnableWindow(1);
		m_ComBaud.EnableWindow(1);
		m_ComParity.EnableWindow(1);
		m_ComData.EnableWindow(1);
		m_ComStop.EnableWindow(1);
		m_BtSampling.EnableWindow(0);
		m_BtCatData.EnableWindow(0);
		m_MagSensorId.EnableWindow(0);

		if(m_dlg_chart && m_graphic_flag)
		{
			m_dlg_chart->ShowWindow(SW_HIDE);
			m_graphic_flag = FALSE;
		}
		m_BtComOpen.SetWindowTextW(_T("打开"));
	}

	if(m_sample_flag && m_dev_connect)
	{
		m_edit_id.EnableWindow(0);
		m_edit_cmd.EnableWindow(1);
		m_MagId.EnableWindow(0);
		m_MagSensorId.EnableWindow(0);
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
		m_BtOutPic.EnableWindow(0);
		m_BtSampling.SetWindowTextW(_T("停止"));
	}
	else if(m_dev_connect)
	{
		m_edit_id.EnableWindow(1);
		m_edit_cmd.EnableWindow(1);
		if (m_cfg.id_max > 1)
			m_MagId.EnableWindow(1);
		else
			m_MagId.EnableWindow(0);

		m_MagSensorId.EnableWindow(1);
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
		m_BtOutPic.EnableWindow(1);
		m_BtCatData.EnableWindow(0);
		m_BtSampling.SetWindowTextW(_T("开始"));
	}
	else
	{
		m_edit_id.EnableWindow(0);
		m_edit_cmd.EnableWindow(0);
		m_MagId.EnableWindow(0);
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
		m_BtOutPic.EnableWindow(0);
		m_BtCatData.EnableWindow(0);
		m_BtSampling.SetWindowTextW(_T("开始"));
	}

	if (m_cfg.test_i == TEST_ANGLE)
		m_BtUser1.ShowWindow(SW_SHOW);
	else
		m_BtUser1.ShowWindow(SW_HIDE);

	if(m_dev_connect == FALSE)
		m_BtUser1.EnableWindow(0);
	else if (m_sample_flag)
		m_BtUser1.EnableWindow(0);
	else
		m_BtUser1.EnableWindow(1);
	
	log_button = m_BtSaveLog.GetCheck();
}

void CqstevbDlg::evb_init_common_ui(void)
{
	int result = 0;
	CString comStr;

	m_dev_connect = FALSE;

#if 0
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
		m_BtComOpen.SetWindowTextW(_T("打开"));
	else
		m_BtComOpen.SetWindowTextW(_T("关闭"));
#else
	evb_refresh_com();
#endif

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
#if 1
	m_edit_font.CreatePointFont(100, _T("Microsoft YaHei"));		//微软雅黑
#else
	m_edit_font.CreateFont(0,	// 35
							0,                // 宽度 (0表示自动匹配)
							0,                // 文本倾斜角度
							0,                // 文本基线倾斜角度
							FW_NORMAL,        // 粗细 (FW_NORMAL=常规, FW_BOLD=粗体)
							FALSE,             // **斜体 (TRUE表示启用)**
							FALSE,            // 下划线
							FALSE,            // 删除线
							DEFAULT_CHARSET,  // 字符集
							OUT_DEFAULT_PRECIS,
							CLIP_DEFAULT_PRECIS,
							DEFAULT_QUALITY,
							DEFAULT_PITCH | FF_DONTCARE,
							_T("Microsoft YaHei"));
#endif
	m_edit_rx.SetFont(&m_edit_font);
	m_edit_rx.SetLimitText(-1);
	m_edit_id.SetWindowTextW(_T(EVB_INIT_INFO));
	m_edit_cmd.SetWindowTextW(_T("0"));

	m_BtSaveLog.SetCheck(FALSE);
	m_BtSaveLog.ShowWindow(SW_SHOW);
	m_BtOutPic.SetCheck(FALSE);
	m_BtOutPic.ShowWindow(SW_SHOW);
	//GetDlgItem(IDC_STATIC_GROUP_DIGITAL_POWER)->ShowWindow(SW_HIDE);
	m_BtUser1.ShowWindow(SW_SHOW);	// SW_SHOW SW_HIDE

	if(m_user_cmd_flag)
		m_edit_cmd.ShowWindow(SW_SHOW);
	else
		m_edit_cmd.ShowWindow(SW_HIDE);

	if (m_cfg.id_max > 1)
		m_MagId.EnableWindow(1);
	else
		m_MagId.EnableWindow(0);
}

void CqstevbDlg::evb_refresh_config_ui(void)
{
	CString magStr;

	m_MagId.Clear();
	m_MagSensorId.Clear();
	m_MagMode.Clear();
	m_MagOsr1.Clear();
	m_MagOsr2.Clear();
	m_MagRange.Clear();
	m_MagSetReset.Clear();
	m_MagFifoMode.Clear();
	m_MagFifoWmk.Clear();
	m_MagReport.Clear();
	m_MagTestI.Clear();
	m_MagId.ResetContent();
	m_MagSensorId.ResetContent();
	m_MagMode.ResetContent();
	m_MagOsr1.ResetContent();
	m_MagOsr2.ResetContent();
	m_MagRange.ResetContent();
	m_MagSetReset.ResetContent();
	m_MagFifoMode.ResetContent();
	m_MagFifoWmk.ResetContent();
	m_MagReport.ResetContent();
	m_MagTestI.ResetContent();

	for (int i = 0; i < m_cfg.id_max; i++)
	{
		magStr.Empty();
		magStr.Format(L"%d", i);
		m_MagId.AddString(magStr.GetBuffer());
	}
	m_MagId.SetCurSel(m_cfg.id);

	for (int i = 0; i < sizeof(mag_sensor_type)/sizeof(mag_sensor_type[0]); i++)
	{
		magStr.Empty();
		magStr.Format(L"%S", mag_sensor_type[i]);
		m_MagSensorId.AddString(magStr.GetBuffer());
	}
	if(m_cfg.chipid == 0x90)
		m_MagSensorId.SetCurSel(0);
	else if(m_cfg.chipid == 0x91)
		m_MagSensorId.SetCurSel(1);
	else if(m_cfg.chipid == 0x20)
		m_MagSensorId.SetCurSel(2);
	else if(m_cfg.chipid == 0x80)
		m_MagSensorId.SetCurSel(3);

	if (m_set.odr)
	{
		for (int i = 0; i < m_set.odr_num; i++)
		{
			magStr.Empty();
			magStr.Format(L"%S", m_set.odr[i]);
			m_MagMode.AddString(magStr.GetBuffer());
		}
		m_MagMode.SetCurSel(m_cfg.mode);
	}
	if (m_set.osr1)
	{
		for (int i = 0; i < m_set.osr1_num; i++)
		{
			magStr.Empty();
			magStr.Format(L"%S", m_set.osr1[i]);
			m_MagOsr1.AddString(magStr.GetBuffer());
		}
		m_MagOsr1.SetCurSel(m_cfg.osr1);
	}
	if (m_set.osr2)
	{
		for (int i = 0; i < m_set.osr2_num; i++)
		{
			magStr.Empty();
			magStr.Format(L"%S", m_set.osr2[i]);
			m_MagOsr2.AddString(magStr.GetBuffer());
		}
		m_MagOsr2.SetCurSel(m_cfg.osr2);
	}
	if (m_set.range)
	{
		for (int i = 0; i < m_set.range_num; i++)
		{
			magStr.Empty();
			magStr.Format(L"%S", m_set.range[i]);
			m_MagRange.AddString(magStr.GetBuffer());
		}
		m_MagRange.SetCurSel(m_cfg.range);
	}
	if (m_set.sr)
	{
		for (int i = 0; i < m_set.sr_num; i++)
		{
			magStr.Empty();
			magStr.Format(L"%S", m_set.sr[i]);
			m_MagSetReset.AddString(magStr.GetBuffer());
		}
		m_MagSetReset.SetCurSel(m_cfg.sr);
	}

	if (m_set.fifo_mode)
	{
		for (int i = 0; i < m_set.fifo_mode_num; i++)
		{
			magStr.Empty();
			magStr.Format(L"%S", m_set.fifo_mode[i]);
			m_MagFifoMode.AddString(magStr.GetBuffer());
		}
		m_MagFifoMode.SetCurSel(m_cfg.fifo_mode);
	}
	if (m_set.fifo_wmk)
	{
		for (int i = 0; i < m_set.fifo_wmk_num; i++)
		{
			magStr.Empty();
			magStr.Format(L"%S", m_set.fifo_wmk[i]);
			m_MagFifoWmk.AddString(magStr.GetBuffer());
		}
		m_MagFifoWmk.SetCurSel(m_cfg.fifo_wmk);
	}

	if (m_set.report)
	{
		for (int i = 0; i < m_set.report_num; i++)
		{
			magStr.Empty();
			magStr.Format(L"%S", m_set.report[i]);
			m_MagReport.AddString(magStr.GetBuffer());
		}
		m_MagReport.SetCurSel(m_cfg.report_mode);
	}

	for (int i = 0; i < sizeof(mag_test_item) / sizeof(mag_test_item[0]); i++)
	{
		if (mag_test_item[i].test_i == 0xff)
		{
			break;
		}
		magStr.Empty();
		magStr.Format(L"%S", mag_test_item[i].str);
		m_MagTestI.AddString(magStr.GetBuffer());
	}
	m_MagTestI.SetCurSel(m_cfg.test_i);
}

void CqstevbDlg::evb_init_config_ui(void)
{
	int item_i = 0;

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

	mag_test_item[item_i].test_i = TEST_DATA;
	mag_test_item[item_i++].str = "read data";
	mag_test_item[item_i].test_i = TEST_SELFTEST;
	mag_test_item[item_i++].str = "read selftest";

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

		m_set.report = mag_report_poll_ibi;
		m_set.report_num = sizeof(mag_report_poll_ibi) / sizeof(mag_report_poll_ibi[0]);

		m_cfg.mode = 4; 			// 0:hpf 4:100
		m_cfg.osr1 = 3; 			// 8
		m_cfg.osr2 = 0; 			// 1 2 4 8
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

		m_set.report = mag_report_poll_ibi;
		m_set.report_num = sizeof(mag_report_poll_ibi) / sizeof(mag_report_poll_ibi[0]);

		m_cfg.mode = 0;				// hpf
		m_cfg.osr1 = 3;				// 8
		m_cfg.osr2 = 2;				// 4
		m_cfg.range = 0;			// 32Gs
		m_cfg.sr = 0;				// set-reset on
	}
	else if(m_cfg.chipid == 0x80)
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

		m_set.report = mag_report_poll;
		m_set.report_num = sizeof(mag_report_poll) / sizeof(mag_report_poll[0]);

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
	m_cfg.test_i = TEST_DATA;

	if(m_usbdev_detect == FALSE)
	{
	#if defined(QST_TEST_ODR)
		mag_test_item[item_i].test_i = TEST_POLL_ODR;
		mag_test_item[item_i++].str = "test odr";
	#endif
	#if defined(QST_TEST_CURRENT)
		mag_test_item[item_i].test_i = TEST_WORK_CURRENT;
		mag_test_item[item_i++].str = "test current";
	#endif
	#if defined(QST_TEST_REG)
		mag_test_item[item_i].test_i = TEST_WRITE_READ_REGISTER;
		mag_test_item[item_i++].str = "test reg w/r";
	#endif
	#if defined(QST_TEST_ANGLE)
		mag_test_item[item_i].test_i = TEST_ANGLE;
		mag_test_item[item_i++].str = "test angle";
	#endif
	}
	mag_test_item[item_i].test_i = 0xff;
	mag_test_item[item_i++].str = "null";

	evb_refresh_config_ui();
	evb_refresh_ui();
	evb_open_config_file();
}

void CqstevbDlg::evb_rx_func(void)
{
	int iLen = m_SerialPort.readAllData((char *)m_rx_buf);
	if((iLen > 0) && (iLen<MAX_RX_READY_BUF))
	{
		m_rx_buf[iLen] = '\0';
		//QST_PRINTF("%s",m_rx_buf);
		evb_rx_display_buf(m_rx_buf, iLen);
		//if ((m_cfg.test_i==TEST_DATA)||(m_cfg.test_i==TEST_POLL_ODR)||(m_cfg.test_i==TEST_SELFTEST)||(m_cfg.test_i==TEST_ANGLE))
		//{
			evb_rx_decode_ascii_buf(m_rx_buf, iLen);
		//}
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
			if( ( (strstr((char*)m_asi_decode.buf, "M-")) || (strstr((char*)m_asi_decode.buf, "$A")) ) && m_sample_flag )
			{
				int num = 0;
				if(m_cfg.test_i==TEST_DATA)	// 
				{
					if(m_cfg.report_mode == 1)
						num = sscanf((char*)m_asi_decode.buf, "%d,%d,%d,M-%d", &m_data.m_raw[0], &m_data.m_raw[1], &m_data.m_raw[2], &m_odr.samples2);
					else
						num = sscanf((char*)m_asi_decode.buf, "%f,%f,%f,M-%d", &m_data.m_out[0], &m_data.m_out[1], &m_data.m_out[2], &m_odr.samples2);
					if(num == 4)
					{
						//QST_PRINTF("sample=%d	%f,%f,%f\r\n", m_odr.samples2, m_data.m_out[0], m_data.m_out[1], m_data.m_out[2]);
						if(log_button)
						{
							evb_write_log_file((char*)m_asi_decode.buf, strlen((char*)m_asi_decode.buf));
							if (m_odr.samples2 == 1)
							{
								int dot_n = std::count((char*)m_asi_decode.buf, (char*)m_asi_decode.buf + strlen((char*)m_asi_decode.buf), ',');
								//QST_PRINTF("dot_n=%d\r\n", dot_n);
								if (dot_n > 4)
									log_temp = TRUE;
								else
									log_temp = FALSE;
							}
						}
						if(user_button)
						{
							if(user_path.IsEmpty())
							{
								SYSTEMTIME st;
								GetLocalTime(&st);

								user_path.Format(L".\\%S_user_log\\", m_cfg.name);
								if (!PathIsDirectory(user_path))
								{
									evb_make_full_path(user_path);
								}
								user_path.Empty();
								user_path.Format(L".\\%S_user_log\\U%d_%S_%d#%d#%d_%S_%S_osr%S%S_%S_%04d%02d%02d%02d%02d%02d.csv", m_cfg.name,
									m_cfg.id, m_cfg.name, m_cfg.w_id, m_cfg.l_id, m_cfg.d_id,
									m_set.range[m_cfg.range], m_set.odr[m_cfg.mode], m_set.osr1[m_cfg.osr1], m_set.osr2[m_cfg.osr2], m_set.sr[m_cfg.sr],
									st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);

							}
							user_button = FALSE;
							evb_write_user_file(user_path, (char*)m_asi_decode.buf, strlen((char *)m_asi_decode.buf));
							snprintf(m_custom_info, sizeof(m_custom_info), "%.1f %.1f %.1f", m_data.m_out[0], m_data.m_out[1], m_data.m_out[2]);
						}
#if defined(QST_CHART)
						if(m_dlg_chart && (m_odr.odr> 0.5f))
						{
							if(m_graphic_flag)
							{
								m_dlg_chart->dlg_chart_set_data(m_data.m_out);
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
				else if (m_cfg.test_i == TEST_ANGLE)
				{
					int accuary = 0;
					num = sscanf_s((char*)m_asi_decode.buf, "$A%d,%f,%f,%f,M,%f,%f,%f,%f,%f,%f,G,%f,%f,%f,%f,%f,%f", 
										&accuary, &m_data.e_out[0], &m_data.e_out[1], &m_data.e_out[2],
										&m_data.m_out[0], &m_data.m_out[1], &m_data.m_out[2], &m_data.mc_out[0], &m_data.mc_out[1], &m_data.mc_out[2],
										&m_data.a_out[0], &m_data.a_out[1], &m_data.a_out[2], &m_data.g_out[0], &m_data.g_out[1], &m_data.g_out[2]);
					if (num == 16)
					{
						m_odr.samples2++;
					}
					if (m_cfg.imu_cali_flag)
					{
						int cali_ret = 0;

						cali_ret = qst_algo_imu_cali(&m_data.a_out[0], &m_data.g_out[0]);
						if (cali_ret == 1)
						{
							m_cfg.imu_cali_flag = FALSE;
							OnBnClickedButtonSampling();
							for (int i = 0; i < 3; i++)
							{
								char itemStr[32];

								memset(itemStr, 0, sizeof(itemStr));
								snprintf(itemStr, sizeof(itemStr), "%f", m_cfg.bias_a[i]);
								m_csv_para.setItem(3, i, itemStr);

								memset(itemStr, 0, sizeof(itemStr));
								snprintf(itemStr, sizeof(itemStr), "%f", m_cfg.bias_g[i]);
								m_csv_para.setItem(4, i, itemStr);
							}
							m_csv_para.saveToFile("./config/parameter.csv");
							QST_PRINTF("IMU cali done, %f	%f	%f	%f	%f	%f\r\n", m_cfg.bias_a[0], m_cfg.bias_a[1], m_cfg.bias_a[2], m_cfg.bias_g[0], m_cfg.bias_g[1], m_cfg.bias_g[2]);
							SetTimer(EVB_TIMER_ID_4, 300, NULL);
						}
					}
					else
					{
						if (log_button)
						{
							evb_write_log_file((char*)m_asi_decode.buf, strlen((char*)m_asi_decode.buf));
						}
						if (user_button)
						{
							if (user_path.IsEmpty())
							{
								SYSTEMTIME st;
								GetLocalTime(&st);
								user_path.Format(L".\\%S_angle\\", m_cfg.name);
								if (!PathIsDirectory(user_path))
								{
									evb_make_full_path(user_path);
								}
								user_path.Format(L".\\%S_angle\\%S_%d#%d#%d_%S_%S_osr%S%S_%S_%04d%02d%02d%02d%02d%02d.csv", m_cfg.name,
									m_cfg.name, m_cfg.w_id, m_cfg.l_id, m_cfg.d_id,
									m_set.range[m_cfg.range], m_set.odr[m_cfg.mode], m_set.osr1[m_cfg.osr1], m_set.osr2[m_cfg.osr2], m_set.sr[m_cfg.sr],
									st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
								//user_path.Format(L".\\angle\\%s-%S[%02x %02x %02x]-%02d%02d%02d.csv", m_id_info.GetBuffer(), m_cfg.name, m_cfg.v_id, m_cfg.w_id, m_cfg.d_id, st.wHour, st.wMinute, st.wSecond);
							}
							user_button = FALSE;
							evb_write_user_file(user_path, (char*)m_asi_decode.buf, strlen((char*)m_asi_decode.buf));
							snprintf(m_custom_info, sizeof(m_custom_info), "%.1f %.1f %.1f", m_data.e_out[0], m_data.e_out[1], m_data.e_out[2]);
						}
					}
					//QST_PRINTF("$A-%d-%d,%0.1f,%0.1f,%0.1f,M,%0.1f,%0.1f,%0.1f,%0.1f,%0.1f,%0.1f,I,%0.2f,%0.2f,%0.2f,%0.4f,%0.4f,%0.4f\r\n", 
					//					accuary, m_odr.samples2, m_data.e_out[0], m_data.e_out[1], m_data.e_out[2],
					//					m_data.m_out[0], m_data.m_out[1], m_data.m_out[2], m_data.mc_out[0], m_data.mc_out[1], m_data.mc_out[2],
					//					m_data.a_out[0], m_data.a_out[1], m_data.a_out[2], m_data.g_out[0], m_data.g_out[1], m_data.g_out[2]);
				}

				m_asi_decode.index = 0;
				memset(m_asi_decode.buf, 0 , sizeof(m_asi_decode.buf));
			}
			else if((strstr((char*)m_asi_decode.buf, "$:i")) )
			{
				int data[5];

				memset(m_cfg_info, 0, sizeof(m_cfg_info));
				memcpy(m_cfg_info, &m_asi_decode.buf[4], m_asi_decode.buf[3]);
				int ret = sscanf_s(m_cfg_info, "0x%02x 0x%02x 0x%02x 0x%04x 0x%04x", &data[0], &data[1], &data[2], &data[3], &data[4]);
				//QST_PRINTF("@receice info@:ret=%d %s\r\n", ret, m_cfg_info);
				if( (ret==5) && ((data[0]==0x80)||(data[0]==0x90)||(data[0]==0x91)||(data[0]==0x92)||(data[0]==0x20)) )
				{
					if (m_cfg.chipid != data[0])
					{
						//evb_init_config_ui();
						CString	disStr;

						disStr.Empty();
						disStr.Format(L"型号选择错误，当前ID为: 0x%02x", data[0]);
						AfxMessageBox(disStr.GetBuffer(), MB_OK, MB_ICONERROR);
						OnBnClickedButtonSampling();
					}
					else
					{
						//m_cfg.chipid = (unsigned char)data[0];
						m_cfg.v_id = (unsigned char)data[1];
						m_cfg.w_id = (unsigned char)data[2];
						m_cfg.d_id = (unsigned short)data[3];
						m_cfg.l_id = (unsigned short)data[4];
						if (log_each_flag)
						{
							log_each_flag = FALSE;
							evb_open_log_file();
						}
					}
					QST_PRINTF("U%d chipid:0x%02x ver:0x%02x wafer:0x%02x pos:0x%04x lot:0x%04x\r\n", m_cfg.id,m_cfg.chipid,m_cfg.v_id,m_cfg.w_id,m_cfg.d_id,m_cfg.l_id);
				}
				else
				{
					QST_PRINTF("U%d chipid:0x%02x sensor not found!\r\n", m_cfg.id,data[0]);
					m_odr.samples2 = (unsigned int)m_cfg.max_count;
					m_cfg.cfg_i = m_cfg.cfg_max;
					m_sample_flag = TRUE;
					//OnTimer(EVB_TIMER_ID_1);
				}

				m_asi_decode.index = 0;
				memset(m_asi_decode.buf, 0 , sizeof(m_asi_decode.buf));
			}
			else
			{
				CString info_file;

				info_file.Format(L".\\info.txt");
				evb_write_user_file(info_file, (char*)m_asi_decode.buf, strlen((char*)m_asi_decode.buf));
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
			evb_create_folder();
			if(m_cfg.name[0] != 0)
			{
				if(m_cfg.test_i == TEST_SELFTEST)
				{
					log_path.Format(L"%S\\U%d_%S_%d#%d#%d_%04d%02d%02d%02d%02d%02d.csv",	log_folder, m_cfg.id, m_cfg.name,m_cfg.w_id, m_cfg.l_id, m_cfg.d_id,
																							st.wYear,st.wMonth,st.wDay,st.wHour,st.wMinute,st.wSecond);
				}
				else if (m_cfg.test_i == TEST_POLL_ODR)
				{
					log_path.Format(L"%S\\U%d_%S_%d#%d#%d_%04d%02d%02d%02d%02d%02d.txt", log_folder, m_cfg.id,m_cfg.name, m_cfg.w_id, m_cfg.l_id, m_cfg.d_id, 
																								st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
				}
				else if (m_cfg.test_i == TEST_ANGLE)
				{
					log_path.Format(L"%S\\%S_%d#%d#%d_%S_%S_osr%S%S_%S_%04d%02d%02d%02d%02d%02d.csv",
						log_folder, m_cfg.name, m_cfg.w_id, m_cfg.l_id, m_cfg.d_id,
						m_set.range[m_cfg.range], m_set.odr[m_cfg.mode], m_set.osr1[m_cfg.osr1], m_set.osr2[m_cfg.osr2], m_set.sr[m_cfg.sr],
						st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
				}
				else if (m_cfg.test_i == TEST_DATA)
				{
					if (m_cfg.at_test)
					{
						log_path.Format(L"%S\\U%d_%03d_%S_%d#%d#%d_%S_%S_osr%S%S_%S_%04d%02d%02d%02d%02d%02d.csv", 
																					log_folder, m_cfg.id, m_cfg.cfg_i, m_cfg.name, m_cfg.w_id, m_cfg.l_id, m_cfg.d_id,
																					m_set.range[m_cfg.range], m_set.odr[m_cfg.mode], m_set.osr1[m_cfg.osr1], m_set.osr2[m_cfg.osr2], m_set.sr[m_cfg.sr],
																					st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
					}
					else
					{
						log_path.Format(L"%S\\U%d_%S_%d#%d#%d_%S_%S_osr%S%S_%S_%04d%02d%02d%02d%02d%02d.csv",
																					log_folder, m_cfg.id, m_cfg.name, m_cfg.w_id, m_cfg.l_id, m_cfg.d_id,
																					m_set.range[m_cfg.range], m_set.odr[m_cfg.mode], m_set.osr1[m_cfg.osr1], m_set.osr2[m_cfg.osr2], m_set.sr[m_cfg.sr],
																					st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
					}
				}
				else
				{
					log_path.Format(L"%S\\%S_%d#%d#%d_%04d%02d%02d%02d%02d%02d.txt", log_folder, m_cfg.name, m_cfg.w_id, m_cfg.l_id, m_cfg.d_id,
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
	if((log_flag == TRUE)&&(str_buf)&&(len>0))
	{
		log_file.Write(str_buf, len);
	}
}

void CqstevbDlg::evb_write_log_file_ext(char* str_buf, unsigned int len)
{
	if ((str_buf) && (len > 0))
	{
		if (log_flag == FALSE)
		{
			log_flag = log_file.Open(log_path.GetString(), CFile::modeCreate | CFile::modeWrite | CFile::modeNoTruncate);
		}
		if (log_flag)
		{
			log_file.SeekToEnd();
			log_file.Write(str_buf, len);
			log_file.Close();
			log_flag = FALSE;
		}
	}
}

void CqstevbDlg::evb_close_log_file(void)
{
	if(log_flag == TRUE)
	{
		log_file.Close();
		log_flag = FALSE;
		if ((log_path.GetLength() > 0) && (m_BtOutPic.GetCheck() == TRUE))
		{
			evb_plot_file(log_path, 1);
		}
	}
}

void CqstevbDlg::evb_clear_user_file(CString path)
{
	BOOL evb_user_flag = FALSE;

	evb_user_flag = user_file.Open(path.GetString(), CFile::modeWrite);
	if (evb_user_flag)
	{
		user_file.SetLength(0);
		user_file.Close();
		evb_user_flag = FALSE;
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
		//QST_PRINTF("evb_write_user_file len=%d flag=%d name:%S\n", len, evb_user_flag, path.GetBuffer());
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
	m_config_tbl.clear();
	if((m_cfg.chipid == 0x20)||(m_cfg.chipid == 0x10))
		m_config_tbl = CSVReader("./config/mag_config_tmr.csv");
	else if((m_cfg.chipid == 0x90)||(m_cfg.chipid == 0x92))
		m_config_tbl = CSVReader("./config/mag_config_qmc6309.csv");
	else if(m_cfg.chipid == 0x91)
		m_config_tbl = CSVReader("./config/mag_config_qmc6309v.csv");
	else if(m_cfg.chipid == 0x80)
		m_config_tbl = CSVReader("./config/mag_config_qmc6308.csv");
		
	m_cfg.cfg_max = m_config_tbl.rowsNum();
	QST_PRINTF("config table: row:%d  col:%d \r\n", m_config_tbl.rowsNum(), m_config_tbl.colsNum());

	const char* str = NULL;
	int i, j;

	m_csv_para.clear();
	m_csv_para = CSVReader("./config/parameter.csv");
	if (m_csv_para.rowsNum() > 0)
	{
		QST_PRINTF("soft mag para:\r\n");
		for (i = 0; i < 3; i++)
		{
			for (j = 0; j < 3; j++)
			{
				str = m_csv_para.getItem(i, j);
				if (str != nullptr)
				{
					m_cfg.mat[i][j] = (float)atof(str);
					QST_PRINTF("mat[%d][%d]=%0.5f ", i, j, m_cfg.mat[i][j]);
				}
			}
			QST_PRINTF("\r\n");
		}
		m_cfg.bias_a[0] = (float)atof(m_csv_para.getItem(3, 0));
		m_cfg.bias_a[1] = (float)atof(m_csv_para.getItem(3, 1));
		m_cfg.bias_a[2] = (float)atof(m_csv_para.getItem(3, 2));
		m_cfg.bias_g[0] = (float)atof(m_csv_para.getItem(4, 0));
		m_cfg.bias_g[1] = (float)atof(m_csv_para.getItem(4, 1));
		m_cfg.bias_g[2] = (float)atof(m_csv_para.getItem(4, 2));
		QST_PRINTF("acc bias: %0.5f %0.5f %0.5f\r\n", m_cfg.bias_a[0], m_cfg.bias_a[1], m_cfg.bias_a[2]);
		QST_PRINTF("gyr bias: %0.5f %0.5f %0.5f\r\n", m_cfg.bias_g[0], m_cfg.bias_g[1], m_cfg.bias_g[2]);
	}
	 
	CString info_file;
	info_file.Format(L".\\info.txt");
	evb_clear_user_file(info_file);
}

void CqstevbDlg::evb_tx_func(unsigned char *buf, int len)
{
#if defined(QST_USE_DEVICE)
	if (m_usbdev_detect)
	{
		return;
	}
#endif
	if(m_SerialPort.isOpened())
	{
		m_SerialPort.writeData((char*)buf, len);
	}
	else
	{
		AfxMessageBox(_T("Please open com!"));
	}
}

void CqstevbDlg::evb_plot_file(CString path, int mode)
{
	QPlotResult r;
	CString args;
	//CString pngFile = L"";

	if (m_cfg.at_test)
	{
		if (log_temp)
			args.Format(L"-f \"%s\" -m col -c 0,1,2,4 --skip 0 --tags Mx,My,Mz,T --units uT,uT,uT,°C, --var --lwidth 0.15 --msize 1.5 --dpi 200", path.GetString());
		else
			args.Format(L"-f \"%s\" -m col -c 0,1,2 --skip 0 --tags Mx,My,Mz --units uT,uT,uT, --var --lwidth 0.15 --msize 1.5 --dpi 200", path.GetString());
	}
	else
	{
		if (log_temp)
			args.Format(L"-f \"%s\" -m col -c 0,1,2,4 --skip 0 --tags Mx,My,Mz,T --units uT,uT,uT,°C, --var --lwidth 0.15 --msize 2 --dpi 300 --show", path.GetString());
		else
			args.Format(L"-f \"%s\" -m col -c 0,1,2 --skip 0 --tags Mx,My,Mz --units uT,uT,uT, --var --lwidth 0.15 --msize 2 --dpi 200 --show", path.GetString());
	}

	if (mode == 0)
	{
		//第 3 个参数是超时时间(ms)。单文件 exe 首次启动要自解压, 给 60 秒比较稳。
		if (!QPlotRun(L".\\extra\\qplot.exe", args.GetString(), r, 30000))
		{
			AfxMessageBox(_T("无法启动 qplot.exe, 请检查路径"));
		}
	}
	else if (mode == 1)
	{
		QPlotRunAsync(L".\\extra\\qplot.exe", args.GetString());
	}
}

void CqstevbDlg::evb_make_full_path(const CString& path)
{
	CString str = path;
	str.Replace('/', '\\');

	// 去掉末尾的反斜杠
	str.TrimRight('\\');

	CString dir;
	int pos = 0;
	CString token = str.Tokenize(_T("\\"), pos);
	while (!token.IsEmpty())
	{
		if (dir.IsEmpty())
			dir = token;
		else
			dir += _T("\\") + token;

		_tmkdir(dir);   // 已存在会返回 -1，忽略即可
		token = str.Tokenize(_T("\\"), pos);
	}
}

void CqstevbDlg::evb_create_folder(void)
{
	int len = 0;
	CString folder = L".\\log";

	memset(log_folder, 0, sizeof(log_folder));
	sprintf((char*)log_folder, ".\\%s_%S", m_cfg.name, m_id_info.GetBuffer());

	if(m_cfg.test_i == TEST_DATA)
	{
		if (m_cfg.at_test)
		{
			len += sprintf((char*)log_folder, "%s\\%s_%s\\%d#%d#%d", log_folder, (m_cfg.fifo_en?"fifo":"drdy"),(m_cfg.report_mode?"ibi":"poll"), m_cfg.w_id,m_cfg.l_id,m_cfg.d_id);
		}
		else
		{
			len += sprintf((char*)log_folder, "%s\\%s_%s", log_folder, (m_cfg.fifo_en?"fifo":"drdy"),(m_cfg.report_mode?"ibi":"poll"));
		}
	}
	else if (m_cfg.test_i == TEST_SELFTEST)
	{
		len += sprintf((char*)log_folder, "%s\\%s", log_folder, (m_cfg.report_mode ? "selftest_ibi" : "selftest"));
	}
	else if (m_cfg.test_i == TEST_POLL_ODR)
	{
		len += sprintf((char*)log_folder, "%s\\odr_%s_%s", log_folder, (m_cfg.fifo_en ? "fifo" : "drdy"), (m_cfg.report_mode ? "ibi" : "poll"));
	}
	else if (m_cfg.test_i == TEST_ANGLE)
	{
		len += sprintf((char*)log_folder, "%s\\%s", log_folder, "angle");
	}
	else
	{
		len += sprintf((char*)log_folder, "%s\\%s", log_folder, "misc");
	}

	folder.Empty();
	folder.Format(L"%S\\", log_folder);
	if(!PathIsDirectory(folder))
	{
		evb_make_full_path(folder);
	}
}

int CqstevbDlg::evb_calc_sample_odr(unsigned int sample)
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
				else
				{
					if( (m_odr.tm_now - m_odr.tm_start) > 20*1000)
					{
						QST_PRINTF("evb_calc_sample_odr timeout!\r\n");
						return 0;
					}
				}
			}
			else
			{
				QST_PRINTF("evb_calc_sample_odr error! (%d-%d)/(%lld-%lld)\r\n", m_odr.samples2,m_odr.samples1,m_odr.tm_now,m_odr.tm_start);
				memset(&m_odr, 0, sizeof(m_odr)); 
				return 0;
			}
		}
	}
	else
	{
		if (m_odr.tm_start == 0)
		{
			m_odr.tm_start = GetTickCount();
		}
		m_odr.tm_now = GetTickCount();
		if ((m_odr.tm_now - m_odr.tm_start) > 20 * 1000)
		{
			QST_PRINTF("evb_calc_sample_odr timeout(no sample)!\r\n");
			return 0;
		}
	}

	return 1;
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


    return TRUE;
}

void CqstevbDlg::evb_refresh_com(void)
{
	m_dev_connect = FALSE;
	m_ComId.ResetContent();

#if defined(QST_USE_DEVICE)
	m_usbdev_detect = FALSE;
	m_usbdev_type = DEVICE_NONE;
	if (usb_open_device())
	{
		usb_close_device();
		m_usbdev_detect = TRUE;
		m_ComId.AddString(L"usb_dev");
		m_ComId.SetCurSel(0);
		m_comset.portNum = 1;
	}
	else
#endif
	{
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
	}

	if (m_comset.portNum > 0)
		m_BtComOpen.SetWindowTextW(_T("打开"));
	else
		m_BtComOpen.SetWindowTextW(_T("关闭"));
}

void CqstevbDlg::OnBnClickedComOpen()
{
	if(m_comset.portNum <= 0)
	{
		evb_refresh_com();
	}
	else if(m_dev_connect == FALSE)
	{
		CString port_name;

		port_name.Empty();
		m_comset.comid = m_ComId.GetCurSel();
		m_ComId.GetWindowTextW(port_name);
#ifdef UNICODE
		m_comset.portName = CW2A(port_name.GetString());
#else
		m_comset.portName = port_name.GetBuffer();
#endif
		m_SerialPort.setMinByteReadNotify(1);
		m_SerialPort.init(m_comset.portName, m_comset.baud, itas109::Parity(m_comset.parity), itas109::DataBits(m_comset.data), itas109::StopBits(m_comset.stop),itas109::FlowControl(com_flow[0]), MAX_RX_FRAME_BUF);
		m_dev_connect = m_SerialPort.open();

		if(m_dev_connect)
		{
			m_SerialPort.clearError();
			m_SerialPort.readAllData((char*)m_rx_buf);
			memset(m_rx_buf, 0, MAX_RX_READY_BUF);
			m_string_rx.Empty();
			m_edit_rx.Clear();
			m_edit_rx.SetWindowTextW(m_string_rx);
			m_cfg.id = 0;
			m_cfg.cfg_i = 1;

			QST_PRINTF("\n*****\nport:%S %d,%d,%d,%d\n", port_name,m_comset.baud,m_comset.parity,m_comset.data,m_comset.stop);
			Sleep(10);
			evb_tx_func((unsigned char*)"rst", 3);
		}
		else
		{
			AfxMessageBox(_T("open COM fail!"), MB_OK, MB_ICONERROR);
			return;
		}
	}
	else
	{
		evb_close_log_file();
		KillTimer(EVB_TIMER_ID_1);
		KillTimer(EVB_TIMER_ID_2);
		KillTimer(EVB_TIMER_ID_3);
		KillTimer(EVB_TIMER_ID_4);

		evb_tx_func((unsigned char*)"rst", 3);
		m_dev_connect = FALSE;
		m_sample_flag = FALSE;
		log_each_flag = TRUE;
		m_cfg.cfg_i = 1;
		m_SerialPort.close();
	}

	m_BtComOpen.EnableWindow(0);
	SetTimer(EVB_TIMER_ID_2, 300, NULL);
}

#if defined(QST_USE_DEVICE)
void CqstevbDlg::OnBnClickedUsbOpen()
{
	if (m_usbdev_type == DEVICE_NONE)
	{
		m_usbdev_type = usb_open_device();
	}
	else
	{
		usb_close_device();
		m_usbdev_type = DEVICE_NONE;
	}
	if (m_usbdev_type)
	{
		m_dev_connect = TRUE;
		memset(m_rx_buf, 0, MAX_RX_READY_BUF);
		m_string_rx.Empty();
		m_edit_rx.Clear();
		m_edit_rx.SetWindowTextW(m_string_rx);
		m_cfg.id = 0;
		m_cfg.cfg_i = 1;
		i2c_init(m_usbdev_type, 400 * 1000);
		mag_hub_read_info(&m_cfg.chipid, &m_cfg.v_id, &m_cfg.w_id, &m_cfg.d_id, &m_cfg.l_id);
		QST_PRINTF("chipid = 0x%02x\r\n", m_cfg.chipid);
	}
	else
	{
		KillTimer(EVB_TIMER_ID_1);
		KillTimer(EVB_TIMER_ID_2);
		KillTimer(EVB_TIMER_ID_3);
		KillTimer(EVB_TIMER_ID_4);
		KillTimer(EVB_TIMER_ID_5);
		evb_close_log_file();
		m_dev_connect = FALSE;
		m_sample_flag = FALSE;
		log_each_flag = TRUE;
		m_cfg.cfg_i = 1;
	}
	m_BtComOpen.EnableWindow(0);
	SetTimer(EVB_TIMER_ID_2, 300, NULL);
}
#endif

void CqstevbDlg::OnBnClickedOpen()
{
#if defined(QST_USE_DEVICE)
	if (m_usbdev_detect)
	{
		OnBnClickedUsbOpen();
	}
	else
#endif
	{
		OnBnClickedComOpen();
	}
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
		int ret = 0;

		ret = evb_calc_sample_odr(m_odr.samples2);
		if (ret == 0)
		{
			m_odr.samples2 = m_cfg.max_count;
		}
		m_StrStatus.Empty();
		m_StrStatus.Format(L"[%S]", m_cfg_info);
		m_StatusBar.SetPaneText(0, m_StrStatus, TRUE);

		m_StrStatus.Empty();
		m_StrStatus.Format(L"U%d %S samples[%d] rate[%.2f Hz]", m_cfg.id ,m_cfg.name, m_odr.samples2, m_odr.odr);
		m_StatusBar.SetPaneText(1, m_StrStatus, TRUE);

		m_StrStatus.Empty();
		if(strlen(m_custom_info))
			m_StrStatus.Format(L"log %S", m_custom_info);
		m_StatusBar.SetPaneText(2, m_StrStatus, TRUE);

		GetLocalTime(&st);
		m_StrStatus.Empty();
		m_StrStatus.Format(L"%04d/%02d/%02d %02d:%02d:%02d", st.wYear,st.wMonth,st.wDay,st.wHour,st.wMinute,st.wSecond);
		m_StatusBar.SetPaneText(3, m_StrStatus, TRUE);
		//QST_PRINTF("samples[%d] rate[%.2f(hz)]\n", m_odr.samples2, m_odr.odr);
		if(m_cfg.test_i == TEST_DATA)
		{
			if (m_odr.samples2 >= (unsigned int)m_cfg.max_count)
			{
				OnBnClickedButtonSampling();	// stop
				if (m_cfg.at_test)
				{
					m_cfg.cfg_i++;
					SetTimer(EVB_TIMER_ID_3, (m_cfg.report_mode ? EVB_TIMER3_IBI_DELAY :EVB_TIMER3_DELAY), NULL);
				}
				else
				{
					AfxMessageBox(_T("Read Data Finish!"), MB_OK, MB_ICONINFORMATION);
				}
			}
		}
		else if (m_cfg.test_i == TEST_SELFTEST)
		{
			if (m_odr.samples2 >= (unsigned int)m_cfg.max_count)
			{
				OnBnClickedButtonSampling();
				if (m_cfg.at_test)
				{
					m_cfg.id++;
					if (m_cfg.id < m_cfg.id_max)
					{
						SetTimer(EVB_TIMER_ID_3, (m_cfg.report_mode ? EVB_TIMER3_IBI_DELAY : EVB_TIMER3_DELAY), NULL);
					}
					else
					{
						m_cfg.id = 0;
						AfxMessageBox(_T("Poll Selftest Finish!"), MB_OK, MB_ICONINFORMATION);
					}
				}
				else
				{
					AfxMessageBox(_T("Poll Selftest Finish!"), MB_OK, MB_ICONINFORMATION);
				}
			}
		}
		else if (m_cfg.test_i == TEST_POLL_ODR)
		{
			if (m_odr.samples2 >= (unsigned int)m_cfg.max_count)
			{
				memset(m_cfg_info, 0, sizeof(m_cfg_info));
				int len = sprintf((char*)m_cfg_info, "%s_%s_osr%s%s_%s,%.2f\r\n",m_set.range[m_cfg.range], m_set.odr[m_cfg.mode], m_set.osr1[m_cfg.osr1], 
																					m_set.osr2[m_cfg.osr2], m_set.sr[m_cfg.sr], m_odr.odr);
				QST_PRINTF("write %s", m_cfg_info);
				evb_write_log_file_ext(m_cfg_info, len);
				OnBnClickedButtonSampling();

				m_cfg.mode++;
				if (m_cfg.mode < m_set.odr_num)
				{
					log_each_flag = FALSE;
					SetTimer(EVB_TIMER_ID_3, (m_cfg.report_mode ? EVB_TIMER3_IBI_DELAY : EVB_TIMER3_DELAY), NULL);
				}
				else
				{
					log_each_flag = TRUE;
					m_cfg.mode = 0;
					if (m_cfg.at_test)
					{
						m_cfg.id++;
						if (m_cfg.id < m_cfg.id_max)
						{
							SetTimer(EVB_TIMER_ID_3, (m_cfg.report_mode ? EVB_TIMER3_IBI_DELAY : EVB_TIMER3_DELAY), NULL);
						}
						else
						{
							m_cfg.id = 0;
							AfxMessageBox(_T("Poll Odr Finish!"), MB_OK, MB_ICONINFORMATION);
						}
					}
					else
					{
						log_each_flag = TRUE;
						AfxMessageBox(_T("Poll Odr Finish!"), MB_OK, MB_ICONINFORMATION);
					}
				}
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
		KillTimer(EVB_TIMER_ID_3);
		if ((m_cfg.test_i == TEST_DATA)|| (m_cfg.test_i == TEST_POLL_ODR) || (m_cfg.test_i == TEST_SELFTEST))
		{
			OnBnClickedButtonSampling();
		}
	}
	else if(nIDEvent == EVB_TIMER_ID_4)
	{
		KillTimer(EVB_TIMER_ID_4);
		AfxMessageBox(_T("完成!"), MB_OK, MB_ICONINFORMATION);
	}
#if defined(QST_USE_DEVICE)
	else if (nIDEvent == EVB_TIMER_ID_5)
	{
		int len = 0;
		if (m_cfg.test_i == TEST_DATA)
		{
			mag_hub_read_data(m_data.m_out);
			len = sprintf((char*)m_asi_decode.buf, "%.1f,%.1f,%.1f,M-%d\r\n", m_data.m_out[0], m_data.m_out[1], m_data.m_out[2], m_odr.samples2);
			m_odr.samples2++;
		}
		else if (m_cfg.test_i == TEST_SELFTEST)
		{
			int ret = mag_hub_do_selftest(m_data.m_raw);
			len = sprintf((char*)m_asi_decode.buf, "%d,%d,%d,M-%d,SELFTEST-%s\r\n", m_data.m_raw[0], m_data.m_raw[1], m_data.m_raw[2], m_odr.samples2,
																					(ret ? "PASS" : "FAIL"));
			m_odr.samples2++;
		}

		evb_rx_display_buf(m_asi_decode.buf, len);
		if (log_button)
		{
			evb_write_log_file((char*)m_asi_decode.buf, len);
		}
		//QST_PRINTF("%.1f,%.1f,%.1f,M-%d\r\n", m_data.m_out[0], m_data.m_out[1], m_data.m_out[2], m_odr.samples2++);
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
	KillTimer(EVB_TIMER_ID_3);

	if(m_dlg_chart)
	{
		delete m_dlg_chart;
		m_dlg_chart = NULL;
	}

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

BOOL CqstevbDlg::evb_get_config(void)
{
	BOOL ret = FALSE;
	char* str;
	int csv_cfg[7] = { 0,0,0,0,0,0,0 };

	if ((m_cfg.at_test) && (m_cfg.cfg_max > 1))
	{
		if (m_cfg.cfg_i >= m_cfg.cfg_max)
		{
			m_cfg.cfg_i = 1;
			m_cfg.id++;
		}

		while ((m_cfg.id < m_cfg.id_max) && (m_cfg.cfg_i < m_cfg.cfg_max))
		{
			str = (char*)m_config_tbl.getItem(m_cfg.cfg_i, 0);
			csv_cfg[0] = atoi(str);
			if (csv_cfg[0] > 0)
			{
				for (int i = 0; i < (sizeof(csv_cfg) / sizeof(csv_cfg[0])); i++)
				{
					str = (char*)m_config_tbl.getItem(m_cfg.cfg_i, i);
					csv_cfg[i] = atoi(str);
					//QST_PRINTF(" v[%d]=%d", i, csv_cfg[i]);
				}
				m_cfg.mode = csv_cfg[2];
				m_cfg.osr1 = csv_cfg[3];
				m_cfg.osr2 = csv_cfg[4];
				m_cfg.delay = csv_cfg[5];
				m_cfg.max_count = csv_cfg[6];
				QST_PRINTF("odr[%s] osr[%s %s] %s max[%d]\r\n",m_set.odr[m_cfg.mode],m_set.osr1[m_cfg.osr1],m_set.osr2[m_cfg.osr2],m_set.sr[m_cfg.sr],m_cfg.max_count);
				ret = TRUE;
				break;
			}
			else
			{
				m_cfg.cfg_i++;
			}

			if (m_cfg.cfg_i >= m_cfg.cfg_max)
			{
				m_cfg.cfg_i = 1;
				m_cfg.id++;
			}
		}

		return ret;
	}
	else
	{
		for (m_cfg.cfg_i = 1; m_cfg.cfg_i < m_cfg.cfg_max; m_cfg.cfg_i++)
		{
			str = (char*)m_config_tbl.getItem(m_cfg.cfg_i, 0);
			csv_cfg[0] = atoi(str);
			if (csv_cfg[0] >= 1)
			{
				for (int i = 0; i < (sizeof(csv_cfg) / sizeof(csv_cfg[0])); i++)
				{
					str = (char*)m_config_tbl.getItem(m_cfg.cfg_i, i);
					csv_cfg[i] = atoi(str);
				}
				if ((m_cfg.mode == csv_cfg[2]) && (m_cfg.osr1 == csv_cfg[3]) && (m_cfg.osr2 == csv_cfg[4]))
				{
					m_cfg.max_count = csv_cfg[6];
					QST_PRINTF("config item %d max count = %d \r\n", m_cfg.cfg_i, m_cfg.max_count);
					break;
				}
			}
		}
		return TRUE;
	}
}

void CqstevbDlg::OnBnClickedButtonSampling()
{
	unsigned char cmd[64];
	int txlen = 0;

	log_button = m_BtSaveLog.GetCheck();
	if(!m_sample_flag)
	{
		m_edit_id.GetWindowTextW(m_id_info);
		if(m_dev_connect == FALSE)
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
			if(m_cfg.fifo_wmk >= m_set.fifo_wmk_num)
			{
				AfxMessageBox(_T("FIFO WKM error"), MB_OK, MB_ICONERROR);
				return;
			}
			m_cfg.fifo_en = 1;
		}
		else
		{
			m_cfg.fifo_en = 0;
		}

		m_cfg.max_count = 0x7fffffff;
		m_cfg.delay = 0;
		if(m_cfg.test_i == TEST_DATA)
		{
			BOOL result = evb_get_config();
			if (result == FALSE)
			{
				m_cfg.cfg_i = 1;
				m_cfg.id = 0;
				AfxMessageBox(_T("读取数据完成!"), MB_OK, MB_ICONINFORMATION);
				return;
			}
		}
		else if (m_cfg.test_i == TEST_WORK_CURRENT)
		{
		}
		else if (m_cfg.test_i == TEST_SELFTEST)
		{
			m_cfg.delay = 50;
			m_cfg.max_count = 300;
		}
		else if (m_cfg.test_i == TEST_WRITE_READ_REGISTER)
		{
			m_cfg.delay = 0;
			m_cfg.sr = 0;
			m_cfg.max_count = 60000;
		}
		else if (m_cfg.test_i == TEST_POLL_ODR)
		{
			if (m_cfg.mode < m_set.odr_num)
			{
				int num = sscanf((char*)m_set.odr[m_cfg.mode], "%dHz", &m_cfg.odr);
				if (m_cfg.mode == 0)
				{
					if((m_cfg.chipid == 0x90)|| (m_cfg.chipid == 0x91)|| (m_cfg.chipid == 0x92))
						m_cfg.odr = 300;
					else if (m_cfg.chipid == 0x20)
						m_cfg.odr = 500;
					else
						m_cfg.odr = 300;
				}
				//QST_PRINTF("test odr : %d Hz\r\n", m_cfg.odr);
			}
			m_cfg.max_count = 30 * m_cfg.odr;
		}
		else if (m_cfg.test_i == TEST_ANGLE)
		{
			int num = sscanf((char*)m_set.odr[m_cfg.mode], "%dHz", &m_cfg.odr);
			if (m_cfg.mode == 0)
			{
				m_cfg.odr = 300;
			}
			if (m_cfg.odr > 100)
				m_cfg.delay = 10;
			else if (m_cfg.odr = 100)
				m_cfg.delay = 20;
			else if(m_cfg.odr > 0)
				m_cfg.delay = (1000/ m_cfg.odr)*4/5;
		}

		m_MagId.SetCurSel(m_cfg.id);
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
		user_path.Empty();
		memset(&m_odr, 0, sizeof(m_odr));
		m_sample_flag = TRUE;
		log_temp = FALSE;

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
		if((m_cfg.delay < 0) && (m_cfg.delay > 60000))
		{
			m_cfg.delay = 0;
		}
#if defined(QST_USE_DEVICE)
		if (m_usbdev_detect)
		{
			if (m_cfg.test_i == TEST_DATA)
			{
				m_cfg.delay = mag_hub_set(m_cfg.range, m_cfg.mode, m_cfg.osr1, m_cfg.osr2, m_cfg.sr, m_cfg.fifo_mode, m_cfg.fifo_wmk);
				mag_hub_enable();
			}
			else if (m_cfg.test_i == TEST_SELFTEST)
			{
				m_cfg.delay = 300;
			}
			evb_open_log_file();
			SetTimer(EVB_TIMER_ID_5, m_cfg.delay, NULL);
		}
		else
#endif
		{
			if (m_cfg.test_i == TEST_ANGLE)
			{
				memset(cmd, 0, sizeof(cmd));
				txlen = sprintf((char*)cmd, "mat0,%f,%f,%f", m_cfg.mat[0][0], m_cfg.mat[0][1], m_cfg.mat[0][2]);
				evb_tx_func(cmd, txlen);
				Sleep(UART_WRITE_DELAY);
				memset(cmd, 0, sizeof(cmd));
				txlen = sprintf((char*)cmd, "mat1,%f,%f,%f", m_cfg.mat[1][0], m_cfg.mat[1][1], m_cfg.mat[1][2]);
				evb_tx_func(cmd, txlen);
				Sleep(UART_WRITE_DELAY);
				memset(cmd, 0, sizeof(cmd));
				txlen = sprintf((char*)cmd, "mat2,%f,%f,%f", m_cfg.mat[2][0], m_cfg.mat[2][1], m_cfg.mat[2][2]);
				evb_tx_func(cmd, txlen);
				Sleep(UART_WRITE_DELAY);

				memset(cmd, 0, sizeof(cmd));
				txlen = sprintf((char*)cmd, "bias0,%f,%f,%f", m_cfg.bias_a[0], m_cfg.bias_a[1], m_cfg.bias_a[2]);
				evb_tx_func(cmd, txlen);
				Sleep(UART_WRITE_DELAY);
				memset(cmd, 0, sizeof(cmd));
				txlen = sprintf((char*)cmd, "bias1,%f,%f,%f", m_cfg.bias_g[0], m_cfg.bias_g[1], m_cfg.bias_g[2]);
				evb_tx_func(cmd, txlen);
				Sleep(UART_WRITE_DELAY);
			}
			memset(cmd, 0, sizeof(cmd));
			txlen = sprintf((char*)cmd, "ma,%d,%d,%d,%d,%d", m_cfg.test_i, m_cfg.fifo_en, (m_cfg.fifo_mode << 6), m_cfg.fifo_wmk, m_cfg.max_count);
			evb_tx_func(cmd, txlen);
			Sleep(UART_WRITE_DELAY);
			memset(cmd, 0, sizeof(cmd));
			txlen = sprintf((char*)cmd, "mb,%d,%d,%d,%d,%d", m_cfg.mode, m_cfg.range, m_cfg.osr1, m_cfg.osr2, m_cfg.sr);
			evb_tx_func(cmd, txlen);
			Sleep(UART_WRITE_DELAY);
			memset(cmd, 0, sizeof(cmd));
			txlen = sprintf((char*)cmd, "start,%d,%d,%d,%d", m_cfg.delay, m_cfg.id, m_cfg.report_mode, m_cfg.at_test);
			evb_tx_func(cmd, txlen);
			Sleep(UART_WRITE_DELAY);
		}

		if (m_cfg.test_i != TEST_WRITE_READ_REGISTER)
		{
			SetTimer(EVB_TIMER_ID_1, 300, NULL);
		}
	}
	else
	{
#if defined(QST_USE_DEVICE)
		if (m_usbdev_detect)
		{
			mag_hub_disable();
		}
#endif
		KillTimer(EVB_TIMER_ID_1);
		KillTimer(EVB_TIMER_ID_2);
		KillTimer(EVB_TIMER_ID_3);
		KillTimer(EVB_TIMER_ID_4);
		KillTimer(EVB_TIMER_ID_5);
		//m_dlg_chart->ShowWindow(SW_HIDE);
		log_each_flag = TRUE;
		evb_close_log_file();
		m_SerialPort.readAllData((char*)m_rx_buf);
		m_SerialPort.clearError();
		m_sample_flag = FALSE;
		evb_tx_func((unsigned char*)"rst", 3);
		Sleep(UART_WRITE_DELAY);
		evb_tx_func((unsigned char*)"rst", 3);
		Sleep(UART_WRITE_DELAY);
		memset(m_rx_buf, 0, MAX_RX_READY_BUF);
		memset(&m_odr, 0, sizeof(m_odr));
	}

	m_BtSampling.EnableWindow(0);
	SetTimer(EVB_TIMER_ID_2, 300, NULL);
}

void CqstevbDlg::OnBnClickedButtonImuCali()
{
	// TODO:
	unsigned char cmd[64];
	int txlen = 0;

	m_cfg.imu_cali_flag = TRUE;
	m_sample_flag = TRUE;
	m_cfg.delay = 20;
	m_cfg.bias_a[0] = m_cfg.bias_a[1] = m_cfg.bias_a[2] = 0.0f;
	m_cfg.bias_g[0] = m_cfg.bias_g[1] = m_cfg.bias_g[2] = 0.0f;

	AfxMessageBox(_T("请保持设备水平朝上，并静止!"), MB_OK, MB_ICONWARNING);
	memset(cmd, 0, sizeof(cmd));
	txlen = sprintf((char*)cmd, "bias0,%f,%f,%f", m_cfg.bias_a[0], m_cfg.bias_a[1], m_cfg.bias_a[2]);
	evb_tx_func(cmd, txlen);
	Sleep(UART_WRITE_DELAY);
	memset(cmd, 0, sizeof(cmd));
	txlen = sprintf((char*)cmd, "bias1,%f,%f,%f", m_cfg.bias_g[0], m_cfg.bias_g[1], m_cfg.bias_g[2]);
	evb_tx_func(cmd, txlen);
	Sleep(UART_WRITE_DELAY);
	memset(cmd, 0, sizeof(cmd));
	txlen = sprintf((char*)cmd, "ma,%d,%d,%d,%d,%d", m_cfg.test_i, m_cfg.fifo_en, (m_cfg.fifo_mode << 6), m_cfg.fifo_wmk, m_cfg.max_count);
	evb_tx_func(cmd, txlen);
	Sleep(UART_WRITE_DELAY);
	memset(cmd, 0, sizeof(cmd));
	txlen = sprintf((char*)cmd, "mb,%d,%d,%d,%d,%d", m_cfg.mode, m_cfg.range, m_cfg.osr1, m_cfg.osr2, m_cfg.sr);
	evb_tx_func(cmd, txlen);
	Sleep(UART_WRITE_DELAY);
	memset(cmd, 0, sizeof(cmd));
	txlen = sprintf((char*)cmd, "start,%d,%d,%d,%d", m_cfg.delay, m_cfg.id, m_cfg.report_mode, m_cfg.at_test);
	evb_tx_func(cmd, txlen);
	Sleep(UART_WRITE_DELAY);
}

BOOL CqstevbDlg::PreTranslateMessage(MSG* pMsg)
{
	//QST_PRINTF("message:%d\r\n",pMsg->message);
	if (pMsg->message == WM_KEYDOWN)
	{
		switch (pMsg->wParam)
		{
		case 'd':
		case 'D':
			if((m_cfg.test_i == TEST_DATA)|| (m_cfg.test_i == TEST_ANGLE))
			{
				QST_PRINTF("user key d down press!\r\n");
				user_button = 1;
			}
			break;
			//return TRUE;
		default:
			break;
		}
	}
	else if (pMsg->message == WM_SYSCHAR)
	{
		switch (pMsg->wParam)
		{
		case 'd':
		case 'D':
			if ((m_cfg.test_i == TEST_DATA) || (m_cfg.test_i == TEST_ANGLE))
			{
				QST_PRINTF("user key char d press!\r\n");
				user_button = 1;
			}
		default:
			break;
		}
	}

	return CDialog::PreTranslateMessage(pMsg);

	//CWnd *pCtrl = CWnd::GetFocus();
//int focusId = pCtrl->GetDlgCtrlID();
	//case VK_RETURN:
//	if(focusId == IDC_EDIT_TEST_ID)
//	{
//		QST_PRINTF("edit id focus!\r\n");
//		OnBnClickedButtonSampling();
//	}
//	return TRUE;
//	break;
	//return TRUE;
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

		if (m_cfg.test_i == TEST_ANGLE)
			m_BtUser1.ShowWindow(SW_SHOW);
		else
			m_BtUser1.ShowWindow(SW_HIDE);

		if (m_sample_flag)
			m_BtUser1.EnableWindow(0);
		else
			m_BtUser1.EnableWindow(1);
	}
	QST_PRINTF("select %d test item: %d\r\n", index, m_cfg.test_i);
}

void CqstevbDlg::OnBnClickedButtonUser()
{
	// TODO:
	if(m_sample_flag)
	{
		user_button = TRUE;		
		QST_PRINTF("user key press(cat data)!\r\n");
	}
}

void CqstevbDlg::OnCbnSelchangeComboFifoReportMode()
{
	// TODO:
	int index;

	index  = m_MagReport.GetCurSel();
	if(index != CB_ERR)
	{
		m_cfg.report_mode = index;
	}
}

void CqstevbDlg::OnCbnSelchangeComboId()
{
	// TODO: 在此添加控件通知处理程序代码
	int index;

	index = m_MagId.GetCurSel();
	if (index != CB_ERR)
	{
		m_cfg.id = index;
		QST_PRINTF("OnCbnSelchangeComboId %d \r\n", m_cfg.id);
	}
}

void CqstevbDlg::OnCbnSelchangeComboSensorType()
{
	// TODO: 在此添加控件通知处理程序代码
	int index;
	unsigned char chipid = 0x00;

	index = m_MagSensorId.GetCurSel();
	if (index != CB_ERR)
	{
		if (index == 0)
			chipid = 0x90;
		else if (index == 1)
			chipid = 0x91;
		else if (index == 2)
			chipid = 0x20;
		else if (index == 3)
			chipid = 0x80;
	}
	if (m_cfg.chipid != chipid)
	{
		m_cfg.chipid = chipid;
		evb_init_config_ui();
	}
	QST_PRINTF("select chipid=0x%x\r\n", m_cfg.chipid);
}

void CqstevbDlg::OnBnClickedCheckOutPic()
{
	// TODO: 在此添加控件通知处理程序代码
	//QST_PRINTF("OnBnClickedCheckOutPic %d\r\n", m_BtOutPic.GetCheck());
	if (m_BtOutPic.GetCheck())
	{
		m_BtSaveLog.SetCheck(TRUE);
	}
}
