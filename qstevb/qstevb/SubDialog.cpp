
// qstevb.cpp : 定义应用程序的类行为。
//

#include "stdafx.h"
#include "SubDialog.h"

BEGIN_MESSAGE_MAP(CSubDialog, CDialog)
ON_WM_TIMER()
END_MESSAGE_MAP()

BOOL CSubDialog::OnInitDialog()
{
	CDialog::OnInitDialog();

	m_Chart_init = FALSE;
	m_DataIndex = 0;
	memset(b_data, 0, sizeof(b_data));
	memset(l_data_1, 0, sizeof(l_data_1));
	memset(l_data_2, 0, sizeof(l_data_2));
	memset(l_data_3, 0, sizeof(l_data_3));
	m_pLineSerie1 = m_pLineSerie2 = m_pLineSerie3 = NULL;
	dlg_chart_init();

	return TRUE;
}

CSubDialog::CSubDialog(CWnd* pParent /*=NULL*/)
	: CDialog(IDD_SUBDIALOG, pParent)
{
	//m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}


void CSubDialog::DoDataExchange(CDataExchange* pDX)
{
    CDialog::DoDataExchange(pDX);

	DDX_Control(pDX, IDC_SENSOR_CHART, m_ChartCtrl);
}

void CSubDialog::dlg_chart_init(void)
{
	if(m_Chart_init == FALSE)
	{
		#define LINE_WIDTH 3
		CChartAxis *pAxis = NULL;

		m_ChartCtrl.EnableRefresh(FALSE);
		m_ChartCtrl.SetEdgeType(EDGE_BUMP); // EDGE_ETCHED
		m_ChartCtrl.GetLegend()->SetVisible(TRUE);
		m_ChartCtrl.GetLegend()->SetHorizontalMode(TRUE);
		m_ChartCtrl.GetLegend()->UndockLegend(80, 8);
		m_ChartCtrl.SetBackGradient(RGB(255, 255, 255), RGB(150, 150, 255), gtVertical);
		//m_ChartCtrl.SetBackColor(RGB(50, 50, 50));
		m_ChartCtrl.SetZoomEnabled(TRUE);

		// set title
		m_ChartCtrl.GetTitle()->RemoveAll();
		m_ChartCtrl.GetTitle()->AddString(_T("Magnetic"));
		// m_ChartCtrl.GetTitle()->SetFont(12, _T("Arial"));
		m_ChartCtrl.GetTitle()->SetColor(RGB(255, 100, 110));
		m_ChartCtrl.RemoveAllSeries();

		//  create buttom axis
		pAxis = m_ChartCtrl.CreateStandardAxis(CChartCtrl::BottomAxis);
		pAxis->SetAutomatic(true);
		pAxis->GetLabel()->SetVisible(false);
		pAxis->GetLabel()->SetText(_T("Frames"));
		pAxis->GetLabel()->SetColor(RGB(255, 0, 0));
		pAxis->SetGap(0.0);		
		//pAxis->SetDiscrete(TRUE);
#if defined(EVB_CHART_BOTTOM_AXIS)
		pAxis->SetVisible(TRUE);
#else
		pAxis->SetVisible(FALSE);
#endif
		// create left axis
		pAxis = m_ChartCtrl.CreateStandardAxis(CChartCtrl::LeftAxis);
		pAxis->SetAutomatic(true);
		pAxis->SetAutomaticMode(CChartAxis::FullAutomatic);
		pAxis->SetGap(6.0);
		pAxis->GetLabel()->SetText(_T("uT"));
		// pAxis->GetLabel()->SetHorizontal(true);
		pAxis->GetLabel()->SetColor(RGB(255, 0, 0));

		m_pLineSerie1 = m_ChartCtrl.CreateLineSerie();
		m_pLineSerie1->SetSeriesOrdering(poNoOrdering);
		m_pLineSerie1->SetColor(RGB(255, 0, 0));
		m_pLineSerie1->SetName(_T("X"));
		m_pLineSerie1->SetPenStyle(PS_SOLID); // PS_SOLID
		m_pLineSerie1->SetWidth(LINE_WIDTH);
		//m_pLineSerie1->OnMouseEvent(CChartMouseListener :: MouseEvent mouseEvent, const CPoint & screenPoint)

		m_pLineSerie2 = m_ChartCtrl.CreateLineSerie();
		m_pLineSerie2->SetSeriesOrdering(poNoOrdering);
		m_pLineSerie2->SetColor(RGB(0, 255, 0));
		m_pLineSerie2->SetName(_T("Y"));
		m_pLineSerie2->SetPenStyle(PS_SOLID); // PS_SOLID
		m_pLineSerie2->SetWidth(LINE_WIDTH);

		m_pLineSerie3 = m_ChartCtrl.CreateLineSerie();
		m_pLineSerie3->SetSeriesOrdering(poNoOrdering);
		m_pLineSerie3->SetColor(RGB(0, 0, 255));
		m_pLineSerie3->SetName(_T("Z"));
		m_pLineSerie3->SetPenStyle(PS_SOLID); // PS_SOLID
		m_pLineSerie3->SetWidth(LINE_WIDTH);

		m_ChartCtrl.EnableRefresh(TRUE);
		m_ChartCtrl.ShowWindow(SW_SHOW);  // SW_SHOW SW_HIDE
		m_Chart_init = TRUE;
	}
}

void CSubDialog::dlg_chart_reset(void)
{
	m_DataIndex = 0;
	m_ChartCtrl.RefreshCtrl();
	memset(b_data, 0, sizeof(b_data));
	memset(l_data_1, 0, sizeof(l_data_1));
	memset(l_data_2, 0, sizeof(l_data_2));
	memset(l_data_3, 0, sizeof(l_data_3));
}

void CSubDialog::dlg_chart_set_data(float in[3])
{
	if(m_Chart_init == FALSE)
	{
		return;
	}

	if (m_DataIndex < EVB_CHART_MAX_P)
	{
		b_data[m_DataIndex] = m_DataIndex;
		l_data_1[m_DataIndex] = (double)in[0];
		l_data_2[m_DataIndex] = (double)in[1];
		l_data_3[m_DataIndex] = (double)in[2];
		m_DataIndex++;
	}
	else
	{
		for (int i = 0; i < EVB_CHART_MAX_P - 1; i++)
		{
			b_data[i] = b_data[i + 1];
			l_data_1[i] = l_data_1[i + 1];
			l_data_2[i] = l_data_2[i + 1];
			l_data_3[i] = l_data_3[i + 1];
		}

		b_data[EVB_CHART_MAX_P - 1] = m_DataIndex;
		l_data_1[EVB_CHART_MAX_P - 1] = (double)in[0];
		l_data_2[EVB_CHART_MAX_P - 1] = (double)in[1];
		l_data_3[EVB_CHART_MAX_P - 1] = (double)in[2];
		m_DataIndex++;
	}
}

void CSubDialog::dlg_chart_update(void)
{
//	m_ChartCtrl.EnableRefresh(TRUE);
	SetTimer(DLG_TIMER_ID_1, 10, NULL);
}


void CSubDialog::OnTimer(UINT_PTR nIDEvent)
{
	if(nIDEvent == DLG_TIMER_ID_1)
	{
		unsigned int array_size = 1;

		KillTimer(DLG_TIMER_ID_1);
		if(m_Chart_init == FALSE)
		{
			return;
		}

		if(m_DataIndex < EVB_CHART_MAX_P)
			array_size = m_DataIndex;
		else
			array_size = EVB_CHART_MAX_P;

		m_pLineSerie1->SetPoints(b_data, l_data_1, array_size);
		m_pLineSerie2->SetPoints(b_data, l_data_2, array_size);
		m_pLineSerie3->SetPoints(b_data, l_data_3, array_size);
		//m_ChartCtrl.EnableRefresh(FALSE);
	}
}


//void CSubDialog::PostNcDestroy()
//{
//    CDialog::PostNcDestroy();
//    delete this;
//}

//void CSubDialog::OnClose()
//{
//	CDialog::OnClose();
	//delete this;
//}

