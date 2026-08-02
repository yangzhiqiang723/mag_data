

#pragma once
 
#include "afxwin.h"
#include "resource.h"
#include "ChartCtrl/ChartCtrl.h" 
#include ".\ChartCtrl\ChartAxisLabel.h"
#include ".\ChartCtrl\ChartLineSerie.h"
#include ".\ChartCtrl\ChartBarSerie.h"

#define DLG_TIMER_ID_1		1
#define EVB_CHART_MAX_P		80
//#define EVB_CHART_BOTTOM_AXIS

class CSubDialog : public CDialog
{
    // Construction
public:
	double				b_data[EVB_CHART_MAX_P];
	double				l_data_1[EVB_CHART_MAX_P];
	double				l_data_2[EVB_CHART_MAX_P];
	double				l_data_3[EVB_CHART_MAX_P];
	//unsigned int		m_bIndex;
	unsigned int		m_DataIndex;
	BOOL				m_Chart_init;

	CChartLineSerie		*m_pLineSerie1;
	CChartLineSerie		*m_pLineSerie2;
	CChartLineSerie		*m_pLineSerie3;
	CChartCtrl			m_ChartCtrl;

public:
    CSubDialog(CWnd* pParent = nullptr);   // standard constructor

	void dlg_chart_init(void);
	void dlg_chart_reset(void);
	void dlg_chart_set_data(float in[3]);
	void dlg_chart_update(void);

	virtual BOOL OnInitDialog();

	afx_msg void OnTimer(UINT_PTR nIDEvent);
//  afx_msg void PostNcDestroy();
//	afx_msg void OnClose();
 
    // Dialog Data
    enum { IDD = IDD_SUBDIALOG };
 
protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
 
    // Implementation
protected:
    DECLARE_MESSAGE_MAP()
};