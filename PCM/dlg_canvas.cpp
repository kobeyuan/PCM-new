#include "dlg_canvas.h"
#include "DrawTexture.h"



CanvasUI::CanvasUI()
{
	ui.setupUi(this);
	m_rendWid = new dTexture(ui.verticalLayoutWidget);
	//m_rendWid->setFixedHeight(300);
	//m_rendWid->setFixedWidth(300);
	ui.verticalLayout->addWidget(m_rendWid);
	//ui.verticalLayout->insertWidget(0,m_rendWid);
	m_rendWid->show();
	m_rendWid->updateGL();
}

CanvasUI::~CanvasUI()
{

}


