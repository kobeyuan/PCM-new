#pragma  once

#include <QtWidgets/QDialog>
#include "ui_testUi.h"
class dTexture;
class CanvasUI : public QDialog
{
	Q_OBJECT

public:
	CanvasUI();
	~CanvasUI();
	dTexture* getTextureCanvas()
	{
		return m_rendWid;
	}
private:
	
	Ui_textFDia ui;
	dTexture* m_rendWid;

};
