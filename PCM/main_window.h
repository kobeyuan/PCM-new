#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QtWidgets/QMainWindow>
#include <QMessageBox>
#include "ui_main_window.h"
//#include "ui_dlg_autoselectkeyframes.h"


#include "paint_canvas.h"
#include "globals.h"
#include <string>
#include <map>
#include <QLabel>
#include "select_tool.h"
#include "paint_interface.h"

#include "graph_cut_node.h"

#include "plans_fit.h"

#include "GCop.h"

#include"dlg_Jlinkage.h"
#include "dlg_graphcuts.h"
#include "dlg_fitPlan.h"
#include "dlg_paintInterface.h"
//#include "dlg_StopMotion.h"

//new UI
#include "dlg_StopMotionRefine.h"

#include "dlg_StopMotionASig.h"

#include "PCM/dlg_autoselection.h"

//for scanning keyframe
#include "dlg_ImageScanner.h";

#include"multiObjectSeg.h"
#include "scanner.h"

#include "saveSnapshotDialog.h"
#include "SingleFrameSet.h"

#include "StopMotion.h"

using namespace std;
class CanvasUI;

class main_window : public QMainWindow
{
	Q_OBJECT

public:
	main_window(QWidget *parent = 0);
	~main_window();
	static std::string title() { return "[MainWindow]: "; }

	void showCoordinateAndIndexUnderMouse( const QPoint& point );

	public slots:
		bool openFile();
		bool openFiles();
		bool openVideoFiles();
		bool openVideo4Selection();
		bool openImageSequences();
		bool openCamera();
		void selectedSampleChanged(QTreeWidgetItem * item, int column);

		bool setSampleVisible();
		bool setSampleInvisible();

		void doClustering();
		void finishClustering();

		void doRegister();
		void finishRegister();

// 		void doSpectralCluster();
// 		void finishSpectralCluster();

		void doGraphCut();
		void finishGraphCut();

		void doGCOptimization();
		void finishGCotimization();

		void setObjectColorMode();
		void setVertexColorMode();
		void setLabelColorMode();

		void setSelectToolMode();
		void setSceneToolMode();

		void setScenePaintMode();

		void showTracer();
		void clearTracer();

		void createTreeWidgetItems();

		void computeSampleNormal();

		void batchTrajClustering();
		void iterateTrajClustering();
		void visDistor();

		void doPlanFit();
		//void finishDoPlanFit();
		void doOrder();
		void doRefineSigFrame();
		bool saveSnapshot();

		//multi-objects-segment
		void doMultiObjectSeg();
		void doScanning();

		void doStopMotion();

public:

	PaintCanvas* getCanvas()
	{
		return main_canvas_;

	}
private:
		void createAction();
		void createFileMenuAction();
		void createPaintSettingAction();
		void createAlgorithmAction();
		void createToolAction();

		void createStatusBar();
		void resetSampleSet();



private:

	//UI
	Ui::main_windowClass ui;
	PaintCanvas*		main_canvas_;

	QLabel*			coord_underMouse_label_;
	QLabel*			vtx_idx_underMouse_label_;

	//Samples Info
	vector< pair<string,string> >		cur_import_files_attr_;
	int			cur_select_sample_idx_;
	int			last_select_sample_idx_;

	Tool*			single_operate_tool_;

private:
	IndexType iterate_sample_idx_;


	//UI
private:
	JLinkageUI * m_linkageUi;
	GraphCutUI * m_graphCutUi;
	PlanFitUI* m_planFitUi;
	PaintUi* m_paint;

	

	//StopMotionUI* m_stopMotion;
	//new UI 11-29
	StopMotionRUI* m_stopMotionR;
	
	//new UI 2-17-2017
	StopMotionASIG* m_stopMotionC;

	KeyFrameCapture* m_keyFrameCap;

	CanvasUI* m_canvas;

	//Ui::autoSelectKey* m_autoSelection;
	DLGAutoSelection*  m_autoSelection;

private:
    bool m_isReadVideo;
};


#endif // MAIN_WINDOW_H
