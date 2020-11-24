#pragma once

#include "Service.h"
#include <vector>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include "InteractionHandler.h"
#include <boost//signals2/signal.hpp>
#include <boost/container/vector.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_painter2D.h>
#include <pcl/visualization/mouse_event.h>
#include <vtkObject.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkCallbackCommand.h>
#include <vtkGenericRenderWindowInteractor.h>


using namespace pcl;

namespace MOMO
{
	class CalibrationService : public Service
	{
	public:
		CalibrationService();
		~CalibrationService();

		void InitStartService();
		void UpdateService();
		void StopService();


		void DrawChessboard(int x, int y, int chessboardSize);
		void DrawTestingPoint(Eigen::Vector2f projectedPoint);
		void AddPointPair();

		bool isTesting() const { return testing; }
		void StartTesting() { testing = true; }
		void StopTesting() { testing = false; }

	private:
		boost::shared_ptr <visualization::PCLPainter2D> uiPainter;
		vtkRenderWindow* uiWindow;
		vtkRenderer* uiRenderer;

		boost::shared_ptr <visualization::PCLPainter2D> projectorPainter;
		vtkRenderWindow* projectorWindow;
		vtkRenderer* projectorRenderer;

		InteractionHandler* uiWindowInteractionHandler;

		//ofxCvColorImage             rgbImage;

		boost::container::vector<Eigen::Vector2f>             currentProjectorPoints;
		std::vector<cv::Point2f>			cvPoints;
		//vector<ofVec3f>             pairsKinect;
		//vector<ofVec2f>             pairsProjector;

		//string                      resultMessage;
		//ofColor                     resultMessageColor;
		Eigen::Vector2f             testPoint;

		int                         chessboardSize;
		int                         chessboardX;
		int                         chessboardY;
		bool                        testing;
		bool                        saved;

		bool						foundChessboard;
	

		vtkCallbackCommand *PassiveEventCallbackCommand;
	};
}