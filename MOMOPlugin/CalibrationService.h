#pragma once

#include "Service.h"
#include <vector>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <boost//signals2/signal.hpp>
#include <boost/container/vector.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>

#include "InteractionHandler.h"


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


		void DrawChessboard(int x, int y);
		void DrawTestingPoint(Eigen::Vector2f projectedPoint);
		void AddPointPair();

		bool isTesting() const { return testing; }
		void StartTesting() { testing = true; }
		void StopTesting() { testing = false; }

		static void MouseCallback(int event, int x, int y, int flags, void* userdata);
		void MouseCallback_P(int event, int x, int y, int flags, void* userdata);

		void MouseDown(int x, int y);

		void MouseUp(int x, int y);

		void MouseMove(int x, int y);

		bool shouldDrawIndicator();

	private:
		//ofxCvColorImage             rgbImage;

		bool mouseDown;
		cv::Point mousePoint;

		cv::String uiWindowName;
		cv::Mat uiMat;

		cv::String projectorWindowName;
		cv::Mat projectorMat;

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
	};
}