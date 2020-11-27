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

#include <matrix/matrix.h>
#include <matrix/matrix_qr.h>

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
		void DrawTestingPoint(cv::Point2f projectedPoint);
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

		void KeyPressed(int key);

		void Calibrate();

		cv::Point2f GetProjectedPoint(cv::Point3f worldPoint);

		std::vector<double> GetCalibration();

		void LoadCalibration();
		void SaveCalibration(std::string path);

		bool isCalibrated() { return calibrated; }

	private:
		bool mouseDown;
		cv::Point mousePoint;

		cv::String uiWindowName;
		cv::Mat uiMat;

		cv::String projectorWindowName;
		cv::Mat projectorMat;

		std::vector<cv::Point2f>           currentProjectorPoints;
		std::vector<cv::Point2f>			cvPoints;
		std::vector<cv::Point3f>	        pairsKinect;
		std::vector<cv::Point2f>           pairsProjector;

		cv::Point					testPoint;

		int                         chessboardSize;
		int                         chessboardX;
		int                         chessboardY;
		bool                        testing;
		bool						foundChessboard;
		bool						calibrated;

		dlib::matrix<double, 0, 11> A;
		dlib::matrix<double, 0, 1> y;
		dlib::matrix<double, 11, 1> x;
	};
}