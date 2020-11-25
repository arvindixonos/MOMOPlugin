#include <stdio.h>
#include <boost/algorithm/clamp.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/core/types_c.h>
#include <opencv2/highgui.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include "CalibrationService.h"
#include "Constants.h"
#include "Utils.h"
#include "MOMOManager.h"

using namespace MOMO;
using namespace cv;

MOMO::CalibrationService::CalibrationService()
	:projectorWindowName("ProjectorWindow"), uiWindowName("UIWindow")
{
	chessboardSize = 300;
	chessboardX = 5;
	chessboardY = 4;
}

MOMO::CalibrationService::~CalibrationService()
{

}

void MOMO::CalibrationService::InitStartService()
{
	testing = false;

	started = true;

	uiMat = cv::Mat(GAME_WINDOW_SIZE_Y, GAME_WINDOW_SIZE_X, CV_8UC3).clone();
	projectorMat = cv::Mat(CALIBRATION_PROJECTION_SIZE_Y, CALIBRATION_PROJECTION_SIZE_X, CV_8UC3).clone();

	namedWindow(uiWindowName);
	namedWindow(projectorWindowName, WINDOW_NORMAL);
	moveWindow(projectorWindowName, 1920, 0);
	
	resizeWindow(uiWindowName, GAME_WINDOW_SIZE_X, GAME_WINDOW_SIZE_Y);
	setWindowProperty(projectorWindowName, cv::WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

	setMouseCallback(uiWindowName, MOMO::CalibrationService::MouseCallback, this);
}

void MOMO::CalibrationService::UpdateService()
{
	//if (kinect.isNewFrame()) 
	{
		/*depthPixels = kinect.getDepthRawPixels();
		rgbImage.setFromPixels(kinect.getImagePixels());
		*/
		cv::Mat cvRGBImage = MOMOManager::getInstance()->GetRGBMat();
		//uiMat.at<uchar*>(0, 0, uiMat.channels()) = MOMOManager::getInstance()->GetRGBData();

		/*cv::Mat c = cvRGBImage(cv::Range(0, 100), cv::Range(0, 100));
		cv::Mat cvRGBImagea = cvRGBImage.clone();*/

		if (testing) 
		{
			/*Eigen::Vector2f t = Eigen::Vector2f(std::min(640 - 1, testPoint.x), std::min(480 - 1, testPoint.y));
			PointXYZ depthPoint = PointXYZ(t.x, t.y, depthPixels[t.x + t.y * 640]);*/
			/*ofVec3f worldPoint = kinect.projectiveToWorld(depthPoint);
			ofVec2f projectedPoint = kpt.getProjectedPoint(worldPoint);
			drawTestingPoint(projectedPoint);*/
		}
		else 
		{
			//cvRgbImage = ofxCv::toCv(rgbImage.getPixels());
			cv::Size patternSize = cv::Size(chessboardX - 1, chessboardY - 1);
			int chessFlags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK;
			foundChessboard = findChessboardCorners(cvRgbImage, patternSize, cvPoints, chessFlags);
			if (foundChessboard) 
			{
				cv::Mat gray;
				cvtColor(cvRgbImage, gray, cv::COLOR_RGB2GRAY);
				cornerSubPix(gray, cvPoints, cv::Size(11, 11), cv::Size(-1, -1),
					cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
				drawChessboardCorners(cvRgbImage, patternSize, cv::Mat(cvPoints), foundChessboard);
			}
		}

		cvRGBImage.copyTo(uiMat(cv::Rect(0, 0, cvRGBImage.cols, cvRGBImage.rows)));
		cv::rectangle(uiMat, cv::Rect(640, 60, 640, 360), cv::Scalar(150, 80, 0), FILLED);

		if (mouseDown && shouldDrawIndicator())
		{
			cv::circle(uiMat, mousePoint, 5, cv::Scalar(0, 255, 0), FILLED);
		}

		cv::imshow(uiWindowName, uiMat);
		cv::imshow(projectorWindowName, projectorMat);
		cv::waitKey(1);
	}
}

void MOMO::CalibrationService::StopService()
{
	started = false;
}

void MOMO::CalibrationService::DrawChessboard(int x, int y)
{
	mousePoint = cv::Point(x, y);

	x -= (GAME_WINDOW_SIZE_X / 2);
	y -= 60;

	if (x < 0 || y < 0 || y > 360)
		return;
	
	x = (int)(((double)x / 640.0) * CALIBRATION_PROJECTION_SIZE_X);
	y = (int)(((double)y / 360.0) * CALIBRATION_PROJECTION_SIZE_Y);

	float w = chessboardSize / chessboardX;
	float h = chessboardSize / chessboardY;
	
	currentProjectorPoints.clear();

	projectorMat = cv::Mat::zeros(projectorMat.rows, projectorMat.cols, CV_8UC3);
	for (int j = 0; j < chessboardY; j++) {
		for (int i = 0; i < chessboardX; i++) {
			int x0 = mapValue(i, 0, chessboardX, 0, chessboardSize);
			int y0 = mapValue(j, 0, chessboardY, 0, chessboardSize);
			if (j > 0 && i > 0) {
				currentProjectorPoints.push_back(Eigen::Vector2f(
					mapValue(x + x0, 0, CALIBRATION_PROJECTION_SIZE_X, 0, 1),
					mapValue(y + y0, 0, CALIBRATION_PROJECTION_SIZE_Y, 0, 1)
				));
			}

			if ((i + j) % 2 == 0)
			{
				cv::rectangle(projectorMat, cv::Rect(x + x0, y + y0, w, h), cv::Scalar(255, 0, 0), FILLED, LINE_8);
			}
		}
	}
}

void MOMO::CalibrationService::DrawTestingPoint(Eigen::Vector2f projectedPoint)
{

}

void MOMO::CalibrationService::AddPointPair()
{

}

bool CalibrationService::shouldDrawIndicator()
{
	return (mousePoint.x > KINECT_DEPTH_WIDTH && mousePoint.y > 60 && mousePoint.y < 360);
}

void CalibrationService::MouseMove(int x, int y)
{
	if (mouseDown)
	{
		DrawChessboard(x, y);
	}
}

void CalibrationService::MouseUp(int x, int y)
{
	mouseDown = false;
}

void CalibrationService::MouseDown(int x, int y)
{
	mouseDown = true;
}

void CalibrationService::MouseCallback_P(int event, int x, int y, int flags, void* userdata)
{
	switch (event)
	{
	case EVENT_LBUTTONDOWN:
		MouseDown(x, y);
		break;
	
	case EVENT_LBUTTONUP:
		MouseUp(x, y);
		break;

	case EVENT_MOUSEMOVE:
		MouseMove(x, y);
		break;

	default:
		break;
	}
}

void CalibrationService::MouseCallback(int event, int x, int y, int flags, void* userdata)
{
	CalibrationService* calibrationService = reinterpret_cast<CalibrationService*>(userdata);
	calibrationService->MouseCallback_P(event, x, y, flags, NULL);
}

