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
#include <tinyxml2.h>
#include "CalibrationService.h"
#include "Constants.h"
#include "Utils.h"
#include "MOMOManager.h"


using namespace MOMO;
using namespace cv;
using namespace tinyxml2;

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
	cv::Mat cvRGBImage = MOMOManager::getInstance()->GetRGBMat();

	if (testing)
	{
		ushort* depthPixels = MOMOManager::getInstance()->GetDepthData();
		int x = testPoint.x;
		int y = testPoint.y;
		cv::Point3f worldPoint = MOMOManager::getInstance()->GetWorldPosition(x, y, depthPixels[x + y * KINECT_DEPTH_WIDTH]);
		cv::Point2f projectedPoint = GetProjectedPoint(worldPoint);
		DrawTestingPoint(projectedPoint);
	}
	else
	{
		projectorMat = cv::Mat::ones(projectorMat.rows, projectorMat.cols, CV_8UC3);

		projectorMat.setTo(cv::Scalar(255, 255, 255));

		DrawChessboard(mousePoint.x, mousePoint.y);

		cv::Size patternSize = cv::Size(chessboardX - 1, chessboardY - 1);
		int chessFlags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK;
		foundChessboard = findChessboardCorners(cvRGBImage, patternSize, cvPoints, chessFlags);
		if (foundChessboard)
		{
			cv::Mat gray;
			cv::cvtColor(cvRGBImage, gray, cv::COLOR_RGB2GRAY);
			cornerSubPix(gray, cvPoints, cv::Size(11, 11), cv::Size(-1, -1),
				cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(cvRGBImage, patternSize, cv::Mat(cvPoints), foundChessboard);
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
	int keyPressed = cv::waitKey(1);

	if (keyPressed != -1)
		KeyPressed(keyPressed);
}

void MOMO::CalibrationService::StopService()
{
	started = false;
}

void MOMO::CalibrationService::DrawChessboard(int x, int y)
{
	x -= (GAME_WINDOW_SIZE_X / 2);
	y -= 60;

	if (x < 0 || y < 0 || y > 360)
		return;
	
	x = (int)(((double)x / 640.0) * CALIBRATION_PROJECTION_SIZE_X);
	y = (int)(((double)y / 360.0) * CALIBRATION_PROJECTION_SIZE_Y);

	float w = chessboardSize / chessboardX;
	float h = chessboardSize / chessboardY;
	
	currentProjectorPoints.clear();

	for (int j = 0; j < chessboardY; j++) {
		for (int i = 0; i < chessboardX; i++) {
			int x0 = mapValue(i, 0, chessboardX, 0, chessboardSize);
			int y0 = mapValue(j, 0, chessboardY, 0, chessboardSize);
			if (j > 0 && i > 0) {
				currentProjectorPoints.push_back(cv::Point2f(
					mapValue(x + x0, 0, CALIBRATION_PROJECTION_SIZE_X, 0, 1),
					mapValue(y + y0, 0, CALIBRATION_PROJECTION_SIZE_Y, 0, 1)
				));
			}

			if ((i + j) % 2 == 0)
			{
				cv::rectangle(projectorMat, cv::Rect(x + x0, y + y0, w, h), cv::Scalar(0, 0, 0), FILLED, LINE_8);
			}
		}
	}
}

void MOMO::CalibrationService::DrawTestingPoint(cv::Point2f projectedPoint)
{
	cv::circle(projectorMat, cv::Point2f(mapValue(projectedPoint.x, 0, 1, 0, CALIBRATION_PROJECTION_SIZE_X),
										mapValue(projectedPoint.y, 0, 1, 0, CALIBRATION_PROJECTION_SIZE_Y)), 10, 
										cv::Scalar(0, 255, 0));
}

void MOMO::CalibrationService::AddPointPair()
{
	int nDepthPoints = 0;
	ushort* depthPixels = MOMOManager::getInstance()->GetDepthData();
	for (int i = 0; i < cvPoints.size(); i++) {
		cv::Point3f worldPoint = MOMOManager::getInstance()->GetWorldPosition((int)cvPoints[i].x, (int)cvPoints[i].y, depthPixels[(int)cvPoints[i].x + (int)cvPoints[i].y * KINECT_DEPTH_WIDTH]);
		if (worldPoint.z > 0)   
			nDepthPoints++;
	}
	if (nDepthPoints == (chessboardX - 1)*(chessboardY - 1)) {

		for (int i = 0; i < cvPoints.size(); i++) {
			cv::Point3f worldPoint = MOMOManager::getInstance()->GetWorldPosition((int)cvPoints[i].x, (int)cvPoints[i].y, depthPixels[(int)cvPoints[i].x + (int)cvPoints[i].y * KINECT_DEPTH_WIDTH]);
			pairsKinect.push_back(worldPoint);
			pairsProjector.push_back(currentProjectorPoints[i]);		
		}

		std::cout << "Total Points:" << pairsProjector.size() << "\n";
	}
	else {
		std::cout << "UNABLE TO ADD POINTS!! \n";
	}
}

void CalibrationService::SaveCalibration(string path)
{
	boost::shared_ptr<XMLDocument> doc = boost::make_shared<XMLDocument>();
	XMLElement* calibration = doc->NewElement("CALIBRATION");
	doc->InsertEndChild(calibration);
	for (int i = 0; i < 11; i++) 
	{
		string elementName = "COEFF";
		elementName += std::to_string(i);
		XMLElement* calib = doc->NewElement(elementName.c_str());
		calib->SetText(x(i, 0));
		calibration->InsertEndChild(calib);
	}

	doc->SaveFile(path.c_str());
}

void CalibrationService::LoadCalibration()
{
	boost::shared_ptr<XMLDocument> doc = boost::make_shared<XMLDocument>();
	doc->LoadFile(CALIBRATION_FILE_NAME);
	XMLElement* element = doc->FirstChildElement();
	XMLNode* coeff = element->FirstChild();

	for (int i = 0; i < 11; i++) {
		x(i, 0) = std::stof(coeff->FirstChild()->Value());
		coeff = coeff->NextSibling();
	}
}

std::vector<double> CalibrationService::GetCalibration()
{
	vector<double> coefficients;
	for (int i = 0; i < 11; i++) {
		coefficients.push_back(x(i, 0));
	}
	return coefficients;
}

cv::Point2f CalibrationService::GetProjectedPoint(cv::Point3f worldPoint)
{
	float a = x(0, 0)*worldPoint.x + x(1, 0)*worldPoint.y + x(2, 0)*worldPoint.z + x(3, 0);
	float b = x(4, 0)*worldPoint.x + x(5, 0)*worldPoint.y + x(6, 0)*worldPoint.z + x(7, 0);
	float c = x(8, 0)*worldPoint.x + x(9, 0)*worldPoint.y + x(10, 0)*worldPoint.z + 1;
	cv::Point2f projectedPoint(a / c, b / c);
	return projectedPoint;
}

void CalibrationService::Calibrate()
{
	int nPairs = pairsKinect.size();
	A.set_size(nPairs * 2, 11);
	y.set_size(nPairs * 2, 1);

	for (int i = 0; i < nPairs; i++) {
		A(2 * i, 0) = pairsKinect[i].x;
		A(2 * i, 1) = pairsKinect[i].y;
		A(2 * i, 2) = pairsKinect[i].z;
		A(2 * i, 3) = 1;
		A(2 * i, 4) = 0;
		A(2 * i, 5) = 0;
		A(2 * i, 6) = 0;
		A(2 * i, 7) = 0;
		A(2 * i, 8) = -pairsKinect[i].x * pairsProjector[i].x;
		A(2 * i, 9) = -pairsKinect[i].y * pairsProjector[i].x;
		A(2 * i, 10) = -pairsKinect[i].z * pairsProjector[i].x;

		A(2 * i + 1, 0) = 0;
		A(2 * i + 1, 1) = 0;
		A(2 * i + 1, 2) = 0;
		A(2 * i + 1, 3) = 0;
		A(2 * i + 1, 4) = pairsKinect[i].x;
		A(2 * i + 1, 5) = pairsKinect[i].y;
		A(2 * i + 1, 6) = pairsKinect[i].z;
		A(2 * i + 1, 7) = 1;
		A(2 * i + 1, 8) = -pairsKinect[i].x * pairsProjector[i].y;
		A(2 * i + 1, 9) = -pairsKinect[i].y * pairsProjector[i].y;
		A(2 * i + 1, 10) = -pairsKinect[i].z * pairsProjector[i].y;

		y(2 * i, 0) = pairsProjector[i].x;
		y(2 * i + 1, 0) = pairsProjector[i].y;
	}

	dlib::qr_decomposition<dlib::matrix<double, 0, 11> > qrd(A);
	x = qrd.solve(y);
}

void CalibrationService::KeyPressed(int key)
{
	if (key == ' ') {
		AddPointPair();
	}
	else if (key == 'q') {
		chessboardSize -= 20;
	}
	else if (key == 'w') {
		chessboardSize += 20;
	}
	else if (key == 'c') {
		Calibrate();
		//testing = true;
	}
	else if (key == 's') {
		SaveCalibration(CALIBRATION_FILE_NAME);
	}
	else if (key == 't')
	{
		testing = !testing;
	
		if (testing)
		{
			LoadCalibration();
		}
	}
}

bool CalibrationService::shouldDrawIndicator()
{
	return (mousePoint.x > KINECT_DEPTH_WIDTH && mousePoint.y > 60 && mousePoint.y < 360);
}

void CalibrationService::MouseMove(int x, int y)
{
	if (mouseDown)
	{
		mousePoint = cv::Point(x, y);
	}
}

void CalibrationService::MouseUp(int x, int y)
{
	mouseDown = false;
}

void CalibrationService::MouseDown(int x, int y)
{
	mouseDown = true;

	if (testing)
	{
		if (x < KINECT_DEPTH_WIDTH)
		{
			x = (int)(((double)x / 640.0) * CALIBRATION_PROJECTION_SIZE_X);
			y = (int)(((double)y / 360.0) * CALIBRATION_PROJECTION_SIZE_Y);

			testPoint.x = x;
			testPoint.y = y;
		}
	}
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

