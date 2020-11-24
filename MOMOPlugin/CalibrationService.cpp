#include <stdio.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <math.h>
#include <opencv2/calib3d.hpp>
#include <vtkContextMouseEvent.h>
#include <vtkContextView.h>
#include <vtkContextScene.h>
#include "CalibrationService.h"
#include "Constants.h"
#include "Utils.h"
#include "MOMOManager.h"

using namespace MOMO;

MOMO::CalibrationService::CalibrationService()
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
	uiPainter = boost::make_shared<visualization::PCLPainter2D>();
	uiRenderer = uiPainter->GetScene()->GetRenderer();
	uiWindow = uiRenderer->GetRenderWindow();
	uiWindow->SetWindowName("UI Window");
	uiWindow->SetPosition(0, 0);
	uiPainter->setWindowSize(CALIBRATION_UI_SIZE_X, CALIBRATION_UI_SIZE_Y);
	uiPainter->GetScene()->GetRenderer()->SetViewport(0.0, 0.0, 1.0, 1.0);
	uiPainter->setBrushColor(255, 0, 0, 255);


	projectorPainter = boost::make_shared<visualization::PCLPainter2D>();
	projectorRenderer = projectorPainter->GetScene()->GetRenderer();
	projectorWindow = projectorRenderer->GetRenderWindow();
	projectorWindow->SetWindowName("Projection Window");
	projectorWindow->SetPosition(1920, 0);
	projectorPainter->setWindowSize(CALIBRATION_PROJECTION_SIZE_X, CALIBRATION_PROJECTION_SIZE_Y);
	projectorPainter->GetScene()->GetRenderer()->SetViewport(0.0, 0.0, 1.0, 1.0);
	projectorPainter->setBrushColor(255, 0, 0, 255);

	uiWindowInteractionHandler = new InteractionHandler();
	uiWindow->GetInteractor()->SetInteractorStyle(uiWindowInteractionHandler);

	started = true;
}

void MOMO::CalibrationService::UpdateService()
{
	//if (kinect.isNewFrame()) 
	{
		/*depthPixels = kinect.getDepthRawPixels();
		rgbImage.setFromPixels(kinect.getImagePixels());
		*/
		cv::Mat cvRgbImage = MOMOManager::getInstance()->GetRGBMat();


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
			int x, y;
			uiWindowInteractionHandler->GetMousePosition(x, y);
			//printf("%d %d \n", x, y);
			
			DrawChessboard(x, y, chessboardSize);

			//cvRgbImage = ofxCv::toCv(rgbImage.getPixels());
			cv::Size patternSize = cv::Size(chessboardX - 1, chessboardY - 1);
			int chessFlags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK;
			foundChessboard = findChessboardCorners(cvRgbImage, patternSize, cvPoints, chessFlags);
			if (foundChessboard) 
			{
				cv::Mat gray;
				cvtColor(cvRgbImage, gray, CV_RGB2GRAY);
				cornerSubPix(gray, cvPoints, cv::Size(11, 11), cv::Size(-1, -1),
					cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
				drawChessboardCorners(cvRgbImage, patternSize, cv::Mat(cvPoints), foundChessboard);
			}
		}
	}

	uiPainter->spinOnce();
	projectorPainter->spinOnce();
}

void MOMO::CalibrationService::StopService()
{
	uiPainter->Delete();
	projectorPainter->Delete();

	started = false;
}

void MOMO::CalibrationService::DrawChessboard(int x, int y, int chessboardSize)
{
	float w = chessboardSize / chessboardX;
	float h = chessboardSize / chessboardY;
	currentProjectorPoints.clear();
	projectorPainter->clearFigures();
	projectorPainter->setBackgroundColor(255, 255, 255);
	projectorPainter->setPenColor(0, 0, 0, 255);
	projectorPainter->setBrushColor(0, 0, 0, 255);
	projectorPainter->clearTransform();
	projectorPainter->translatePen(x, y);

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
 				projectorPainter->addRect(x0, y0, w, h);
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


