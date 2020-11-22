#pragma once

#include <boost/bind.hpp>
#include <boost/function.hpp>

#define GAME_WINDOW_SIZE_X 1280
#define GAME_WINDOW_SIZE_Y 720


#define CALIBRATION_PROJECTION_SIZE_X 1280
#define CALIBRATION_PROJECTION_SIZE_Y 720

#define CALIBRATION_UI_SIZE_X 1280
#define CALIBRATION_UI_SIZE_Y 720




#define STABLITY_SERVICE_UPDATE_RATE 10
#define STABILITY_DEVICE_UNSTABLE_CHECK_FRAMES 3
#define STABILITY_DEVICE_STABLE_CHECK_FRAMES 20
#define STABILITY_DEVIATION_THRESHOLD 0.3

#define X_VAL 0
#define Y_VAL 1
#define Z_VAL 2

typedef boost::function<void()> voidcallback;

