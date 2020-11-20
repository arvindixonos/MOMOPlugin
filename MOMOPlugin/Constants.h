#pragma once

#include <boost/bind.hpp>
#include <boost/function.hpp>

#define STABLITY_SERVICE_UPDATE_RATE 10
#define STABILITY_DEVICE_UNSTABLE_CHECK_FRAMES 3
#define STABILITY_DEVICE_STABLE_CHECK_FRAMES 20
#define STABILITY_DEVIATION_THRESHOLD 0.3

#define X_VAL 0
#define Y_VAL 1
#define Z_VAL 2

typedef boost::function<void()> voidcallback;

