/*
 * OpenCV helper
 * include OpenCV headers and link libraries.
 */

#pragma once

#pragma warning(disable: 4819 4996)

#include <opencv2/core/version.hpp>
 
#define OPEPCV_LIB_PREFIX   "opencv_"
 
#ifdef _DEBUG
    #define OPENCV_LIB_POSTFIX  "d.lib"
#else
    #define OPENCV_LIB_POSTFIX  ".lib"
#endif
 
#define STR_EXPAND(tok) #tok
#define STR(tok) STR_EXPAND(tok)
 
#define OPENCV_VERSION_STRING STR(CV_VERSION_MAJOR) STR(CV_VERSION_MINOR) STR(CV_VERSION_REVISION)
 
#define OPENCV_LIB_EXPAND(x) OPEPCV_LIB_PREFIX x OPENCV_VERSION_STRING OPENCV_LIB_POSTFIX

/* usage
	#pragma comment(lib, OPENCV_LIB_EXPAND("core"))
 */