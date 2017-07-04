#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <ctime>
#include <map>
using namespace std;
using namespace cv;

#ifdef USE_GPU
	#include <opencv2/cudafeatures2d.hpp>
	using cuda::GpuMat;
#endif



