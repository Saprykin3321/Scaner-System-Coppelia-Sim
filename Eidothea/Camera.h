#pragma once

#ifndef Camera_h
#define Camera_h

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/videoio/videoio_c.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <iostream>
#include <stdio.h>
#include <time.h> 

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#else
#include <unistd.h>
#endif

class Camera
{
	cv::Mat					frame;
	cv::Mat					CamMat = cv::Mat(3, 3, CV_32FC1);
	cv::Mat					DistCoef;
	cv::Mat					undistord_frame;
	cv::VideoCapture		cap;
	double					center_y;
	double					ReEr;
	char					filename[100]; // For filename
	char					namefile[100]; // For filename
	int						count_frame = 1;

public:
	Camera() {};
	~Camera() {};

	void set_param(bool out, short width, short height);
	void show_video();
	void show_video_undistord();
	void take_snapshot_undistord(char *name_file);

	void first_calibration(short numCornersHor,		short numCornersVer,
						   char *path_to_folder,	char *path_to_first_pic);

	double calibration(short numCornersHor, short numCornersVer,
					 char *path_to_folder, char *path_to_first_pic);

	void calibration_laser();

	//// GET PARAMETERS
	double get_center_y();
	double get_ReEr() { return ReEr; };
	cv::Mat get_CM() { return CamMat; };
	cv::Mat get_D() { return DistCoef; };
};

#endif // !Camera_h
