#include "Camera.h"

char path_to_calibration_camera_file[] = "XML_file/calibration.xml";

//Camera::Camera(bool out, short width, short height)
//{
//	cap.open(0);
//	if (!cap.isOpened())
//		std::cout << "Problem connecting to cam " << std::endl;
//	else
//		std::cout << "Successfuly connected to camera " << std::endl;
//	cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
//	cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
//}

void Camera::set_param(bool out, short width, short height)
{
	cap.open(0);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
}

void Camera::show_video()
{
	for (;;)
	{
		cap.read(frame);
		if (frame.empty())
		{
			std::cout << "ERROR! Can't read frame";
			break;
		}
		cv::imshow("Camera", frame);
		if (cv::waitKey(10) == 'q')
		{
			std::cout << "Closing camera..." << std::endl;
			Sleep(100);
			break;
		}
	}
	cap.release();
	cv::destroyAllWindows();
}

void Camera::show_video_undistord()
{
	for (;;)
	{
		cap.read(frame);
		if (frame.empty())
		{
			std::cout << "ERROR! Can't read frame";
			break;
		}
		if (CamMat.empty() || DistCoef.empty())
		{
			std::cout << "ERROR! Can't read CamMat or DistCoef";
			break;
		}
		undistort(frame, undistord_frame, CamMat, DistCoef);
		imshow("Camera", undistord_frame);
		if (cv::waitKey(10) == 'q')
		{
			std::cout << "Closing camera..." << std::endl;
			Sleep(100);
			break;
		}
	}
	cap.release();
	cv::destroyAllWindows();
}

void Camera::take_snapshot_undistord(char *name_file)
{
	for (;;)
	{
		cap.read(frame);
		if (frame.empty())
		{
			std::cout << "ERROR! Can't read frame";
			break;
		}
		if (CamMat.empty() || DistCoef.empty())
		{
			std::cout << "ERROR! Can't read CamMat or DistCoef";
			break;
		}
		undistort(frame, undistord_frame, CamMat, DistCoef);
		Sleep(5);
		imshow("Camera", undistord_frame);
		char button = cv::waitKey(30);
		if (button == 's') {
			sprintf_s(filename, name_file, count_frame);
			cv::waitKey(10);
			imshow("Main camera", undistord_frame);
			imwrite(filename, undistord_frame);
			std::cout << "Frame_" << count_frame << std::endl;
			count_frame++;
		}

		if (button == 'q') {
			std::cout << "Closing..." << std::endl;
			Sleep(100);
			cv::destroyWindow("Main camera");
			break;
		}
	}
	count_frame = 1;
	cap.release();
	cv::destroyAllWindows();
}

void Camera::first_calibration(short numCornersHor,   short numCornersVer,
								char *path_to_folder,  char *path_to_first_pic)
{
	int numSquares = numCornersHor * numCornersVer;
	cv::Size board_sz = cv::Size(numCornersHor, numCornersVer);

	std::vector<std::vector<cv::Point3f>> object_points;
	std::vector<std::vector<cv::Point2f>> image_points;
	std::vector<cv::Point2f> corners;

	std::vector<cv::Mat> images;
	cv::Mat gray_images;

	std::vector<cv::String> fn;
	cv::glob(path_to_folder, fn, false);

	size_t count_img = fn.size();
	for (size_t i = 0; i < count_img; i++)
		images.push_back(cv::imread(fn[i]));

	std::vector<cv::Point3f> obj;
	for (int j = 0; j < numSquares; j++)
		obj.push_back(cv::Point3f(j / numCornersHor, j%numCornersHor, 0.0f));

	for (int i = 0; i < count_img; i++)
	{
		cv::cvtColor(images[i], gray_images, cv::COLOR_BGR2GRAY);
		bool found = findChessboardCorners(images[i], board_sz, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
		if (found)
		{
			object_points.push_back(obj);
			cornerSubPix(gray_images, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1));
			image_points.push_back(corners);
			drawChessboardCorners(gray_images, board_sz, corners, found);
		}
	}

	cv::Mat image = cv::imread(path_to_first_pic);

	cv::FileStorage fs(path_to_calibration_camera_file, cv::FileStorage::WRITE);

	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	CamMat.ptr<float>(0)[0] = 1;
	CamMat.ptr<float>(1)[1] = 1;
	calibrateCamera(object_points, image_points, image.size(), CamMat, DistCoef, rvecs, tvecs);

	//----- Ошибка ReEr
	std::vector<cv::Point2f> imagePoints2;
	int p, totalPoints = 0;
	double totalErr = 0, err;
	std::vector<float> perViewErrors;
	perViewErrors.resize(object_points.size());
	for (p = 0; p < (int)object_points.size(); p++)
	{
		projectPoints(cv::Mat(object_points[p]), rvecs[p], tvecs[p],
					  CamMat, DistCoef, imagePoints2);
		err = norm(cv::Mat(image_points[p]), cv::Mat(imagePoints2), cv::NORM_L2);
		int n = (int)object_points[p].size();
		perViewErrors[p] = (float)std::sqrt(err*err / n);
		totalErr += err * err;
		totalPoints += n;
	}

	ReEr = sqrt(totalErr / totalPoints);

	fs << "cameraMatrix" << CamMat;
	fs << "distCoeffs" << DistCoef;
	fs << "ReEr" << ReEr;
	fs.release();
	std::cout << "Calibration success" << std::endl;
}

double Camera::calibration(short numCornersHor, short numCornersVer,
						 char *path_to_folder, char *path_to_first_pic)
{
	cv::FileStorage fs(path_to_calibration_camera_file, cv::FileStorage::READ);
	if (fs.isOpened())
	{
		std::cout << "CALIBRATION CAMERA FILE LOAD SUCCES" << std::endl;
		fs["cameraMatrix"]  >>  CamMat;
		fs["distCoeffs"]    >>  DistCoef;
		fs["ReEr"]          >>  ReEr;
		return 0;
	}
	else
	{
		clock_t start = clock();
		std::cout << "NOT FIND CALIBRATION FILE" << std::endl;
		first_calibration(numCornersHor,   numCornersVer,
			              path_to_folder,  path_to_first_pic);
		clock_t end = clock();
		return (double)(end - start)/1000;
	}
}

void Camera::calibration_laser()
{
	for (;;)
	{
		int y = 100;
		cap.read(frame);
		if (frame.empty())
		{
			std::cout << "ERROR! Can't read frame";
			break;
		}
		if (CamMat.empty() || DistCoef.empty())
		{
			std::cout << "ERROR! Can't read CamMat or DistCoef";
			break;
		}
		undistort(frame, undistord_frame, CamMat, DistCoef);
		while (y < undistord_frame.rows)
		{
			cv::line(undistord_frame, cvPoint(0, y), cvPoint(undistord_frame.cols, y),
			cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
			y += 100;
		}
		cv::imshow("Camera", undistord_frame);
		if (cv::waitKey(10) == 'q')
		{
			std::cout << "Closing..." << std::endl;
			Sleep(100);
			cv::destroyWindow("Main camera");
			break;
		}
	}
	cap.release();
	cv::destroyAllWindows();
}

double Camera::get_center_y()
{
	if (CamMat.empty())
	{
		std::cout << "ERROR! Can't read CamMat";
	}
	else
	{
		return center_y = CamMat.at<double>(cv::Point(2, 1));
	}
}