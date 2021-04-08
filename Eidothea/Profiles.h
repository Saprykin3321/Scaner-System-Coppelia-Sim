#pragma once

#ifndef Autofilter_h
#define Autofilter_h

#include <vector>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <numeric>
#include <cmath>

#define PI 3.14159265358979323846

void safe_data		(std::vector<double>& x,           std::vector<double>& y,  std::vector<double>& z,
					 std::vector<cv::Point3d>& cloud,  std::ofstream &out,      char *name);

void read_file_for_MNK(std::vector<double>& s, std::vector<double>& x_cub);
void MNK(std::vector<double>& x, std::vector<double>& y, float *a, float *b);

struct PositionAUV
{
	double X, Y, Z, Phi, Theta, Psi;
};

struct Profile
{
	std::vector<cv::Point3d> points_SSK;
	PositionAUV coord;
};

struct HSV_param
{
	short h_min, s_min, v_min, h_max, s_max, v_max;
};


class Profiles
{
protected:
	HSV_param					hsv_parameters;
	std::vector<double>			y_middle;
	std::vector<double>			z, x, y;
	double						h, difference_y, m, b;
	std::vector<cv::Point3i>	surface;
	std::vector<cv::Point3d>	normals;

	float a_MNK = 0, b_MNK = 0; //Переделать

public:
	std::vector<Profile> profiles;

	Profiles() {};
	Profiles(double dist_cam_to_laser);
	~Profiles() {};

	// F I N D   M & B
	void calculate_m_b(double center_y, double *D_real,
					  float  step, char *path_to_folder);
	void Find_coeff(double center_y,	double *D_real,
					float  step,		char *path_to_folder);

	double get_m() { return m; };
	double get_b() { return b; };

	// P R O C E S S I N G   I M A G E //
	void Find_HSV_Parameters(cv::Mat image);
	void Find_medium_laser(cv::Mat image);
	void Calculate_coordinate(double center_y);
	void Safe_dots(std::vector<cv::Point3d>& cloud, std::ofstream &out, const char *name);
	double Scan_frame(cv::Mat image, double center_y);

	double get_dif_y() { return difference_y; };

	double get_min_h() { return hsv_parameters.h_min; };
	double get_min_s() { return hsv_parameters.s_min; };
	double get_min_v() { return hsv_parameters.v_min; };

	double get_max_h() { return hsv_parameters.h_max; };
	double get_max_s() { return hsv_parameters.s_max; };
	double get_max_v() { return hsv_parameters.v_max; };

	void MNK(); //Переделать

	// P R O F I L E S //
	void create_profiles(std::vector<cv::Point3d>& cloud,
						 double X,		double Y,		double Z,
						 double phi,	double theta,	double qsi);

	// T R I A N G U L A T I O N //
	double CalcDist(std::vector<double> p1, std::vector<double> p2);
	void triangul();
	std::vector<cv::Point3i> get_result_triangul() { return surface; };

	// N O R M A L S //
	void calc_normals();
	std::vector<cv::Point3d> get_normals() { return normals; };

	// O U T P U T   I N   F I L E //
	void safe_cloud(std::ofstream &out, const char *name);
	void safe(std::ofstream &out, const char *name);
};
#endif // !Autofilter_h