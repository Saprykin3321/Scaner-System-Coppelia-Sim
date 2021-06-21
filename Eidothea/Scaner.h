#pragma once

#ifndef Scaner

#include <vector>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <numeric>
#include <cmath>
#include <algorithm>  // sort

#define PI 3.14159265358979323846

//��������� � ������� � ������� ���
struct PositionAUV
{
	double X, Y, Z, Phi, Theta, Psi;
};

//��������� �������. �������� ������ � ������� ��� � ����� � �.�. ������
struct Profile
{
	std::vector<cv::Point3d> points_SSK;
	PositionAUV coord;
};

//��������� �������� ������ HSV
struct HSV_param
{
	short h_min, s_min, v_min, h_max, s_max, v_max;
};


class Scaner
{
protected:
	HSV_param					hsv_parameters;
	//���������� ���������� ������
	std::vector<double>			y_middle;
	//���������� ����� � �.�. ������
	std::vector<double>			z, x, y;
	//h - ���������� �� ������ �� ������, difference_y - ������� ����� ��������� ���������
	//m, b - ������������ ��� ������� ���������
	double						h, difference_y, m, b;
	//������������ ��� ������� ���������������� ��������� x � y
	float						a_MNK = 0, b_MNK = 0;
	//������� ��� ������������ � ������� ��������
	std::vector<cv::Point3i>	surface;
	std::vector<cv::Point3d>	normals;

public:
	std::vector<Profile> profiles;

	Scaner() {};
	Scaner(double dist_cam_to_laser); //�����������, ����������� ���������� h
	~Scaner() {};

	//������� ��� ������� ���������������� ���������
	//��� ������� ������������� m � b ��� ������� ���������
	void calculate_m_b	(double center_y,  double *D_real,
						 float  step,      char *path_to_folder,
						 int resolution_x);
	//��� ������� ������������� a � b ��� ������� ��������� x � y
	void calculate_aMNK_bMNK();
	//����� ������� ��� ���������� ������. 
	//������������ ��������, ���� �� ������������� ����� ����� ���������
	//���� ��� - ��������� ����������� �� �� �������� ���� � ������� ��� �����
	void calibration_laser	(double center_y,  double *D_real,
							 float  step,      char *path_to_folder,
							 int resolution_x);

	//������� ��� ��������� ����������
	double get_m() { return m; };
	double get_b() { return b; };

	float get_a_MNK() { return a_MNK; };
	float get_b_MNK() { return b_MNK; };

	//��������� �����������, ������ ���������������� ���������
	//������� ��� ����������� ���������� HSV. ������������� ����� �� ��������� ����������,
	//����� ����������� ������� �� �������� � ������� ����� ����� ���������
	//���� ������� ������ ���������� �����, �� ����� ��������, ���� ���, �� ����������� �������� �������
	//��������� ������ ��������, �� �������� ��������� ������� ����� ����������
	void Find_HSV_Parameters(cv::Mat image);
	//������� ��� ���������� ���������� ��������� ������
	void Find_medium_laser(cv::Mat image, int resolution_x);
	//������� ��� ���������� ���������������� ���������
	void Calculate_coordinate(double center_y, int resolution_x, int resolution_y);

	//��� ������� ���������� ��� ���� - Find medium laser � calculate coordinate
	double Scan_frame(cv::Mat image, double center_y, int resolution_x, int resolution_y);

	//���������� ������ ������� � �� ������
	void Safe_dots(std::vector<cv::Point3d>& cloud, std::ofstream &out, const char *name);


	//������� ��� ��������� ����������
	double get_dif_y() { return difference_y; };

	double get_min_h() { return hsv_parameters.h_min; };
	double get_min_s() { return hsv_parameters.s_min; };
	double get_min_v() { return hsv_parameters.v_min; };

	double get_max_h() { return hsv_parameters.h_max; };
	double get_max_s() { return hsv_parameters.s_max; };
	double get_max_v() { return hsv_parameters.v_max; };


	// �������� ��������� �� �� ������ � ���������� ��, ���������� �� � ����
	// ������� ����� ���������� ��� �������
	void create_profiles(std::vector<cv::Point3d>& cloud,
						 double X,		double Y,		double Z,
						 double phi,	double theta,	double qsi);

	// ���������� ������ ����� � ����
	void safe_cloud(std::ofstream &out, const char *name);

	// ������������
	double CalcDist(std::vector<double> p1, std::vector<double> p2);
	void triangul();
	std::vector<cv::Point3i> get_result_triangul() { return surface; };

	// ������ �������� (�� ��������)
	void calc_normals();
	std::vector<cv::Point3d> get_normals() { return normals; };

	// ����� ���������� �� ������ ������� �� ������ (�� ��������)
	void find_deformation(const char *original, const char *deformation, double difference, std::ofstream &out, const char *name);
};
#endif // !Scaner_h