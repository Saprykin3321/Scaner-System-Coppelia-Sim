#pragma once

#ifndef Scaner

#include <vector>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <numeric>
#include <cmath>
#include <algorithm>  // sort

#define PI 3.14159265358979323846

//Структура с данными о позиции НПА
struct PositionAUV
{
	double X, Y, Z, Phi, Theta, Psi;
};

//Структура профили. Включает данные о позиции НПА и точки в С.К. камеры
struct Profile
{
	std::vector<cv::Point3d> points_SSK;
	PositionAUV coord;
};

//Параметры цветовой модели HSV
struct HSV_param
{
	short h_min, s_min, v_min, h_max, s_max, v_max;
};


class Scaner
{
protected:
	HSV_param					hsv_parameters;
	//пиксельная координата лазера
	std::vector<double>			y_middle;
	//координаты точек в С.К. камеры
	std::vector<double>			z, x, y;
	//h - расстояние от камеры до лазера, difference_y - разница между соседними пикселями
	//m, b - коэффициенты для расчета дистанции
	double						h, difference_y, m, b;
	//коэффициенты для расчета пространственных координат x и y
	float						a_MNK = 0, b_MNK = 0;
	//вектора для триангуляции и расчета нормалей
	std::vector<cv::Point3i>	surface;
	std::vector<cv::Point3d>	normals;

public:
	std::vector<Profile> profiles;

	Scaner() {};
	Scaner(double dist_cam_to_laser); //конструктор, указывается расстояние h
	~Scaner() {};

	//Функции для расчета пространственных координат
	//Для расчета коэффициентов m и b для расчета дальности
	void calculate_m_b	(double center_y,  double *D_real,
						 float  step,      char *path_to_folder,
						 int resolution_x);
	//Для расчета коэффициентов a и b для расчета координат x и y
	void calculate_aMNK_bMNK();
	//Общая функция для калибровки лазера. 
	//Производится проверка, есть ли калибровочные файлы папке программы
	//Если нет - программа высчитывает их по функциям выше и создает эти файлы
	void calibration_laser	(double center_y,  double *D_real,
							 float  step,      char *path_to_folder,
							 int resolution_x);

	//Геттеры для различных параметров
	double get_m() { return m; };
	double get_b() { return b; };

	float get_a_MNK() { return a_MNK; };
	float get_b_MNK() { return b_MNK; };

	//Обработка изображения, расчет пространственных координат
	//Функция для определения параметров HSV. Накладывается маска из начальных параметров,
	//затем вычисляется среднее по столбцам и разница между этими столбцами
	//если разница больше некоторого числа, то маска подходит, если нет, то увеличиваем параметр яркости
	//впринципе ничего сложного, но детально объяснять текстом будет тяжеловато
	void Find_HSV_Parameters(cv::Mat image);
	//Функция для вычисления пиксельных координат лазера
	void Find_medium_laser(cv::Mat image, int resolution_x);
	//Функция для вычисления пространственных координат
	void Calculate_coordinate(double center_y, int resolution_x, int resolution_y);

	//Эта функция объединяет две выше - Find medium laser и calculate coordinate
	double Scan_frame(cv::Mat image, double center_y, int resolution_x, int resolution_y);

	//Сохранение одного профиля в СК камеры
	void Safe_dots(std::vector<cv::Point3d>& cloud, std::ofstream &out, const char *name);


	//Геттеры для различных параметров
	double get_dif_y() { return difference_y; };

	double get_min_h() { return hsv_parameters.h_min; };
	double get_min_s() { return hsv_parameters.s_min; };
	double get_min_v() { return hsv_parameters.v_min; };

	double get_max_h() { return hsv_parameters.h_max; };
	double get_max_s() { return hsv_parameters.s_max; };
	double get_max_v() { return hsv_parameters.v_max; };


	// Пересчет координат из СК камеры в глобальную СК, сохранение их в файл
	// Функция также запоминает все профили
	void create_profiles(std::vector<cv::Point3d>& cloud,
						 double X,		double Y,		double Z,
						 double phi,	double theta,	double qsi);

	// Сохранение облака точек в файл
	void safe_cloud(std::ofstream &out, const char *name);

	// Триангуляция
	double CalcDist(std::vector<double> p1, std::vector<double> p2);
	void triangul();
	std::vector<cv::Point3i> get_result_triangul() { return surface; };

	// Расчет нормалей (не работает)
	void calc_normals();
	std::vector<cv::Point3d> get_normals() { return normals; };

	// Поиск деформаций на основе разницы по высоте (не работает)
	void find_deformation(const char *original, const char *deformation, double difference, std::ofstream &out, const char *name);
};
#endif // !Scaner_h