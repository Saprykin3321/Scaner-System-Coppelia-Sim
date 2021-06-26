#include "Scaner.h"

//Некоторые нужные для работы функции
#define ranged_for(var, min, max, step) for(auto var = (min); var < (max); var += (step))

struct myclass {
	bool operator() (cv::Point pt1, cv::Point pt2)
	{
		if (pt1.x != pt2.x)
		{
			return (pt1.x < pt2.x);
		}
		return (pt1.y < pt2.y);
	}
} myobject;

bool IsNan(double a, double b, double c)
{
	return (isnan(a) || isnan(b) || isnan(c));
}

void getLine(double x1, double y1, double x2, double y2, double& a, double& b, double& c)
{
	a = y1 - y2;
	b = x2 - x1;
	c = x1 * y2 - x2 * y1;
}

clock_t start1, end;

struct ElementInfo
{
	ElementInfo(const std::vector<cv::Point>& contour, cv::Mat& thresholded)
	{
		cv::Rect _upright = boundingRect(contour);

		//Переменные для скользящего среднего
		int moving_average = 3;
		int current_index = 0;
		bool allow_pushing = false;
		cv::Point2f* buffer = new cv::Point2f[moving_average];

		//Попиксельное прохождение всех пикселей в прмоугольнике вокруг контура
		for (int i = _upright.x; i < _upright.x + _upright.width; ++i)
		{
			std::vector<cv::Point2f> point_sum;

			uchar* begin_ptr = thresholded.ptr<uchar>(_upright.y);
			begin_ptr += i;

			for (int j = _upright.y; j < _upright.y + _upright.height; ++j)
			{
				uchar grayscale = *begin_ptr;
				begin_ptr += thresholded.step;

				//Если данная точка встречается в контуре
				if (grayscale > 128)
				{
					point_sum.push_back(cv::Point2f(i, j));
				}
			}

			if (point_sum.size() > 0)
			{
				cv::Point2f medium_point;

				for (int pt = 0; pt < point_sum.size(); pt++)
				{
					medium_point += point_sum[pt];
				}

				medium_point = medium_point / (double)point_sum.size();
				
				//Расчёт скользящего среднего
				buffer[current_index] = medium_point / moving_average;
				cv::Point2f ma = cv::Point2f(0, 0);
				for (int j = 0; j < moving_average; j++)
				{
					ma += buffer[j];
				}

				//Разрешение записи в массив, только при условии
				if (current_index == moving_average - 1)
				{
					allow_pushing = true;
				}

				current_index = (current_index + 1) % moving_average;

				if (allow_pushing)
				{
					medium_points.push_back(ma);
				}
			}
		}

		delete[] buffer;
	}

	std::vector<cv::Point> contour;
	std::vector<cv::Point2f> medium_points;
};

//Пути к калибровочным файлам
char path_to_calibration_distance_file[] = "Calibration_file/laser_distantion.xml";
char path_to_calibration_width_file[] = "Calibration_file/laser_width.xml";

//Конструктор
Scaner::Scaner(double dist_cam_to_laser)
{
	hsv_parameters.h_min = 0;
	hsv_parameters.s_min = 0;
	hsv_parameters.v_min = 170;
	hsv_parameters.h_max = 255;
	hsv_parameters.s_max = 255;
	hsv_parameters.v_max = 255;
	difference_y = 0;
	m = 0;
	b = 0;
	h = dist_cam_to_laser;
}


//Расчет m и b
void Scaner::calculate_m_b(double center_y, double *D_real,
						  float step,      char *path_to_folder,
						  int resolution_x)
{
	std::vector<cv::String> fn;
	cv::glob(path_to_folder, fn, false);
	size_t count = fn.size();
	double Q_real = 0, x_y = 0, x_2 = 0;
	double sum_x = 0, sum_y = 0, sum_x_y = 0, sum_x_2 = 0;
	double difference_y = 0;
	
	for (size_t i = 0; i < count; i++)
	{
		Find_medium_laser(cv::imread(fn[i]), resolution_x);
		difference_y = abs(center_y - *(std::find_if(y_middle.rbegin(), y_middle.rend(), [](int i) { return i > 0; })));

		Q_real = atan2(h, *D_real);
		x_y = difference_y * Q_real;
		x_2 = difference_y * difference_y;

		sum_x += difference_y;
		sum_y += Q_real;
		sum_x_y += x_y;
		sum_x_2 += x_2;

		*D_real += step;
		y_middle.clear();
	}

	m = ((count*sum_x_y) - (sum_x*sum_y)) / ((count*sum_x_2) - (sum_x*sum_x));
	b = ((sum_y)-(m*sum_x)) / count;

	cv::FileStorage fs(path_to_calibration_distance_file, cv::FileStorage::WRITE);
	fs << "m" << m;
	fs << "b" << b;
}

//Расчет a_MNK и b_MNK
void Scaner::calculate_aMNK_bMNK()
{
	std::string line;
	float a = 0, b = 0, sumx = 0, sumx2 = 0, sumy = 0, sumxy = 0;
	int n = 0;
	char t = ';';

	std::ifstream fileMNK("Calibration_file/dataMNK.csv");

	while (getline(fileMNK, line))
	{
		fileMNK >> a >> t >> b;
		sumx += a;
		sumy += b;
		sumx2 += a * a;
		sumxy += a * b;
		n += 1;
	}
	fileMNK.close();

	a_MNK = (n*sumxy - (sumx*sumy)) / (n*sumx2 - sumx * sumx);
	b_MNK = (sumy - a_MNK * sumx) / n;
	cv::FileStorage fs_w(path_to_calibration_width_file, cv::FileStorage::WRITE);
	fs_w << "a_MNK" << a_MNK;
	fs_w << "b_MNK" << b_MNK;
}

void Scaner::calibration_laser(double center_y,  double *D_real,
							   float step,	     char *path_to_folder,
							   int resolution_x)
{
	//Дистанция
	cv::FileStorage fs(path_to_calibration_distance_file, cv::FileStorage::READ);
	if (fs.isOpened())	{
		std::cout << "CALIBRATION DISTANCE FILE LOAD SUCCES" << std::endl;
		fs["m"] >> m;
		fs["b"] >> b;
	}
	else	{
		std::cout << "ERROR TO LOAD CALIBRATION DISTANCE FILE" << std::endl;
		calculate_m_b(center_y, D_real, step, path_to_folder, resolution_x);
	}
	//Ширина сцены
	cv::FileStorage fs_w(path_to_calibration_width_file, cv::FileStorage::READ);
	if (fs_w.isOpened()) {
		std::cout << "CALIBRATION WIDTH FILE LOAD SUCCES" << std::endl;
		fs_w["a_MNK"] >> a_MNK;
		fs_w["b_MNK"] >> b_MNK;
	}
	else {
		std::cout << "ERROR TO LOAD CALIBRATION WIDTH FILE" << std::endl;
		calculate_aMNK_bMNK();
	}
}


// Обработка изображения

void Scaner::Find_HSV_Parameters(cv::Mat image)
{
	std::vector<double>		y_mid;
	std::vector<short>		mass_for_y;
	std::vector<cv::Point>	WhiteCoordinates;
	cv::Mat					hsv;
	cv::Mat					mask;


	cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
	for (;;)
	{
		y_mid.clear();
		y_mid.shrink_to_fit();
		mass_for_y.clear();
		WhiteCoordinates.clear();

		cv::Scalar hsv_l(hsv_parameters.h_min, hsv_parameters.s_min, hsv_parameters.v_min);
		cv::Scalar hsv_h(hsv_parameters.h_max, hsv_parameters.s_max, hsv_parameters.v_max);
		cv::inRange(hsv, hsv_l, hsv_h, mask);

		findNonZero(mask, WhiteCoordinates);

		sort(WhiteCoordinates.begin(), WhiteCoordinates.end(), myobject);

		int count1 = 0, count2 = 0, count_y = 0, count_x = 0, min_id = 0, max_id = 0;
		long double sum_y = 0, y_y = 0, x_x = 0;

		while (count1 < (WhiteCoordinates.size() - 1))
		{
			if (WhiteCoordinates.at(count2).x == WhiteCoordinates.at(count2 + 1).x)
			{
				count_y += 1;
				mass_for_y.push_back(WhiteCoordinates.at(count2).y);
			}
			else
			{
				count_y += 1;
				mass_for_y.push_back(WhiteCoordinates.at(count2).y);
				for (int j = 0; j < mass_for_y.size(); j++)
				{
					sum_y += mass_for_y.at(j);
				}

				y_y = sum_y / count_y;
				x_x = WhiteCoordinates.at(count2).x;

				y_mid.push_back(y_y);

				y_y = 0;
				x_x = 0;
				count_y = 0;
				sum_y = 0;
				mass_for_y.clear();
			}

			count1 += 1;
			count2 += 1;
		}
		long double y_last = y_mid.at(0);
		for (int b = 1; b < y_mid.size(); b++)
		{
			difference_y = difference_y + abs(y_mid.at(b) - y_last);
			y_last = y_mid.at(b);
		}
		difference_y = difference_y / (y_mid.size());
		if ((difference_y > 0.20) & (hsv_parameters.v_min < 255))
		{
			hsv_parameters.v_min += 1;
		}
		else
		{
			break;
		}
	}
}

// Определение пиксельных координат лазера
void Scaner::Find_medium_laser(cv::Mat image, int resolution_x)
{
	cv::Mat imgHSV;
	cv::Mat imgThresholded;
	cv::cvtColor(image, imgHSV, cv::COLOR_BGR2HSV);
	cv::inRange(imgHSV, 
				cv::Scalar(hsv_parameters.h_min, hsv_parameters.s_min, hsv_parameters.v_min), 
				cv::Scalar(hsv_parameters.h_max, hsv_parameters.s_max, hsv_parameters.v_max),
				imgThresholded);

	std::vector<std::vector<cv::Point>> contours;	//Обнаруженные контуры

	findContours(imgThresholded, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE); //Обнаружение контуров

	//Строим структуры
	std::vector<ElementInfo> elemInfos;
	for (auto contour : contours)
	{
		elemInfos.push_back(ElementInfo(contour, imgThresholded));
	}

	cvtColor(imgThresholded, imgThresholded, cv::COLOR_GRAY2BGR);

	std::vector<cv::Point2i> point_point;
	std::vector<cv::Point2i> new_point;
	for (int i = 1; i <= resolution_x; i++)
	{
		point_point.push_back(cv::Point2i(i, NULL));
	}
	//std::cout << "point_point " << point_point.size() << std::endl;
	for (const ElementInfo& elin : elemInfos)
	{
		for (cv::Point pt : elin.medium_points)
		{
			new_point.push_back(pt);
		}
	}
	point_point.insert(point_point.end(), new_point.begin(), new_point.end());
	sort(point_point.begin(), point_point.end(), myobject);
	auto iter_x = point_point.cbegin();
	iter_x++;
	for (std::vector<cv::Point>::iterator it = point_point.begin(); it != point_point.end();)
	{
		if (it->x == iter_x->x)
		{
			point_point.erase(it);
		}
		else
		{
			it++;
			iter_x++;
			if (iter_x == point_point.end()) break;
		}
	}
	for (int i = 0; i < point_point.size(); i++)
	{
		y_middle.push_back(point_point.at(i).y);
	}
	//Отрисовка элементов
	/*for (const ElementInfo& elin : elemInfos)
	{
		for (cv::Point pt : elin.medium_points)
		{
			imgThresholded.at<cv::Vec3b>(pt) = cv::Vec3b(255, 100, 100);
		}

		for (cv::Point pt : elin.contour)
		{
			imgThresholded.at<cv::Vec3b>(pt) = cv::Vec3b(255, 0, 255);
		}
	}
	imshow("Результат", imgThresholded);
	cv::waitKey(0);*/
}

//Расчет пространственных координат
void Scaner::Calculate_coordinate(double center_y, int resolution_x, int resolution_y)
{
	if (m == 0 || b == 0) {
		std::cout << "Error: coefficient m or b is not defined." << std::endl;
	}
	else if (a_MNK == 0 || b == 0)
	{
		std::cout << "Error: coefficient a_MNK or b_MNK is not defined." << std::endl;
	}
	else {
		double pfc = 0;
		std::vector<double> z_laser;
		float z_laser_calc = 0, z_calc = 0;
		for (int i = 0; i < y_middle.size(); i++)
		{
			if (y_middle.at(i) == 0) //Разрывы
			{
				pfc = 0;
				z_laser_calc = 0;
				z_calc = 0;
			}
			else //Вычисление дальности
			{
				pfc = abs(center_y - y_middle.at(i));
				z_laser_calc = h / (atan(pfc * m + b));
				z_calc = sqrt(h*h + z_laser_calc * z_laser_calc);
			}
			z_laser.push_back(z_laser_calc);
			z.push_back(z_calc);
		};
	}
	//Нахождение максимального z
	float maximum_z = *max_element(z.begin(), z.end());

	//Вычисление ширины
	float x_s = a_MNK * maximum_z + b_MNK;
	float x_left = -x_s / 2; //Левая граница
	float x_s_step = x_s / resolution_x; //Шаг прироста

	//Вычисление высоты
	float y_s = resolution_y * x_s_step;
	float y_left = -y_s / 2;
	float y_s_step = y_s / resolution_y;

	//Запись ширины в вектор
	for (int k = 0; k < resolution_x; k++)
	{
		x.push_back(x_left);
		x_left += x_s_step;
	}

	//Определение высоты по пиксельным координатам
	for (int i = 0; i < resolution_x; i++)
	{
		y_left = y_middle.at(i)*y_s_step;
		y.push_back(y_left);
	}
	y_middle.clear();

	//Удаление "разрывов"
	auto iter_x = x.cbegin();
	auto iter_y = y.cbegin();
	short l = 0;
	for (std::vector<double>::iterator it = z.begin(); it != z.end();)
	{
		if (*it == 0)
		{
			z.erase(it);
			x.erase(iter_x + l);
			y.erase(iter_y + l);
		}
		else
		{
			it++;
			l++;
		}
	}
}

// Сохранение профиля в СК камеры
void Scaner::Safe_dots(std::vector<cv::Point3d>&	cloud,
	std::ofstream&				out,
	const char					*name)
{
	for (int i = 0; i < x.size(); i++)
		cloud.push_back(cv::Point3d(x.at(i), y.at(i), z.at(i)));

	out.open(name, std::ios::app);
	for (std::vector<cv::Point3d>::iterator it = cloud.begin(); it != cloud.end(); it++)
	{
		out << it->x << ";" << it->y << ";" << it->z << std::endl;
	}
	cloud.clear();
	x.clear();
	y.clear();
	z.clear();
	out.close();
}

//Общая функция для сканирования
double Scaner::Scan_frame(cv::Mat image,		double center_y,
							int resolution_x,	int resolution_y)
{
	clock_t start = clock();
	Find_medium_laser(image, resolution_x);
	Calculate_coordinate(center_y, resolution_x, resolution_y);
	clock_t end = clock();
	return (double)(end - start) / 1000;
}

//Пересчет координат из СК камеры в глобальную СК
void Scaner::create_profiles(std::vector<cv::Point3d>& cloud,
							   double X,	 double Y,		double Z,
							   double phi,	 double theta,  double qsi)
{
	theta = -theta;
	qsi = -qsi;

	/// NEW VECTORS P_AUV AND P_GLOBAL
	std::vector<double> p_auv;
	p_auv.resize(3);
	std::vector<double> p_global;
	p_global.resize(3);

	/// ROTATION DELTA 1
	double Rot_d_1[3][3] = { {0,  -1,  0},
							 {1,  0,   0},
							 {0,  0,   1} };
	//OFFSET DELTA 1
	double Offset_d_1[3][1] = { {0.725},
								{0},
								{0.3} };

	/// ROTATION DELTA 2
	double Rot_d_2[3][3] = { {cos(qsi)*cos(theta), -sin(qsi)*cos(phi) + cos(qsi)*sin(theta)*sin(phi), sin(qsi)*sin(phi) + cos(qsi)*cos(phi)*sin(theta)},
						     {sin(qsi)*cos(theta),  cos(qsi)*cos(phi) + sin(phi)*sin(theta)*sin(qsi), -cos(qsi)*sin(phi) + sin(theta)*sin(qsi)*cos(phi)},
						     {-sin(theta),			cos(theta)*sin(phi),							  cos(theta)*cos(phi)} };

	/// OFFSET DELTA 2
	double Offset_d_2[3][1] = { {X},
								{Y},
								{Z} };
	for (int i = 0; i < z.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			p_auv.at(j) = (Rot_d_1[j][0] * x.at(i) + Rot_d_1[j][1] * y.at(i) + Rot_d_1[j][2] * z.at(i)) + Offset_d_1[j][0];
		}
		for (int k = 0; k < 3; k++)
		{
			p_global.at(k) = (Rot_d_2[k][0] * p_auv.at(0) + Rot_d_2[k][1] * p_auv.at(1) + Rot_d_2[k][2] * p_auv.at(2)) + Offset_d_2[k][0];
		}
		cloud.push_back(cv::Point3d(p_global.at(0), p_global.at(1), p_global.at(2)));
	}

	profiles.push_back({ cloud, X, Y, Z, phi, theta, qsi });
	cloud.clear();
	x.clear();
	y.clear();
	z.clear();
}

//Сохранение облака точек в файл .csv
void Scaner::safe_cloud(std::ofstream &out, const char *name)
{
	std::ofstream	outData;
	int count = 0;
	out.open(name, std::ios::app);

	for (std::vector<Profile>::iterator it = profiles.begin(); it != profiles.end(); it++)
	{
		//out << "number of frame:" << count << std::endl;
		for (int i = 0; i < it->points_SSK.size(); i++)
		{
			out << it->points_SSK.at(i).x << ";" << it->points_SSK.at(i).y << ";"
				<< it->points_SSK.at(i).z << std::endl;
		}
	}
	profiles.clear();
	out.close();
}

//Триангуляция
double Scaner::CalcDist(std::vector<double> p1, std::vector<double> p2) 
{
	return sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2) + pow(p1[2] - p2[2], 2));
}

void Scaner::triangul()
{
	int i = 1;
	double eps = 1.4;
	double maxDist = 10;
	int currentProfileLength = 0;
	int prevProfileLength = 0;

	for (i = 1; i < profiles.size(); i++)
	{
		std::vector<cv::Point3d> currentProfile = profiles.at(i).points_SSK;
		std::vector<cv::Point3d> prevProfile = profiles.at(i-1).points_SSK;
		currentProfileLength += prevProfile.size();

		int k = 0;
		int n = 0;
		int kN = k + 1;
		int nN = n + 1;

		int currentPointsCount = currentProfile.size();
		int prevPointsCount = prevProfile.size();

		cv::Point3d currentVm = { profiles.at(i).coord.X, profiles.at(i).coord.Y, profiles.at(i).coord.Z };
		cv::Point3d prevVm = { profiles.at(i-1).coord.X, profiles.at(i-1).coord.Y, profiles.at(i-1).coord.Z };

		//Следующий цикл нужен для объединения всех точек
		//профилей currentProfile и prevProfile в треугольники.
		//Здесь происходит расчет дистанции между соседними точками
		//этих профилей
		//и ее сравнение с maxDist.
		//Если заданные условия if выполняются, то в коллекцию индексов
		//точек треугольников <faces> производится запись индексов
		//обрабатываемых точек

		while (kN < currentPointsCount - 1 && nN < prevPointsCount - 1)
		{
			//std::cout << k << "   " << kN << "   " << n << "   " << nN << std::endl;
			if (abs(currentProfile.at(k).y - prevProfile.at(n).y) < eps)   // 1 потому что профиль по y
			{
				std::vector<double> sumVectorCurrent_kN = { currentProfile.at(kN).x + currentVm.x, currentProfile.at(kN).y + currentVm.y, currentProfile.at(kN).z + currentVm.z };
				std::vector<double> sumVectorCurrent_k = { currentProfile.at(k).x + currentVm.x, currentProfile.at(k).y + currentVm.y, currentProfile.at(k).z + currentVm.z };
				std::vector<double> sumVectorPrev_n = { prevProfile.at(n).x + prevVm.x, prevProfile.at(n).y + prevVm.y, prevProfile.at(n).z + prevVm.z };
				std::vector<double> sumVectorPrev_nN = { prevProfile.at(nN).x + prevVm.x, prevProfile.at(nN).y + prevVm.y, prevProfile.at(nN).z + prevVm.z };
				if (CalcDist(sumVectorCurrent_kN, sumVectorPrev_n) < maxDist)
				{
					if (CalcDist(sumVectorCurrent_kN, sumVectorCurrent_k) < maxDist && CalcDist(sumVectorCurrent_k, sumVectorPrev_n) < maxDist)
					{
						int a1 = currentProfileLength + k + 1;
						int a2 = currentProfileLength + kN + 1;
						int a3 = prevProfileLength + n + 1;
						cv::Point3i triangle = { a2, a1, a3 };
						surface.push_back(triangle);
					}

					if (CalcDist(sumVectorPrev_nN, sumVectorPrev_n) < maxDist && CalcDist(sumVectorPrev_nN, sumVectorCurrent_kN) < maxDist)
					{
						int a1 = prevProfileLength + n + 1;
						int a2 = currentProfileLength + kN + 1;
						int a3 = prevProfileLength + nN + 1;

						cv::Point3i triangle2 = { a2, a1, a3 };
						surface.push_back(triangle2);
					}
				}
				k += 1;
				n += 1;
				kN = k + 1;
				nN = n + 1;
			}
			else
			{
				if (currentProfile.at(k).y - prevProfile.at(n).y - eps < 0.0)
				{
					n += 1;
					nN = n + 1;
				}
				else
				{
					k += 1;
					kN = k + 1;
				}
			}
		}
		prevProfileLength = currentProfileLength;
	}
}


//Расчет нормалей (не работает)
void Scaner::calc_normals()
{
	std::vector<cv::Point3d> localVertices;
	for (std::vector<Profile>::iterator it = profiles.begin(); it != profiles.end(); it++)
	{
		for (int i = 0; i < it->points_SSK.size(); i++)
		{
			localVertices.push_back({ it->points_SSK.at(i).x , it->points_SSK.at(i).y, it->points_SSK.at(i).z });
		}
	}

	// Массив нормалей
	normals = std::vector<cv::Point3d>(localVertices.size());

	// Инициализация нулями
	for (int i = 0; i < normals.size(); i++)
	{
normals[i] = cv::Point3d();
	}

	// Перебор всех треугольников
	for (int i = 0; i < localVertices.size(); i += 3)
	{
		float nx = std::numeric_limits<float>::quiet_NaN(), ny = std::numeric_limits<float>::quiet_NaN(), nz = std::numeric_limits<float>::quiet_NaN();
		int counter = 0;

		cv::Point3d a, b, c;

		// Возможные комбинации следования точек в треугольнике
		int ind[18]{
				i, i + 1, i + 2,
				i, i + 2, i + 1,
				i + 1, i, i + 2,
				i + 1, i + 2, i,
				i + 2, i, i + 1,
				i + 2, i + 1, i
		};

		int indOffset = 0;

		// Расчет нормали если она не была расчитана на предыдущей 
		// итерации
		while (IsNan(nx, ny, nz) && counter < 6)
		{
			a = localVertices[surface[ind[indOffset + 0]].x];
			b = localVertices[surface[ind[indOffset + 1]].y];
			c = localVertices[surface[ind[indOffset + 2]].z];

			auto A = a - b;
			auto B = b - c;

			auto Nx = A.y * B.z - A.z * B.y;
			auto Ny = A.z * B.x - A.x * B.z;
			auto Nz = A.x * B.y - A.y * B.x;

			auto len = sqrt(Nx * Nx + Ny * Ny + Nz * Nz);

			nx = Nx / len;
			ny = Ny / len;
			nz = Nz / len;

			//Иначе indOffset станет 18 и будет выход за пределы массива
			if (counter < 5)
			{
				indOffset += 3;
			}
			counter += 1;
		}

		auto fn = cv::Point3d(nx, ny, nz);

		normals[surface[ind[indOffset]].x] = fn;
		normals[surface[ind[indOffset + 1]].y] = fn;
		normals[surface[ind[indOffset + 2]].z] = fn;
	}
}

//Поиск деформаций (не работает)
void Scaner::find_deformation(const char *original, const char *deformation, double difference,
							    std::ofstream &out, const char *name)
{
	std::string line_1;
	std::string line_2;
	double x_orig = 0, y_orig = 0, z_orig = 0;
	double x_deform = 0, y_deform = 0, z_deform = 0;
	std::vector<cv::Point3d> orig;
	std::vector<cv::Point3d> deform;
	std::vector<cv::Point3d> find_deform;
	double diff = 0;
	char t = ';';
	std::ifstream fileORIG(original);
	std::ifstream fileDEFORM(deformation);

	//Считываение точек из оригинала (.CSV)
	while (getline(fileORIG, line_1))
	{
		fileORIG >> x_orig >> t >> y_orig >> t >> z_orig;
		orig.push_back(cv::Point3d(x_orig, y_orig, z_orig));
	}

	//Считывание точек из файла с деформацией (.CSV)
	while (getline(fileDEFORM, line_2))
	{
		fileDEFORM >> x_deform >> t >> y_deform >> t >> z_deform;
		deform.push_back(cv::Point3d(x_deform, y_deform, z_deform));
	}

	std::cout << "Size orig: " << orig.size() << std::endl;
	std::cout << "Size deform: " << deform.size() << std::endl;

	fileORIG.close();
	fileDEFORM.close();
	out.open(name, std::ios::app);
	//Поиск деформации
	for (int i = 0; i < deform.size(); i++)
	{
		//if (orig.at(i).y == deform.at(i).y)
		//{
		diff = abs(orig.at(i).z - deform.at(i).z);
		if (diff >= difference)
		{
			//std::cout << "Deform point: " << deform.at(i) << " difference: " << diff << std::endl;
			out << deform.at(i).x << ";" << deform.at(i).y << ";" << deform.at(i).z << std::endl;
		}
		//}
	}
}

void Scaner::safe_triangul(std::ofstream &out, const char *name)
{
	std::ofstream	outData;
	int				count = 1;

	out.open(name, std::ios::app);
	out << "o Object.1" << std::endl;
	for (std::vector<Profile>::iterator it = profiles.begin(); it != profiles.end(); it++)
	{
		for (int i = 0; i < it->points_SSK.size(); i++)
		{
			out << "v " << it->points_SSK.at(i).x << " " << it->points_SSK.at(i).y << " "
				<< it->points_SSK.at(i).z << std::endl;
		}
		count += 1;
	}
	for (int c = 0; c < surface.size(); c++)
	{
		out << "f " << surface.at(c).x << " " << surface.at(c).y << " " << surface.at(c).z << std::endl;
	}
	out.close();
}