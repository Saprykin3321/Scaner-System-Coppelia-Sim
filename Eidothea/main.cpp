#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <iostream>
#include <process.h>
#include <time.h>
#include <Windows.h>

#include "Scaner.h"

#define PI 3.14159265359

/// ��������� ��� ���������� ������
double start = 0.15;  // ��������� S, � ������� ������ ���������
float step = 0.05;  // ��� ���������
/// ���� � ��������� ������, ������� ����� � ����� ���������
char name_folder_for_laser_calibration[] = "Pictures for calibration laser/*png"; //��� ���������� ������
char path_to_original_file[] = "original.csv"; //�������� ���� ��� ����������
char path_to_deformation_file[] = "deformation.csv"; //�������� ���� � �����������

//��� ������� ����� ����� �������. ������ ��� ����� �����, ��� ����� �� ��������, ���� ���������
void clear_file(std::ofstream &out)
{
	out.open("test_2021.csv", std::ios::out);
	out.close();
}
extern "C" {  //��� ���������
#include "extApi.h"
}

///��� ������� ����� ������� ���� � ����� � ������������� �� � ������ ��� ������������ ������ (������, ������ �����)
char buffer[80];
char* current_time()
{
	time_t rawtime;
	struct tm * timeinfo;
	char *buff2 = buffer;
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buff2, 80, "%d%m%y%H%M%S", timeinfo);
	return (buff2);
}

int main()
{
	//��������� ���������
	std::string command;
	int clientID;
	int simRes;

	simxInt AUV;
	simxInt cuboid0;
	simxInt Laser;

	simxInt camera;
	simxInt resolution[2];
	simxUChar * image = 0;
	double center_y;

	float position[3]; //���������� ��� �������� ������� �������
	float orientation[3]; //���������� ��� �������� ������� ����������
	char filename[100];
	std::string s_name_file;
	short count_of_frame = 1; //������� ������
	//�������� ������� ������ find. 0.071 - ���������� ����� ������� � �������
	Scaner scan(0.071);

	//���� ��� ������ ������ �����
	std::vector<cv::Point3d> cloud_point_2020;

	while (true) //��������� ���������� ��������� ������� ���������. ��������� ������� ��������� � ���������� � ����� ���������
	{
		//���������� ������� �� ������������ � ������ ���� string
		std::cout << "Enter the command: ";
		std::getline(std::cin, command);
		std::istringstream iss(command);
		std::vector<std::string> results((std::istream_iterator<std::string>(iss)),
			std::istream_iterator<std::string>());


		//����� ���������
		if ((results.at(0) == "Start") && (results.size() == 1))
		{
			clientID = simxStart("127.0.0.1", 19997, true, true, 5000, 5);
			simRes = simxStartSimulation(clientID, simx_opmode_oneshot);
			printf("simRes is %d", simRes);
			if (clientID != -1) { printf("Start success\n"); }
			else { printf("Error ClientID"); }

			//��� ��������� �������� handle �������� ���������. ������� ������������� ��, ���� � ��� ������ ��������
			simxGetObjectHandle(clientID, "Vision_sensor", &camera, simx_opmode_blocking);
			simxGetObjectHandle(clientID, "AUV", &AUV, simx_opmode_blocking);
			simxGetObjectHandle(clientID, "Cuboid0", &cuboid0, simx_opmode_blocking);
			std::cout << "VISION_SENSOR_HANDLE: " << camera << std::endl;
			std::cout << "AUV_HANDLE: " << AUV << std::endl;
		}


		// ������������ ���������
		else if ((results.at(0) == "Pause") && (results.size() == 1))
		{
			std::cout << "Pause simulation..." << std::endl;
			simRes = simxPauseSimulation(clientID, simx_opmode_oneshot);
			printf("simRes is %d", simRes);
		}


		// ��������� ��������� � ���������� ���������
		else if ((results.at(0) == "Stop") && (results.size() == 1))
		{
			std::cout << "Stop simulation" << std::endl;
			simRes = simxStopSimulation(clientID, simx_opmode_oneshot);
			simxFinish(clientID);
			printf("simRes is %d", simRes);
			std::cout << "Programm is stopped" << std::endl;
			break;
		}


		///��������� ������ ����� � ������
		else if ((results.at(0) == "Frame") && (results.size() == 1))
		{
			simxGetVisionSensorImage(clientID, camera, resolution, &image, 0, simx_opmode_oneshot_wait);
			cv::Mat channel(resolution[0], resolution[1], CV_8UC3, image);
			cv::flip(channel, channel, 0);
			cv::cvtColor(channel, channel, cv::COLOR_RGB2BGR);
			s_name_file = name_folder_for_laser_calibration; // ���� ����������� � ����� ��� ���������� ������
			s_name_file += current_time();
			s_name_file += ".png";
			const char *sc_name_file_one_frame = s_name_file.c_str();
			sprintf_s(filename, sc_name_file_one_frame);
			cv::imwrite(filename, channel);
			std::cout << "Frame was get and save in " << name_folder_for_laser_calibration << std::endl;
		}


		/// ���������� ������ � ������
		else if ((results.at(0) == "Calibration") && (results.size() == 1))
		{
			simxGetVisionSensorImage(clientID, camera, resolution, &image, 0, simx_opmode_oneshot_wait);

			cv::Mat channel(resolution[0], resolution[1], CV_8UC3, image);
			cv::flip(channel, channel, 0);
			cv::cvtColor(channel, channel, cv::COLOR_RGB2BGR);

			//���������� ������������� ����������� � ����� Calibration_file
			s_name_file = "Results/HSV_mask/";
			s_name_file += current_time();
			s_name_file += "_original.png";
			const char *sc_name_file_orig = s_name_file.c_str();
			sprintf_s(filename, sc_name_file_orig);
			cv::imwrite(filename, channel);

			//��������� ��������� HSV ��� ������� �����
			scan.Find_HSV_Parameters(channel);
			std::cout << "HSV_low_mask: " << scan.get_min_h() << " " << scan.get_min_s() << " " << scan.get_min_v() << std::endl;
			std::cout << "HSV_high_mask: " << scan.get_max_h() << " " << scan.get_max_s() << " " << scan.get_max_v() << std::endl;

			//���������� ����� ����������� � ����� Calibration_file
			cv::Mat			hsv;
			cv::Mat			mask;
			cv::Scalar		hsv_l(scan.get_min_h(), scan.get_min_s(), scan.get_min_v());
			cv::Scalar		hsv_h(scan.get_max_h(), scan.get_max_s(), scan.get_max_v());
			cv::cvtColor(channel, hsv, cv::COLOR_BGR2HSV);
			cv::inRange(hsv, hsv_l, hsv_h, mask);

			s_name_file = "Results/HSV_mask/";
			s_name_file += current_time();
			s_name_file += "_hsv.png";
			const char *sc_name_file_hsv = s_name_file.c_str();
			sprintf_s(filename, sc_name_file_hsv);
			cv::imwrite(filename, mask);

			//���������� ������ (���������� ������������� ��� ������� ��������� � ������ �����)
			center_y = channel.rows / 2;
			scan.calibration_laser(center_y, &start, step, name_folder_for_laser_calibration, channel.cols);
			std::cout << "m: " << scan.get_m() << ", b: " << scan.get_b() << std::endl;
			std::cout << "a_MNK: " << scan.get_a_MNK() << ", b_MNK: " << scan.get_b_MNK() << std::endl;
		}

		//������� ��� ������������ ������ �����
		else if ((results.at(0) == "Scan") && (results.at(1) == "one") && (results.at(2) == "frame"))
		{
			simxGetObjectPosition(clientID, AUV, -1, position, simx_opmode_oneshot_wait);
			simxGetObjectOrientation(clientID, AUV, -1, orientation, simx_opmode_oneshot_wait);
			simxGetVisionSensorImage(clientID, camera, resolution, &image, 0, simx_opmode_oneshot_wait);

			//����� ������� � ������� ������ � ������� � ���������� ���
			//std::cout << "X: " << position[0] << "\tY: " << position[1] << "\tZ: " << position[2]
			//		  << "\tAlp: " << (orientation[0]*180)/PI << "\tBet: " << (orientation[1]*180)/PI
			//		  << "\tYa: "  << (orientation[2]*180)/PI  << std::endl;

			//�������� ���� � ������ � ��������� ��� � ����� Results
			cv::Mat channel(resolution[0], resolution[1], CV_8UC3, image);
			cv::flip(channel, channel, 0);
			cv::cvtColor(channel, channel, cv::COLOR_RGB2BGR);
			s_name_file = "Results/Scan_one_frame/";
			s_name_file += current_time();
			s_name_file += ".png";
			const char *sc_name_file_one_frame = s_name_file.c_str();
			sprintf_s(filename, sc_name_file_one_frame);
			cv::imwrite(filename, channel);

			//C ����� �� ������ ����������� ������ ����� ����� ������������ 
			s_name_file = "Results/Scan_one_frame/";
			s_name_file += current_time();
			s_name_file += "_cloud.csv";
			const char *name_file_cloud = s_name_file.c_str();
			std::ofstream outData(name_file_cloud);
			clear_file(outData);

			//������� ������������
			scan.Scan_frame(channel, center_y, channel.cols, channel.rows);
			scan.create_profiles(cloud_point_2020, position[0],    position[1],    position[2],
												   orientation[0], orientation[1], orientation[2]);
			scan.safe_cloud(outData, name_file_cloud);

			//� ���������� ������ ������� � ����� Results ����������� ���� � ������ � ������ ����� � ���������� ������� ���������
		}

		//����������� ������������ � ��������� �������� ���
		else if ((results.at(0) == "Start") && (results.at(1) == "scan"))
		{
			float vel = std::stod(results.at(2)); //�������� �������� ��������
			simxSetFloatSignal(clientID, "velocity", vel, simx_opmode_oneshot); //���������� � ���������

			while (true)
			{
				///����������� �������������� ������� ������������, �� ��� ���������� ������ � � ������� ����������� �� �����
				simxGetObjectPosition(clientID, AUV, -1, position, simx_opmode_oneshot_wait);
				simxGetObjectOrientation(clientID, AUV, -1, orientation, simx_opmode_oneshot_wait);
				simxGetVisionSensorImage(clientID, camera, resolution, &image, 0, simx_opmode_oneshot_wait);

				/*std::cout << "X: " << position[0] << "\tY: " << position[1] << "\tZ: " << position[2]
						  << "\tAlp: " << (orientation[0]*180)/PI << "\tBet: " << (orientation[1]*180)/PI
						  << "\tYa: "  << (orientation[2]*180)/PI  << std::endl;*/

				cv::Mat channel(resolution[0], resolution[1], CV_8UC3, image);
				cv::flip(channel, channel, 0);
				cv::cvtColor(channel, channel, cv::COLOR_RGB2BGR);
				cv::imshow("Eidothea scan", channel);

				scan.Scan_frame(channel, center_y, channel.cols, channel.rows);
				scan.create_profiles(cloud_point_2020, position[0],    position[1],    position[2],
													   orientation[0], orientation[1], orientation[2]);

				count_of_frame++;

				//���� ������ ������� q �� ���� � ����������� ������, ���������� ������������. 
				//�� �������� ���� ���������� � ��������� �������, ����� � ����� ������, ��������� � ������
				//����� ���������� ������������ �������� ��� ���������� 0, � ��������� �������� ��������� � ����� .csv � ����� Results
				if (cv::waitKey(10) == 'q')
				{
					//����������
					cv::destroyWindow("Eidothea scan");

					s_name_file = "Results/Continuous_scan/";
					s_name_file += current_time();
					s_name_file += "_cloud.csv";
					const char *name_file_cloud = s_name_file.c_str();
					std::ofstream outData(name_file_cloud);
					clear_file(outData);

					simxSetFloatSignal(clientID, "velocity", float(0.00), simx_opmode_oneshot);
					scan.safe_cloud(outData, name_file_cloud);

					//������������� ���������� ������ ����� ����� ���������������
					//find.triangul();
					//char name_file_2[] = "test_2021_vrep_11.obj";
					//std::ofstream outData_2(name_file_2);
					//clear_file(outData_2);
					//find.safe(outData_2, name_file_2);

					std::cout << "Total scan frame: " << count_of_frame << std::endl;
					count_of_frame = 1;
					std::cout << "Exit..." << std::endl;
					Sleep(100);
					break;
				}
			}
		}

		//����� ������������� ���� � �������������� ������� ����������. ��� ������� ��� ������ ���������� �� ������ ������� �� z.
		//�������� �����������
		else if ((results.at(0) == "Find") && (results.at(1) == "deformation"))
		{
			s_name_file = "deformation_point.csv";
			const char *name_file_cloud = s_name_file.c_str();
			std::ofstream outData(name_file_cloud);
			clear_file(outData);
			scan.find_deformation(path_to_original_file, path_to_deformation_file, 0.01, outData, name_file_cloud);
		}
	}
	return 0;
}