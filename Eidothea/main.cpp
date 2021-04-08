#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>  
#include <process.h>
#include <Windows.h>
using namespace std;
#include <time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Profiles.h"
#include "Camera.h"

char path_to_difference_pix[] = "calibration_distance/*png";

void clear_file(std::ofstream &out) //Костыль
{
	out.open("test_2021.csv", std::ios::out);
	out.close();
}

extern "C" {
#include "extApi.h"
}

#define PI 3.14159265359

int main()
{
	int clientID = simxStart("127.0.0.1", 19997, true, true, 5000, 5);
	int simRes = simxStartSimulation(clientID, simx_opmode_oneshot);
	printf("simRes is %d", simRes);

	if (clientID != -1)
	{
		printf("success\n");
		//simxSynchronous(clientID, true);
	}

	else
	{
		printf("error");
	}

	simxInt camera;
	simxGetObjectHandle(clientID, "Vision_sensor", &camera, simx_opmode_blocking);
	simxInt resolution[2];
	simxUChar * image = 0;
	std::cout << "VISION_SENSOR_HANDLE: " << camera << std::endl;

	simxInt AUV;
	simxGetObjectHandle(clientID, "AUV", &AUV, simx_opmode_blocking);
	std::cout << "AUV_HANDLE: " << AUV << std::endl;
	float position[3];
	float orientation[3];

	Profiles find(0.071);
	int sTime = extApi_getTimeInMs();
	cv::namedWindow("opencv test", cv::WINDOW_AUTOSIZE);

	cv::Mat image_vrep = cv::imread("D:/Test_vrep/One/1.png");
	double center_y = image_vrep.rows / 2;
	double start = 0.25;
	float step = 0.05;
	find.calculate_m_b(center_y, &start, step, path_to_difference_pix);

	vector<cv::Point3d>cloud_point_2020;
	char name_file[] = "test_2021_vrep_4.csv";
	std::ofstream outData(name_file);
	clear_file(outData);

	while (true)
	{
		simxGetObjectPosition(clientID, AUV, -1, position, simx_opmode_oneshot_wait);
		simxGetObjectOrientation(clientID, AUV, -1, orientation, simx_opmode_oneshot_wait);

		int retval = simxGetVisionSensorImage(clientID, camera, resolution, &image, 0, simx_opmode_oneshot_wait);
		//std::cout << "retval: " << retval << std::endl;
		if (retval != simx_return_ok) {
			continue;
		}
		cv::Mat channel(resolution[0], resolution[1], CV_8UC3, image);
		cv::flip(channel, channel, 0);
		cv::cvtColor(channel, channel, cv::COLOR_RGB2BGR);
		cv::resize(channel, channel, cv::Size(1280, 720));
		cv::imshow("opencv test", channel);

		find.Scan_frame(channel, center_y);
		find.create_profiles(cloud_point_2020, position[0], position[1], position[2],
			0, 0, 0);

		if (cv::waitKey(10) == 'q')
		{
			std::cout << "Exit..." << std::endl;
			Sleep(100);
			break;
		}

		//std::cout << "X: " << position[0] << " Y: " << position[1] << " Z: " << position[2]
		//	      << " Alp: " << (orientation[0]*180)/PI  << " Bet: " << (orientation[1]*180)/PI 
		//	      << " Ya: "  << (orientation[2]*180)/PI  << std::endl;

		//printf("Time: %d\n", extApi_getTimeInMs()); // Mouse position x is actualized when the cursor is over V-REP's window
	}

	find.safe_cloud(outData, name_file);

	find.triangul();
	char name_file_2[] = "test_2021_vrep_4.obj";
	std::ofstream outData_2(name_file_2);
	clear_file(outData_2);
	find.safe(outData_2, name_file_2);

	simxStopSimulation(clientID, simx_opmode_oneshot);
	simxFinish(clientID);
	return 0;
}


//#include <stdio.h>
//#include <stdlib.h>
//#include <vector>
//#include <iostream>  
//#include <process.h>
//#include <Windows.h>
//using namespace std;
//#include <time.h>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//
//extern "C" {
//#include "extApi.h"
//}
//
//int main()
//{
//	int clientID = simxStart("127.0.0.1", 19997, true, true, 5000, 5);
//
//
//	if (clientID != -1)
//	{
//		printf("success");
//	}
//	else
//	{
//		printf("error");
//	}
//	simxStartSimulation(clientID, simx_opmode_oneshot);
//
//	//simxInt leftmotor;
//	//simxInt rightmotor;
//	//simxGetObjectHandle(clientID, "Leftmotor", &leftmotor,simx_opmode_blocking);
//	//simxGetObjectHandle(clientID, "Rightmotor", &rightmotor, simx_opmode_blocking);
//	simxInt camera;
//	simxGetObjectHandle(clientID, "Vision_sensor", &camera, simx_opmode_blocking);
//	simxInt resolution[2];
//	simxUChar * image = 0;
//	//cv::namedWindow("opencv test", cv::WINDOW_AUTOSIZE);
//
//
//	//if (clientID != -1)
//	//{
//	//	cout << " Connection status to VREP: SUCCESS" << endl;
//	//	simxInt syncho = simxSynchronous(clientID, 1);
//	//	int start = simxStartSimulation(clientID, simx_opmode_oneshot_wait);
//	//}
//
//	for (int time = 0; time < 10000; time++) {
//		//Sleep(1000);
//		//simxSetJointTargetVelocity(clientID, leftmotor, -time*0.02, simx_opmode_oneshot);
//		//simxSetJointTargetVelocity(clientID, rightmotor, -time*0.03, simx_opmode_oneshot);
//
//		int retval = simxGetVisionSensorImage(clientID, camera, resolution, &image, 0, simx_opmode_oneshot);
//		if (retval != simx_return_ok) {
//			continue;
//		}
//		cv::Mat channel(resolution[0], resolution[1], CV_8UC3, image);
//		//The image data read back is flipped vertically, the problem should be that the direction of the vertical coordinate axis of cvMat and v-rep is opposite, and the flip is normal
//		cv::flip(channel, channel, 0);
//		//The rgb channel is distributed when the image data read back, and cvMat defaults to bgr
//		cv::cvtColor(channel, channel, cv::COLOR_RGB2BGR);
//		cv::resize(channel, channel, cv::Size(1280, 720));
//		if (time == 4000)
//		{
//			cv::imwrite("1.png", channel);
//		}
//		//count_frame += 1;
//		std::cout << time << std::endl;
//		//cv::imshow("opencv test", channel);
//		//cv::waitKey(10);
//	}
//	simxStopSimulation(clientID, simx_opmode_oneshot);
//	simxFinish(clientID);
//	return 0;
//}

//#include <Windows.h>
//#include <iostream>
//#include <stdio.h>
//#include <stdlib.h>
//
//
//extern "C" {
//#include "extApi.h"
//}
//
//using namespace std;
//#define PI 3.14
//int main()
//{
//	bool VERBOSE = true;
//	int clientID = 0;
//	int leftmotorHandle = 0;
//	int rightmotorHandle = 0;
//
//	int lbrJoint1 = 0;
//	int lbrJoint2 = 0;
//	int lbrJoint3 = 0;
//	int lbrJoint4 = 0;
//	int lbrJoint5 = 0;
//	int lbrJoint6 = 0;
//	int lbrJoint7 = 0;
//
//	int counter = 0;
//
//	//! Todo Naresh: check to run this in parallel with real robot driver. May need to integrate my planner
//	bool WORK = true;
//	simxFinish(-1);                                                     //! Close any previously unfinished business
//	clientID = simxStart((simxChar*)"127.0.0.1", 19000, true, true, 5000, 5);  //!< Main connection to V-REP
//	Sleep(1);
//	if (clientID != -1)
//	{
//		cout << " Connection status to VREP: SUCCESS" << endl;
//		simxInt syncho = simxSynchronous(clientID, 1);
//		int start = simxStartSimulation(clientID, simx_opmode_oneshot_wait);
//		int TEST1 = simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", &leftmotorHandle, simx_opmode_oneshot_wait);
//		int TEST2 = simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor", &rightmotorHandle, simx_opmode_oneshot_wait);
//
//		simxGetObjectHandle(clientID, "LBR_iiwa_14_R820_joint1", &lbrJoint1, simx_opmode_oneshot_wait);
//		simxGetObjectHandle(clientID, "LBR_iiwa_14_R820_joint2", &lbrJoint2, simx_opmode_oneshot_wait);
//		simxGetObjectHandle(clientID, "LBR_iiwa_14_R820_joint3", &lbrJoint3, simx_opmode_oneshot_wait);
//		simxGetObjectHandle(clientID, "LBR_iiwa_14_R820_joint4", &lbrJoint4, simx_opmode_oneshot_wait);
//		simxGetObjectHandle(clientID, "LBR_iiwa_14_R820_joint5", &lbrJoint5, simx_opmode_oneshot_wait);
//		simxGetObjectHandle(clientID, "LBR_iiwa_14_R820_joint6", &lbrJoint6, simx_opmode_oneshot_wait);
//		simxGetObjectHandle(clientID, "LBR_iiwa_14_R820_joint7", &lbrJoint7, simx_opmode_oneshot_wait);
//
//		if (VERBOSE)
//		{
//			cout << "Computed object handle: " << TEST1 << "  " << leftmotorHandle << endl;
//			cout << "Computed object handle: " << TEST2 << "  " << rightmotorHandle << endl;
//		}
//
//		//        simxPauseCommunication(clientID,true);
//		simxSetJointTargetPosition(clientID, lbrJoint1, 0.0, simx_opmode_oneshot_wait);
//		simxSetJointTargetPosition(clientID, lbrJoint2, 0.0, simx_opmode_oneshot_wait);
//		simxSetJointTargetPosition(clientID, lbrJoint3, 0.0, simx_opmode_oneshot_wait);
//		simxSetJointTargetPosition(clientID, lbrJoint4, 0.0, simx_opmode_oneshot_wait);
//		simxSetJointTargetPosition(clientID, lbrJoint5, 0.0, simx_opmode_oneshot_wait);
//		simxSetJointTargetPosition(clientID, lbrJoint6, 0.0, simx_opmode_oneshot_wait);
//		simxSetJointTargetPosition(clientID, lbrJoint7, 0.0, simx_opmode_oneshot_wait);
//		//        simxPauseCommunication(clientID,false);
//
//		cout << "At Second Block..." << endl;
//
//		//        simxPauseCommunication(clientID,1);
//		simxSetJointTargetPosition(clientID, lbrJoint1, 90.0* (PI / 180), simx_opmode_oneshot_wait);
//		simxSetJointTargetPosition(clientID, lbrJoint2, 90.0* (PI / 180), simx_opmode_oneshot_wait);
//		simxSetJointTargetPosition(clientID, lbrJoint3, 170.0* (PI / 180), simx_opmode_oneshot_wait);
//		simxSetJointTargetPosition(clientID, lbrJoint4, -90.0* (PI / 180), simx_opmode_oneshot_wait);
//		simxSetJointTargetPosition(clientID, lbrJoint5, 90.0* (PI / 180), simx_opmode_oneshot_wait);
//		simxSetJointTargetPosition(clientID, lbrJoint6, 90.0* (PI / 180), simx_opmode_oneshot_wait);
//		simxSetJointTargetPosition(clientID, lbrJoint7, 0.0* (PI / 180), simx_opmode_oneshot_wait);
//		//        simxPauseCommunication(clientID,0);
//
//		//        float joint2 = 1;
//		//        //simxSetJointTargetVelocity(clientID, lbrJoint2, 0.1, simx_opmode_oneshot_wait);
//		//        while (simxGetConnectionId(clientID)!=-1  && WORK)              ///**<  while we are connected to the server.. */
//		//        {
//		////            simxSetJointTargetVelocity(clientID, leftmotorHandle, 0.2, simx_opmode_oneshot_wait);
//		////            simxSetJointTargetVelocity(clientID, rightmotorHandle, 0.2, simx_opmode_oneshot_wait);
//		//                TEST3 = simxSetJointTargetVelocity(clientID, lbrJoint2, -0.1, simx_opmode_oneshot);
//		////            TEST3 = simxSetJointTargetPosition(clientID, lbrJoint2, joint2 * (PI/180), simx_opmode_oneshot);
//		//
//		//            if(counter>1000)
//		//            {
//		//                simxSetJointTargetVelocity(clientID, leftmotorHandle, 0.0, simx_opmode_oneshot_wait);
//		//                simxSetJointTargetVelocity(clientID, rightmotorHandle, 0.0, simx_opmode_oneshot_wait);
//		//                simxSetJointTargetVelocity(clientID, lbrJoint2, 0, simx_opmode_oneshot_wait);
//		//                break;
//		//            }
//		//            cout<<counter<< "  "<< TEST3<<endl;
//		//            counter++;
//		//            joint2 = joint2 + 0.08;
//		//        }
//	}
//	else
//	{
//		cout << " Connection status to VREP: FAILED" << endl;
//	}
//	simxFinish(clientID);
//	return clientID;
//}