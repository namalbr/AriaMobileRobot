//#include "Aria.h"
//#include "ariaUtil.h"
//#include <math.h>
//#include <time.h> 
//#include <Windows.h>
//#include <stdio.h>
//#include <fstream>
//#include <iostream>
//#include <stdlib.h> 
//using namespace std;
//
//int realX = 0.0;
//int realY = 0.0;
//int realZ = 0.0;
//int realTh = 0.0;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//using namespace std;
//double sn[8];
//double leftVel;
//double RightVel;
//double nominal = 600;
//double avoid_distance = 1000;
//double error;
//double leftside[3], rightside[3];
//double leftang[3], rightang[3];
//
////===================PID======
//
//double kp = 3, ki = .00001, kd = 5;
//double w[] = { 1,0,0 };
//
//double kpLD = 1, kiLD = .0001, kdLD = 4.5;
//double kpRD = 1, kiRD = .0001, kdRD = 4.5;
////======================== FUzzy ===============
//
//double sn0_dis = 600;
//double sn1_dis = sn0_dis / (cos((40)*(M_PI / 180)));
//double sn0_max = 1200;
//double sn1_max = sn0_max / (cos((40)*(M_PI / 180)));
//
//double sn_low[8], sn_high[8];
//
//float fw[] = {200,100,-100,-200};  // fuzzy weights 
//double f_avoid = 1200;
//
////==============================================
//bool handleDebugMessage(ArRobotPacket *pkt)
//{
//	if (pkt->getID() != ArCommands::MARCDEBUG) return false;
//	char msg[256];
//	pkt->bufToStr(msg, sizeof(msg));
//	msg[255] = 0;
//	ArLog::log(ArLog::Terse, "Controller Firmware Debug: %s", msg);
//	return true;
//}
//
//bool handleSimStatPacket(ArRobotPacket* pkt)
//{
//	if (pkt->getID() != 0x62) return false; // SIMSTAT has id 0x62
//
//											//printf("SIMSTAT pkt received: ");
//	char a = pkt->bufToByte();  // unused byte
//	char b = pkt->bufToByte();  // unused byte
//	ArTypes::UByte4 flags = pkt->bufToUByte4();
//	//printf("\tFlags=0x%x\n", flags);
//	int simint = pkt->bufToUByte2();
//	int realint = pkt->bufToUByte2();
//	int lastint = pkt->bufToUByte2();
//	//printf("\tSimInterval=%d, RealInterval=%d, LastInterval=%d.\n", simint, realint, lastint);
//	realX = pkt->bufToByte4();
//	realY = pkt->bufToByte4();
//	realZ = pkt->bufToByte4();
//	realTh = pkt->bufToByte4();
//	//printf("\tTrue Pose = (%d, %d, %d, %d)\n", realX, realY, realZ, realTh);
//	if (flags & ArUtil::BIT1)
//	{
//		double lat = pkt->bufToByte4() / 10e6;
//		double lon = pkt->bufToByte4() / 10e6;
//		double alt = pkt->bufToByte4() / 100;
//		double qual = pkt->bufToByte() / 100;
//		//printf("\tLatitude = %f deg., Longitude = %f deg., Altitude = %f m, Quality = %f%%\n", lat, lon, alt, qual);
//	}
//	else
//	{
//		//puts("No geoposition.");
//	}
//	return true;
//}
//void sonarRead(double sn[8]) {
//
//	// cos Rule has been Utilized to Get the Missing Side of the Triangle formed with the Left or Right most and the 3rd Sonar from the Left or right most
//	rightside[0] = sqrt(pow(sn[6], 2) + pow(sn[7], 2) - (2 * sn[6] * sn[7] * cos(40 * (M_PI / 180)))); //calculation
//	rightside[1] = sqrt(pow(sn[5], 2) + pow(sn[7], 2) - (2 * sn[5] * sn[7] * cos(60 * (M_PI / 180)))); //calculation
//	rightside[2] = sqrt(pow(sn[4], 2) + pow(sn[7], 2) - (2 * sn[4] * sn[7] * cos(80 * (M_PI / 180)))); //calculation
//																									   //-----------------Left Sides Calculation --------------------------------------
//	leftside[0] = sqrt(pow(sn[1], 2) + pow(sn[0], 2) - (2 * sn[1] * sn[0] * cos(40 * (M_PI / 180)))); //calculation
//	leftside[1] = sqrt(pow(sn[2], 2) + pow(sn[0], 2) - (2 * sn[2] * sn[0] * cos(60 * (M_PI / 180)))); //calculation
//	leftside[2] = sqrt(pow(sn[3], 2) + pow(sn[0], 2) - (2 * sn[3] * sn[0] * cos(80 * (M_PI / 180)))); //calculation
//																									  //-----------------Right Angles Calculation --------------------------------------
//	rightang[0] = (pow(rightside[0], 2) + pow(sn[7], 2) - pow(sn[6], 2)) / (2 * rightside[0] * sn[7]); //calculation
//	rightang[1] = (pow(rightside[1], 2) + pow(sn[7], 2) - pow(sn[5], 2)) / (2 * rightside[1] * sn[7]); //calculation
//	rightang[2] = (pow(rightside[2], 2) + pow(sn[7], 2) - pow(sn[4], 2)) / (2 * rightside[2] * sn[7]); //calculation
//																									   //-----------------Left Sides Calculation --------------------------------------
//	leftang[0] = (pow(leftside[0], 2) + pow(sn[0], 2) - pow(sn[1], 2)) / (2 * leftside[0] * sn[0]);//calculation
//	leftang[1] = (pow(leftside[1], 2) + pow(sn[0], 2) - pow(sn[2], 2)) / (2 * leftside[1] * sn[0]);//calculation
//	leftang[2] = (pow(leftside[2], 2) + pow(sn[0], 2) - pow(sn[3], 2)) / (2 * leftside[2] * sn[0]);//calculation
//																								   //-----------------Left Angles --------------------------------------
//	leftang[0] = acos(leftang[0])*(180 / M_PI);
//	leftang[1] = acos(leftang[1])*(180 / M_PI);
//	leftang[2] = acos(leftang[2])*(180 / M_PI);
//	//-----------------Left Angles --------------------------------------
//	rightang[0] = acos(rightang[0])*(180 / M_PI);
//	rightang[1] = acos(rightang[1])*(180 / M_PI);
//	rightang[2] = acos(rightang[2])*(180 / M_PI);
//	//cout << sonar[7] << endl;
//}
//
//double B_low(double sensor, int num)
//{
//	double y;
//	if (num == 0 || num == 7)
//	{
//		y = 1 - (sensor / sn0_max);
//	}
//	else if(num ==1 || num ==6 )
//		y = 1-(sensor / sn1_max);
//
//	return y;
//}
//
//double B_high(double sensor , int num)
//{
//	double y;
//	if (num == 0 || num == 7)
//	{
//		y =  (sensor / sn0_max);
//	}
//	else if (num == 1 || num == 6)
//		y = (sensor / sn1_max);
//
//	return y;
//}
//
//double minima(double x, double y)
//{
//	if (x < y) return x;
//	else return y;
//}
//
//double output()
//{
//	double zo = minima(B_low(sn[0], 0), B_high(sn[1], 1));
//	double oo = minima(B_high(sn[0], 0), B_high(sn[1], 1));
//	double zz = minima(B_low(sn[0], 0), B_low(sn[1], 1));
//	double oz = minima(B_high(sn[0], 0), B_low(sn[1], 1));
//
//	return ((zo*fw[0])+(oo*fw[1])+(zz*fw[2])+(oz*fw[3]))/(zo+oo+zz+oz);
//}
//
//int main(int argc, char** argv)
//{
//	// Initialize some global data
//
//	bool input;
//
//	//cout << "Type ? (0 for PID & 1 for FUZZY) :";
//	//cin >> input;
//
//	input = 1;
//
//	clock_t t1 = clock(), t3 = 0, t2 = 0;
//
//	SYSTEMTIME st;
//
//	Aria::init();
//	ArArgumentParser parser(&argc, argv);
//	parser.loadDefaultArguments();
//	ArRobot robot;
//	ArRobotConnector robotConnector(&parser, &robot);
//	ArAnalogGyro gyro(&robot);
//	ArGlobalRetFunctor1<bool, ArRobotPacket *> myhandleSimStatPacket(&handleSimStatPacket);
//	robot.addPacketHandler(&myhandleSimStatPacket, ArListPos::FIRST);
//
//	if (!robotConnector.connectRobot())
//	{
//		if (!parser.checkHelpAndWarnUnparsed())
//		{
//			ArLog::log(ArLog::Terse, "Could not connect to robot, will not have parameter file so options displayed later may not include everything");
//		}
//		else
//		{
//			ArLog::log(ArLog::Terse, "Error, could not connect to robot.");
//			Aria::logOptions();
//			Aria::exit(1);
//		}
//	}
//
//	if (!robot.isConnected())
//	{
//		ArLog::log(ArLog::Terse, "Internal error: robot connector succeeded but ArRobot::isConnected() is false!");
//	}
//
//	if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
//	{
//		Aria::logOptions();
//		Aria::exit(1);
//		return 1;
//	}
//	ArKeyHandler keyHandler;
//	Aria::setKeyHandler(&keyHandler);
//	robot.attachKeyHandler(&keyHandler);
//	printf("You may press escape to exit\n");
//	robot.runAsync(true);
//	ArUtil::sleep(1000);
//	robot.lock();
//	robot.comInt(ArCommands::ENABLE, 1);
//	robot.unlock();
//	robot.runAsync(true);
//	ArUtil::sleep(1000);
//	robot.lock();
//	robot.comInt(ArCommands::ENABLE, 1);
//	robot.unlock();
//
//	GetSystemTime(&st);
//	unsigned int startTime = st.wHour * 3600 * 1000 + st.wMinute * 60 * 1000 + st.wSecond * 1000 + st.wMilliseconds;
//	char fileName[50];
//	bool loop = false;
//	//cout << "Type ? (0 for PID / 1 for Fuzzy)<<"
//	sprintf(fileName, "data/Fuzzy_Map2/Final_Target_ Reach.csv");
//	FILE *fsA = fopen(fileName, "a");
//	fprintf(fsA, "nowTime, distX, distY,dist, Px, Py\n");
//
//	for (int c = 1; c <= 40; c++)
//	{
//
//
//		c = 13;
//
//		loop = true;
//		srand(time(NULL));
//		cout << "Iteration : \t" << c << endl;
//		double sonarDistVar = 0.0;
//		double sn[8]; //most left 0
//		double snMin = 0.0;// Minimum Sonar Value 
//		double snMinIndex = 0.0;//Minimum Sonar Number
//
//		double err = 0.0;
//
//		double theta = 0.0; // Odometry Error angle
//		double Px = 0.0; // Odometry Error X
//		double Py = -1500.0; // Odometry Error Y
//
//		double dist = 0.0;
//
//		struct robPose //structure to access the waypoints one by one. 
//		{
//			double x = 0;
//			double y = 0;
//		};
//		robPose robInitPose; // Not Used
//		robPose target; // waypoint to next target 
//
//		robot.lock();
//		robot.stop();
//		ArUtil::sleep(1000);
//		robot.unlock();
//
//		robot.com(ArCommands::SIM_RESET);
//
//		t1 = clock();
//
//		robot.stop();
//		//}
//		//---------------------------------------------------- WALL FOLLOWING ----------------------------------------------------------//
//		sprintf(fileName, "data/Fuzzy_Map2/Fuzzy_Wall_%d.csv", c);
//		t1 = clock();
//
//		FILE *fs = fopen(fileName, "w");
//		fprintf(fs, "nowTime, distX, distY,dist, Px, Py\n");
//
//		///////////////////////////////-----FUZZY----///////////////////////////////////////
//		while (input && loop) {
//
//			bool f_left, f_right;
//
//			if (rand() % 2 == 1)
//			{
//				f_left = true;
//				f_right = false;
//			}
//			else
//			{
//				f_right = true;
//				f_left = false;
//			}
//
//			cout << rand() % 2 << endl;
//			while (Aria::getRunning() && loop)// if Aria is Running and if Following is True,
//			{
//				for (int j = 0; j < 8; j++)
//				{
//					sn[j] = robot.getSonarRange(j);
//					if (sn[j] > 1600) sn[j] = 1600;
//				}
//
//				if (sn[2] < f_avoid)
//				{
//					leftVel = nominal / 2;//Left Velocity
//					RightVel = 0;// Right Velocity
//				}
//
//				if (sn[5] < f_avoid)
//				{
//					leftVel = 0;//Left Velocity
//					RightVel = nominal / 2;// Right Velocity
//
//				}
//
//				else if (f_left)
//				{
//
//					double zo = minima(B_low(sn[0], 0), B_high(sn[1], 1));
//					double oo = minima(B_high(sn[0], 0), B_high(sn[1], 1));
//					double zz = minima(B_low(sn[0], 0), B_low(sn[1], 1));
//					double oz = minima(B_high(sn[0], 0), B_low(sn[1], 1));
//
//					double op = ((zo*fw[0]) + (oo*fw[1]) + (zz*fw[2]) + (oz*fw[3])) / (zo + oo + zz + oz);
//
//					//cout <<"Left\t"<< zo<<"\t"<< oo<<"\t"<<zz<<"\t"<<oz<<"\t"<<op<< endl;
//
//
//					leftVel = nominal - op;//Left Velocity
//					RightVel = nominal + op;// Right Velocity
//
//				}
//				else if (f_right)
//				{
//
//					double zo = minima(B_low(sn[7], 7), B_high(sn[6], 6));
//					double oo = minima(B_high(sn[7], 7), B_high(sn[7], 7));
//					double zz = minima(B_low(sn[7], 7), B_low(sn[6], 6));
//					double oz = minima(B_high(sn[7], 7), B_low(sn[6], 6));
//
//					double op = ((zo*fw[3]) + (oo*fw[2]) + (zz*fw[1]) + (oz*fw[0])) / (zo + oo + zz + oz);
//
//					//cout <<"Right\t"<< zo<<"\t"<< oo<<"\t"<<zz<<"\t"<<oz<<"\t"<<op<< endl;
//
//
//					leftVel = nominal - op;//Left Velocity
//					RightVel = nominal + op;// Right Velocity
//
//				}
//
//				robot.lock();
//				robot.setVel2(leftVel, RightVel);
//				robot.unlock();
//				ArUtil::sleep(50);
//
//				//============== LOG DATA ==========================
//				Px = robot.getX();
//				Py = robot.getY() - 1500;
//				target.x = 8000;
//				target.y = 8500;
//				double nowTime;
//				t2 = clock();
//				nowTime = (double)(t2 - t1) / CLOCKS_PER_SEC;
//				double distX = (Px - target.x);
//				double distY = (Py - target.y);
//				dist = sqrt(distX * distX + distY * distY);
//
//				//cout << Py << "\t" << distY << "\t" << Px << "\t" << distX << "\t"<< dist<<endl;
//
//				fprintf(fs, "%.3f, %f, %f, %f, %.0f, %.0f", nowTime, distX, distY, dist, Px, Py);
//				cout << Px << "\t" << Py << "\t" << dist << endl;
//
//
//
//				// Map 1 Condition: 
//				//if (Px <= 500  && Px >= -500 && Py <= 1000 && Py >= 500) {  // if the Home goal y distance is less than 100
//				//if (Px <= 8500 && Px >= 7000 ){ 
//					//if (Py <= 9000 && Py >= 8500)
//				if (dist < 800)
//				{
//					{// Map 2
//						char buffer[500];
//						robot.stop();
//						printf("Wall following Completed using Fuzzy Controller...!!!");
//						sprintf(buffer, "Wall following Completed by %.2f...!!!", nowTime);
//						robot.comStr(ArCommands::SIM_MESSAGE, buffer);
//						fprintf(fsA, "%.3f, %f, %f, %f, %.0f, %.0f\n", nowTime, distX, distY, dist, Px, Py);
//						//fprintf(fsA, " Last distance =,  %.3f, Iteration = , %d\n", dist, c);
//						//fclose(fsA);
//						loop = false;
//					}
//				}
//				fprintf(fs, "\n");
//
//				//cout << Px << "\t" << Py << endl;
//
//			}
//
//		}
//		fclose(fs);
//	
//	
//	Aria::exit(0); // Exit aria
//	}			   //robot.waitForRunExit();
//	//}
//
//	return 0;
//}