//#include "Aria.h"
//#include "ariaUtil.h"
//#include <math.h>
//#include <time.h> 
//#include <Windows.h>
//#include <stdio.h>
//#include <fstream>
//#include <iostream>
//
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
//double nominal = 250;
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
//
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
//int main(int argc, char** argv)
//{
//	// Initialize some global data
//
//
//
//	clock_t t1 = clock(), t3 = 0, t2 = 0;
//	//gyro.activate();
//	//robot.unlock();
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
//
//	for (int c = 1; c <= 100; c++)
//	{
//		cout << "for loop\t" << c << endl;
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
//		bool target_not_completed = true; //Odometry selection
//		bool follow_the_wall = false; // Wall following Selection
//		double robHeading = 0.0;
//		double LWheel = 0.0;
//		double RWheel = 0.0;
//		double robTravel = 0.0;
//		double dist = 0.0;
//		double ang = 0.0;
//		double errDistPrev = 1;
//		double errAngPrev = 1;
//		double errAngKi = 0.0;
//
//		struct robPose //structure to access the waypoints one by one. 
//		{
//			double x = 0;
//			double y = 0;
//		};
//		robPose robInitPose; // Not Used
//		robPose target; // waypoint to next target 
//
//		target.x = 0.0;
//		target.y = 0.0;
//		robPose nextTarget[19]; //all 19 waypoints including the initial Pose and a End with a slight reverse to make way for wall following 
//		{nextTarget[0].x = 0;
//		nextTarget[0].y = -1500;
//		nextTarget[1].x = 0;
//		nextTarget[1].y = 750;
//		nextTarget[2].x = -2800;
//		nextTarget[2].y = 750;
//		nextTarget[3].x = -2800;
//		nextTarget[3].y = 2750;
//		nextTarget[4].x = 2000;
//		nextTarget[4].y = 2750;
//		nextTarget[5].x = 2000;
//		nextTarget[5].y = 750;
//		nextTarget[6].x = 4000;
//		nextTarget[6].y = 750;
//		nextTarget[7].x = 4000;
//		nextTarget[7].y = 4750;
//		nextTarget[8].x = 2000;
//		nextTarget[8].y = 4750;
//		nextTarget[9].x = 2000;
//		nextTarget[9].y = 7750;
//		nextTarget[10].x = -2750;
//		nextTarget[10].y = 7750;
//		nextTarget[11].x = -2750;
//		nextTarget[11].y = 12750;
//		nextTarget[12].x = -500;
//		nextTarget[12].y = 12750;
//		nextTarget[13].x = -500;
//		nextTarget[13].y = 9750;
//		nextTarget[14].x = 2000;
//		nextTarget[14].y = 9750;
//		nextTarget[15].x = 2000;
//		nextTarget[15].y = 12750;
//		nextTarget[16].x = 6250;
//		nextTarget[16].y = 12750;
//		nextTarget[17].x = 6250;
//		nextTarget[17].y = 15000;
//		nextTarget[18].x = 6250;
//		nextTarget[18].y = 14000; }
//		int robPoseIndex = 1;	// waypoint number 
//
//		double Pxm = 0.0;
//		double Pym = 0.0;
//		double PThm = 0.0;
//		double PrevX = 0.0;
//		double PrevY = 0.0;
//		double PrevTh = 0.0;
//		double Xtol = 0.05; // X Odom Error Tolerance
//		double Ytol = 0.05; // Y Odom Error Tolerance
//		double Thtol = 0.05*M_PI / 180.0; // Angle Odom Error Tolerance in Radians
//		sprintf(fileName, "data/file_Odo_%d.csv", c);
//		FILE *fs = fopen(fileName, "w");
//		fprintf(fs, "nowTime, distX, distY, Px, Py, realX, realY\n");
//
//		FILE *fsA;// = fopen("Targets.csv", "a");;
//		target_not_completed = true;
//
//
//		robot.lock();
//		robot.stop();
//		ArUtil::sleep(1000);
//		robot.unlock();
//
//		robot.com(ArCommands::SIM_RESET);
//
//		follow_the_wall = false;
//		t1 = clock();
//		while (target_not_completed == true && follow_the_wall == false) {
//			cout << "Odo" << endl;
//
//			target = nextTarget[robPoseIndex]; // assign the x and y of destination 
//			for (int j = 0; j < 8; j++)
//			{//get sonar values
//				sn[j] = robot.getSonarRange(j);
//				if (sn[j] > 300.0)
//					sn[j] = 300.0;
//				//printf("%.0f, ", sn[j]);
//			}
//			//printf("\n");
//
//			sonarDistVar = sn[0] * 0.8 + sn[1] * 0.9 + sn[2] * 1.0 + sn[3] - sn[4] - sn[5] * 1.0 - sn[6] * 0.9 - sn[7] * 0.8;
//
//			Px = robot.getX(); // Get Global X
//			Py = robot.getY() - 1500.0; // Get Global Y
//			theta = robot.getTh()*M_PI / 180.0; // Robot Rotation Angle Relative to the Ground Plane. (Positive X is to the Right) in Radians
//			Pxm += (Px - PrevX)*Xtol; // X Euclidean Distance multiplied by the Odometry Error to get the error compensation
//			Pym += (Py - PrevY)*Ytol; //Y Euclidean Distance multiplied by the Odometry Error to get the error compensation
//			PThm += (theta - PrevTh)*Thtol;// Angle Euclidean Distance multiplied by the Odometry Error to get the error compensation
//			PrevX = Px; //Previous X Value
//			PrevY = Py; // Previous Y Value
//			PrevTh = theta; //Previous Angle Value
//
//			Px -= Pxm; //Compensating for the Odometry Error by Reducing it by the Global value for X, Y and the Angle
//			Py -= Pym;
//			theta -= PThm;
//
//			double distX = (Px - target.x); // Calculated X Distance to the target
//			double distY = (Py - target.y); // Calculated Y Distance to the Target
//			dist = sqrt(distX * distX + distY * distY); //Calculated Euclidean Distance to the target 
//			ang = atan2(target.y - Py, target.x - Px); // Calculated Angle to the Target
//
//													   //////////////------------------------- ANGLE TUNING -----------------------------------////////////////////////
//
//
//			double kpAngleError = 350.0; //Proportional Gain for Angle Error
//			double KdAngError = 350.0; // Deravative Gain for Angle Error
//			double KiAngError = 0.0001;// Integral Gain For Angle Error
//			double angErrE = (ang - theta); // Angle error
//			errAngKi += angErrE; //Integrating the Error
//			if (errAngKi > 3)// Integrator cap between -3 and 3
//				errAngKi = 3.0;
//			else if (errAngKi < -3)
//				errAngKi = -3.0;
//			double angErr = angErrE * kpAngleError + (angErrE - errAngPrev)*KdAngError + errAngKi * KiAngError;
//			errAngPrev = angErrE;// Previous Error
//
//			if ((ang - theta) < -M_PI) //Checking wheather the Turn is Left or Right
//				angErr = angErr + M_PI * 2 * kpAngleError;// Add a Gain to the Existing PID value if the error is less than -180 
//			else if ((ang - theta) > M_PI)
//				angErr = angErr - M_PI * 2 * kpAngleError;// Add a Gain to the Existing PID value if the error is greater than 180 
//
//
//														  //////////////////////----------------------DISTANCE TUNING------------------------//////////////////////
//			double KpDistError = 0.9;//0.1
//			double KdDistError = 1.0;//0.1
//			err = (dist)*KpDistError + (dist - errDistPrev)*KdDistError;// Distance PID Value
//			errDistPrev = dist;// Previous Error 
//							   //err = 200;
//			if (err > 550)//550
//				err = 550;// Maintain the Maximum error at 550
//
//			int dir = 1;
//
//			if (sn[0] < 200.0 || sn[1] < 200.0 || sn[2] < 250.0 || sn[3] < 250.0 || sn[4] < 250.0 || sn[5] < 250.0 || sn[6] < 200.0 || sn[7] < 200.0) {
//				//Checking the Minimum Distances of all the sonars. Execute Condition even if one falls below the Assigned Value
//				snMin = sn[0];
//				//Start with the First Sensor 
//				for (int j = 0; j < 8; j++) {
//					if (snMin > sn[j]) {
//						//Identify the Minimum sonar Value and the Sonar Index by comparing all the Sonars, Starting with the First 
//						snMin = sn[j];
//						snMinIndex = j;
//					}
//				}
//				if ((3 - snMinIndex) >= 0)// Choose which Direction 
//					dir = 1;//Left Direction Sonar Array is Closer to the Wall
//				else
//					dir = -1;//Right Direction Sonar Array is Closer to the Wall
//				double snTrapWt = 0.01;// Trap Weight 
//				LWheel = dir * (20.0 + (5000 - snMin) * snTrapWt);// Left Wheel Velocity (20.0 is used to have a Velocity if snMin output Value is 5000)
//				RWheel = -dir * (20.0 + (5000 - snMin) *snTrapWt);// Right Wheel Velocity (20.0 is used to have a Velocity if snMin output Value is 5000)
//				if (abs(sonarDistVar) < 100) {// Checking Wheather Any of the Sonars are closer to the wall by less than 100mm
//											  // Changing the Wheel Velocities
//					LWheel = dir * 100 - 10;
//					RWheel = -dir * 100 - 10;
//				}
//				if (abs(LWheel) > 100)
//					LWheel = LWheel / abs(LWheel) * 100;//Limit the Left Wheel Velocity to 100
//				if (abs(RWheel) > 100)
//					RWheel = RWheel / abs(RWheel) * 100;//Limit the Right Wheel Velocity to 100 
//														//printf("Pose2: ");
//			}
//			else {// if the Sonars are > 200 and <= 300,
//				if (angErr > 300)// if the Angle PID output is > 300
//					err = 100;// set the Distance PID to 100
//				LWheel = err + 20.0 - angErr - sonarDistVar * 0.8;// if a Negative value comes to the angle PID, turn will be to the Left
//				RWheel = err + 20.0 + angErr + sonarDistVar * 0.8;// if a Positive Value comes to the angle PID , turn will be to the Right
//
//																  // Velocity Cap to 800 of both left and Right wheels
//				if (abs(LWheel) > 800)//400
//					LWheel = LWheel / abs(LWheel) * 800;
//				if (abs(RWheel) > 800)
//					RWheel = RWheel / abs(RWheel) * 800;
//				//Velocity Cap to 800 of both left and Right wheels
//			}
//			robot.comInt(ArCommands::SIM_STAT, 1);
//			robot.setVel2(LWheel, RWheel);// Set
//
//										  //printf("Lerr, Rerr, sonarDistVar, angErr, Px, Py = %.0f, %.0f, %f, %f, %.0f, %.0f\n", LWheel, RWheel, sonarDistVar, angErr, Px, Py);
//
//			GetSystemTime(&st);
//			t3 = clock();
//			double nowTime = (double)(t3 - t1) / CLOCKS_PER_SEC;
//			//unsigned int nowTime = st.wHour * 3600 * 1000 + st.wMinute * 60 * 1000 + st.wSecond * 1000 + st.wMilliseconds - startTime;
//			fprintf(fs, "%.3f, %f, %f, %.0f, %.0f, %i, %i,", nowTime, distX, distY, Px, Py, realX, realY);
//			char buffer[25];
//
//			if (abs(dist) <= 25) {// if the Robot is Closer to the Target by 25mm or Less, Stop the Robot
//				if (robPoseIndex >= 17) { // Finish Executing the Odometry Model if the Pose index is 18 or greater than 18
//					cout << "odo done" << endl;
//					target_not_completed = false;// Finish Odometry model
//					follow_the_wall = true;// Start Wall Following model
//					fsA = fopen("data/Targets.csv", "a");
//					fprintf(fsA, "%.3f, %f, %f, %f, %.0f, %.0f, %i, %i,\n", nowTime, distX, distY, dist, Px, Py, realX, realY);
//					fclose(fsA);
//				}
//				cout << "next target" << robPoseIndex << endl;
//				robot.stop();
//				//printf("Goal %i reached...!!!", robPoseIndex);
//				fprintf(fs, "Goal %i reached...!!!, %f, %f", robPoseIndex, target.x, target.y);
//				sprintf(buffer, "Goal %i reached by %i...!!!", robPoseIndex, nowTime);
//				robot.comStr(ArCommands::SIM_MESSAGE, buffer);
//				robPoseIndex++;
//			}
//			fprintf(fs, "\n");
//
//			ArUtil::sleep(49);
//
//
//
//		}
//		fclose(fs);
//		cout << "odo done!" << endl;
//		robot.stop();
//		//}
//		//---------------------------------------------------- WALL FOLLOWING ----------------------------------------------------------//
//		sprintf(fileName, "data/File_Wall_%d.csv", c);
//		t1 = clock();
//		fs = fopen(fileName, "w");
//		fprintf(fs, "nowTime, distX, distY, Px, Py\n");
//
//
//		while (follow_the_wall == true && target_not_completed == false) {
//			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//			double error_L[3], error_R[3], errorLeft = 0, errorRight = 0; //Left Soanr Error,Right Sonar Error,Full Error Right,Full Error Left
//			double PID_L = 0, PID_LD = 0; //Left PID //Left Distance PID
//			double PID_R = 0, PID_RD = 0; //Right PID // Right Distance PID
//			double errorLD; // Left Distance Error
//			double errorRD; // Right Distance Error
//
//			double integral_L = 0, diff_L = 0, last_error = 0; // Left Angle Integral, Differential and Previous Error
//			double integral_R = 0, diff_R = 0, last_error_R = 0; // Right Angle Integral, Differential and Previous Error
//			double integral_LD = 0, diff_LD = 0, last_errorLD = 0; // Left Distance Integral, Differential and Previous Error
//			double integral_RD = 0, diff_RD = 0, last_errorRD = 0; // Left Distance Integral, Differential and Previous Error
//			bool Following = true; //Wheather Following or not
//			bool firstValDone = false;
//			double sonarLeftFirstVal = sn[0]; // Get the First Sonar Value for the Left Most Sonar
//			double sonarRightFirstVal = sn[7]; // Get the First Sonar Value for the Right Most Sonar
//
//			int x = 0;
//			while (Aria::getRunning() && Following)// if Aria is Running and if Following is True,
//			{
//				for (int j = 0; j < 8; j++) {// Get the Sonar Values for all the Sonars
//					sn[j] = robot.getSonarRange(j);
//					//printf("%.0f, ", sn[j]);
//				}	//printf("\n");
//				sonarRead(sn);
//				sonarLeftFirstVal = sn[0];
//				sonarRightFirstVal = sn[7];
//				//firstValDone = true;
//				//cout << Following << " " << firstValDone << endl;
//
//				while (Following)
//				{
//					for (int j = 0; j < 8; j++) {
//						sn[j] = robot.getSonarRange(j);
//						printf("%.0f, ", sn[j]);
//					}	printf("\n");
//					sonarRead(sn);
//					int dir = 1;
//
//
//					if (sonarLeftFirstVal<sonarRightFirstVal)//if sonar Lef Value is Less than the Right, Select the Left Wall
//					{
//						cout << "Left Wall" << endl;
//						if (sn[2] > avoid_distance)// if 3rd sonar from the first sonar in the left araay is greater than the Avoid Distance, 
//						{
//							//leftVel, RightVel = conditions.LeftAngleConditions(error_L, errorLeft, integral_L, diff_L, last_error, error_R, PID_L);
//							//cout << leftVel << " " << RightVel << endl;
//							for (int i = 0; i < 3; i++)
//							{
//								error_L[i] = leftang[i] - 90;// Get Errors of the Left Side Angles
//							}
//							errorLeft = (error_L[0] * w[0]) + (error_L[1] * w[1]) + (error_L[2] * w[2]);// Multiply by the Weight 
//							integral_L = integral_L + errorLeft;//Integral Error
//							diff_L = errorLeft - last_error;// Differential Error
//							last_error = errorLeft;// Previous Error
//
//
//							PID_L = (kp*errorLeft) + (ki*integral_L) + (kd*diff_L);// Left PID Value
//							leftVel = nominal - PID_L / 2;// Left Velocity
//							RightVel = nominal + PID_L / 2;// Right Velocity
//						}
//						else// Left Distance
//						{
//							errorLD = sn[2] - avoid_distance;//Left Distance Error
//							integral_LD = integral_LD + errorLD;//Integral Error 
//							diff_LD = errorLD - last_errorLD;// Differential Error 
//							last_errorLD = errorLD;// Previous Error
//
//							PID_LD = (kpLD*errorLD) + (kiLD*integral_LD) + (kdLD*diff_LD);// PID Value for Distance. Left
//
//							leftVel = nominal - PID_LD / 2;// Left Wheel Velocity
//							RightVel = nominal + PID_LD / 2;// Right Wheel Velocity
//						}
//					}
//					else
//					{
//						cout << "Right Wall" << endl;
//
//						if (sn[5] > avoid_distance)
//						{
//							//leftVel, RightVel = conditions.RightAngleConditions(error_R, errorRight, integral_R, diff_R, last_error_R, error_L, PID_R);
//
//							for (int i = 0; i < 3; i++)
//							{
//								error_R[i] = rightang[i] - 90;// Get Errors of the Right Side Angles
//							}
//							errorRight = (error_R[0] * w[0]) + (error_R[1] * w[1]) + (error_R[2] * w[2]);// Multiply by the Weight
//							integral_R = integral_R + errorRight;//Integral Error
//							diff_R = errorRight - last_error_R;// Differential Error
//							last_error_R = errorRight;// Previous Error
//
//
//							PID_R = (kp*errorRight) + (ki*integral_R) + (kd*diff_R);// Right PID Value
//							leftVel = nominal + PID_R / 2;//Left Velocity
//							RightVel = nominal - PID_R / 2;// Right Velocity
//						}
//						else
//						{
//							errorRD = sn[5] - avoid_distance;// Right Distance Error
//							integral_RD = integral_RD + errorRD;//Integral Error
//							diff_RD = errorRD - last_errorRD;// Differential Error
//							last_errorRD = errorRD;// Previous Error
//
//							PID_RD = (kpRD*errorRD) + (kiRD*integral_RD) + (kdRD*diff_RD);//Right Distance PID
//
//							leftVel = nominal + PID_RD / 2;// Left Wheel Velocity
//							RightVel = nominal - PID_RD / 2;// Right Wheel Velocity
//						}
//						//}
//					}
//					robot.lock();
//					robot.setVel2(leftVel, RightVel);
//					robot.unlock();
//					ArUtil::sleep(50);
//
//					//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//					Px = robot.getX();
//					Py = robot.getY() - 1500.0;
//					target.x = 0;
//					target.y = -1500;
//					double nowTime;
//					t2 = clock();
//					nowTime = (double)(t2 - t1) / CLOCKS_PER_SEC;
//					double distX = (Px - target.x);
//					double distY = (Py - target.y);
//					dist = sqrt(distX * distX + distY * distY);
//
//
//					fprintf(fs, "%.3f, %f, %f, %.0f, %.0f,", nowTime, distX, distY, Px, Py);
//					if (distY <= -100) {  // if the Home goal y distance is less than 100
//						char buffer[50];
//						follow_the_wall = false; // exit the wall following
//						Following = false; // exit the wall following
//						robot.stop();
//						printf("Wall following Completed...!!!");
//						sprintf(buffer, "Wall following Completed by %i...!!!", nowTime);
//						robot.comStr(ArCommands::SIM_MESSAGE, buffer);
//					}
//					fprintf(fs, "\n");
//				}
//			}
//		}
//		fclose(fs);
//
//	}
//	Aria::exit(0); // Exit aria
//				   //robot.waitForRunExit();
//
//
//	return 0;
//}