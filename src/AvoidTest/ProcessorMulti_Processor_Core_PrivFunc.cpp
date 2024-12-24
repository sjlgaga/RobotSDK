// You need to program this file.

#include "../NoEdit/ProcessorMulti_Processor_Core_PrivFunc.h"
#include <cmath>
#include <algorithm>
//*******************Please add static libraries in .pro file*******************
// e.g. unix:LIBS += ... or win32:LIBS += ...

bool DECOFUNC(setParamsVarsOpenNode)(QString qstrConfigName, QString qstrNodeType, QString qstrNodeClass, QString qstrNodeName, void *paramsPtr, void *varsPtr)
{
	XMLDomInterface xmlloader(qstrConfigName, qstrNodeType, qstrNodeClass, qstrNodeName);
	ProcessorMulti_Processor_Core_Params *params = (ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars *vars = (ProcessorMulti_Processor_Core_Vars *)varsPtr;
	/*======Please Program below======*/
	/*
	Function: open node.
	Procedure:
	1: load parameters (params). [GetParamValue(xmlloader,params,tag); GetEnumParamValue(xmlloader,params,tag); GetUEnumParamValue(xmlloader,params,tag)]
	2: initialize variables (vars).
	3: If everything is OK, return 1 for successful opening and vice versa.
	*/

	return 1;
}

bool DECOFUNC(handleVarsCloseNode)(void *paramsPtr, void *varsPtr)
{
	ProcessorMulti_Processor_Core_Params *params = (ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars *vars = (ProcessorMulti_Processor_Core_Vars *)varsPtr;
	/*======Please Program below======*/
	/*
	Function: close node.
	Procedure:
	1: handle/close variables (vars).
	2: If everything is OK, return 1 for successful closing and vice versa.
	*/

	return 1;
}

void DECOFUNC(getInternalTrigger)(void *paramsPtr, void *varsPtr, QObject *&internalTrigger, QString &internalTriggerSignal)
{
	ProcessorMulti_Processor_Core_Params *params = (ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars *vars = (ProcessorMulti_Processor_Core_Vars *)varsPtr;
	internalTrigger = NULL;
	internalTriggerSignal = QString();
	/*======Occasionally Program above======*/
	/*
	Function: get internal trigger [defined in vars] for node.
	You need to program here when you need internal trigger (internalTrigger + internalTriggerSignal) for node.
	E.g.
	internalTrigger=&(vars->trigger);
	internalTriggerSignal=QString(SIGNAL(triggerSignal()));
	*/
}

void DECOFUNC(initializeOutputData)(void *paramsPtr, void *varsPtr, boost::shared_ptr<void> &outputDataPtr)
{
	ProcessorMulti_Processor_Core_Params *params = (ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars *vars = (ProcessorMulti_Processor_Core_Vars *)varsPtr;
	outputDataPtr = boost::shared_ptr<void>(new SourceDrainMono_Sensor_EncoderIMU_Order_InputData());
	/*======Occasionally Program below/above======*/
	/*
	Function: initial output data.
	You need to program here when you need to manually initialize output data.
	*/
}

void DECOFUNC(getMultiInputDataSize)(void *paramsPtr, void *varsPtr, QList<int> &inputDataSize)
{
	ProcessorMulti_Processor_Core_Params *params = (ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars *vars = (ProcessorMulti_Processor_Core_Vars *)varsPtr;
	inputDataSize = QList<int>();
	/*======Please Program above======*/
	/*
	Function: get input data size to be grabbed from buffer.
	Rules:
	inputDataSize=0: grab and remove all data from buffer.
	inputDataSize>0: grab inputDataSize latest data from buffer.
	inputDataSize<0: grab and remove inputDataSize ancient data from buffer.
	E.g.
	inputDataSize=QList<int>()<<0<<1<<-1...;
	*/
}

void LP2GP(double lx, double ly, double posx, double posy, double ori, double *gx, double *gy)
{
	// 此处需完成
	// 输入：lx,ly为激光坐标系下激光点坐标；posx,posy,ori为机器人当前位姿；
	// 输出：gx,gy即函数输出的变换后的全局坐标

	*gx = posx + lx * sin(ori) + ly * cos(ori);
	*gy = posy - lx * cos(ori) + ly * sin(ori);
}

// Input Port #0: Buffer_Size = 10, Params_Type = SourceDrainMono_Sensor_EncoderIMU_Params, Data_Type = SourceDrainMono_Sensor_EncoderIMU_Data
// Input Port #1: Buffer_Size = 10, Params_Type = SensorTimer_Sensor_URG_Params, Data_Type = SensorTimer_Sensor_URG_Data
// Input Port #2: Buffer_Size = 10, Params_Type = SensorTimer_Sensor_xtion_Params, Data_Type = SensorTimer_Sensor_xtion_Data
bool DECOFUNC(processMultiInputData)(void *paramsPtr, void *varsPtr, QVector<QVector<void *> > inputParams, QVector<QVector<void *> > inputData, void *outputData, QList<int> &outputPortIndex)
{
	ProcessorMulti_Processor_Core_Params *params = (ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars *vars = (ProcessorMulti_Processor_Core_Vars *)varsPtr;
	QVector<SourceDrainMono_Sensor_EncoderIMU_Params *> inputparams_0;
	copyQVector(inputparams_0, inputParams[0]);
	QVector<SensorTimer_Sensor_URG_Params *> inputparams_1;
	copyQVector(inputparams_1, inputParams[1]);
	QVector<SensorTimer_Sensor_xtion_Params *> inputparams_2;
	copyQVector(inputparams_2, inputParams[2]);
	QVector<SourceDrainMono_Sensor_EncoderIMU_Data *> inputdata_0;
	copyQVector(inputdata_0, inputData[0]);
	QVector<SensorTimer_Sensor_URG_Data *> inputdata_1;
	copyQVector(inputdata_1, inputData[1]);
	QVector<SensorTimer_Sensor_xtion_Data *> inputdata_2;
	copyQVector(inputdata_2, inputData[2]);
	SourceDrainMono_Sensor_EncoderIMU_Order_InputData *outputdata = (SourceDrainMono_Sensor_EncoderIMU_Order_InputData *)outputData;
	outputPortIndex = QList<int>();
	if (inputdata_0.size() == 0)
	{
		return 0;
	}
	if (inputdata_1.size() == 0)
	{
		return 0;
	}
	if (inputdata_2.size() == 0)
	{
		return 0;
	}
	/*======Please Program below======*/
	/*
	Step 1: process inputdata_index, then store it into outputdata.
	Step 2 [optional]: determine the outputPortIndex. (if not, outputdata will be sent by all ports)
	E.g. outputPortIndex=QList<int>()<<(outportindex1)<<(outportindex2)...
	*/

	// inputdata_0                                             // EncoderIMU
	// inputdata_1                                             // URG
	// inputdata_2                                             // Xtion (RGB && depth)
	// cv::imshow("color", inputdata_2.front()->cvColorImg);   // Show RGB image
	// cv::imshow("depth", inputdata_2.front()->cvDepthImg);   // Show depth image

	short steer = 100; // [-400, 400]
	short speed = 100; // [-180, 180]


    double targetX1 = 42;
    double targetY2 = 2;

    double targetX2 = 40;
    double targetY2 = 36.8;
	const double PI = 3.1415926535897932384626433832795;
    int obstacleAngle = 50;
	double obstacleWeight = 1;
    double targetWeight = 10;
    // 小车可选角度（依赖于 steer）
	int availableMaxAngle = 90;					// available angle: -45 ~ 45
    // 小车可选速度（依赖于 speed）
	int availableMaxSpeed = 180;
    double AvoidThreshold = 2;                //暂时用来测避障的

    // 当前小车位置
    double posx = inputdata_0.front()->x;
    double posy = inputdata_0.front()->y;
    double carori = inputdata_0.front()->orientation;
    qDebug()<< "posx: " << posx<<" posy: "<<posy << endl;
    /*
	// 最近目标点


	// 最近目标点距离
	double targetDistance = sqrt(pow(targetX - posx, 2) + pow(targetY - posy, 2));

    */

	// 初始化障碍物全局坐标
	int urgSize = inputdata_1.front()->datasize;
	double urgUnit = inputparams_1.front()->unit;
	short *urgData = inputdata_1.front()->data;
	const double urgRes = 0.5;
	std::vector<std::pair<double, double> > obstacleGxGy;
	int leftDatasum = 0, rightDatasum = 0, midDatasum = 0;
	bool isAvoiding = false;
    std::pair<double,double> minObstacleGxGy = std::make_pair(0,0);

    double minObDistance = (1<<20);
	for (int i = 0; i < urgSize; i++)
	{ // 60 degrees, left and right
		if (urgData[i] == 0)
			continue;
		if (i < 120)
			leftDatasum += urgData[i];
		else if (i >= 120 && i < 240)
			midDatasum += urgData[i];
		else if (i >= 240)
			rightDatasum += urgData[i];

		// 计算障碍物全局坐标 (只考虑前方障碍物)
		if (i >= 180 - obstacleAngle && i <= 180 + obstacleAngle)
		{
            double dis = double(urgData[i]) / urgUnit;
			double angle = (urgRes * i) / 180.0 * PI;
			double gx, gy;
			double lx = dis * cos(angle);
			double ly = dis * sin(angle);
			LP2GP(lx, ly, posx, posy, carori, &gx, &gy);
			obstacleGxGy.push_back(std::make_pair(gx, gy));
//            qDebug() << "lxly" << lx << " " << ly << endl;
//            qDebug() << "gxgy" << gx << " " << gy << endl;

			// 障碍物与小车距离
			double distance = sqrt(pow(gx - posx, 2) + pow(gy - posy, 2));
//            minObDistance = std::min(minObDistance,distance);
            if(distance < minObDistance){
                minObstacleGxGy = std::make_pair(gx,gy);
                minObDistance = distance;
            }
			if (distance < AvoidThreshold) // 如果障碍物距离小于最近目标点距离，则走向目标点时存在障碍物，需要避障
				isAvoiding = true;
		}
	}

    qDebug()<< "minObDistance: " << minObDistance << endl;
    qDebug()<< "is avoiding: " << isAvoiding << endl;
    qDebug()<<endl;

	int obstacleLength = obstacleGxGy.size();

    double angleK = 400 / (availableMaxAngle / 2); // each angle has linear relation with steer
    double distanceK = 10000; // each distance has linear relation with speed (distance = time * speed)

	double bestEvaluation = 0;
	double bestAngle = 0;
    double bestSpeed = 150;

	// 动态窗口法 遍历所有可能的角度和速度
	for (int i = 90 - availableMaxAngle / 2; i <= 90 + availableMaxAngle / 2; i++)
    {
			// 计算每个角度对应的移动距离
            double moveDistance = 0.5;
			// 计算每个角度对应的移动角度
			double moveAngle = i / 180.0 * PI;

			// 计算每个角度对应的预测位置
			double availableX;
			double availableY;
			// 激光点在全局坐标系中的位置 单位m
			double lx = moveDistance * cos(moveAngle);
			double ly = moveDistance * sin(moveAngle);
//            qDebug() << "posx: " << posx << " posy: " << posy << endl;
//            qDebug() << "obx: " << minObstacleGxGy.first << " oby: " << minObstacleGxGy.second << endl;
//            qDebug() << "x: " << availableY << "  y: " << availableY << endl;
			LP2GP(lx, ly, posx, posy, carori, &availableX, &availableY);

//            qDebug() << "dist: " << moveDistance << " cosA: " << cos(moveAngle) << " sinA: " << sin(moveAngle) << endl;
            double minObstacleDistance = sqrt(pow(availableX -minObstacleGxGy.first, 2) + pow(availableY - minObstacleGxGy.second, 2));

			// 计算预测位置与障碍物的最小距离
			if (isAvoiding)
			{
//				for (int k = 0; k < obstacleLength; k++)
//				{
//					double obstacleDistance = sqrt(pow(availableX - obstacleGxGy[k].first, 2) + pow(availableY - obstacleGxGy[k].second, 2));
//					if (obstacleDistance < minObstacleDistance)
//					{
//						minObstacleDistance = obstacleDistance;
//					}
//				}
			}
			else
			{
				minObstacleDistance = 0;
			}

			// 计算目标点距离
            double targetDistance1 = sqrt(pow(availableX - targetX1, 2) + pow(-availableY - targetY1, 2));
            double targetDistance2 = sqrt(pow(availableX - targetX2, 2) + pow(-availableY - targetY2, 2));
            double targetDistance = std::min(targetDistance1, targetDistance2);
			// 评价函数（预测位置与障碍物的距离越大，评价函数越大；预测位置与目标点的距离越小，评价函数越大）
            double evaluation = minObstacleDistance * obstacleWeight + targetWeight / targetDistance;
//            qDebug() << "ob eval: " << minObstacleDistance * obstacleWeight << " target eval: "<< targetWeight / targetDistance << endl;

			// 更新最优解
			if (evaluation > bestEvaluation)
			{
				bestEvaluation = evaluation;
				bestAngle = i;
                bestSpeed = 150;
			}

	}
	speed = bestSpeed;
    steer = -(bestAngle - 90.0) * angleK;

    speed = speed;
    steer =  steer;
    qDebug() << bestAngle << "   " << steer << endl;
	// Show RGB image && compass
	double ori = -((double)steer / 400.0) * (M_PI / 2.0);
	cv::Mat img;
	inputdata_2.front()->cvColorImg.copyTo(img);
	cv::flip(img, img, 1);
	cv::Point compass = cv::Point(100, 100);
	cv::circle(img, compass, 80, cv::Scalar(0, 255, 0), 1, CV_AA);
	cv::line(img, compass,
			 cv::Point(compass.x - sin(ori) * 80,
					   compass.y - cos(ori) * 80),
			 cv::Scalar(0, 255, 0), 3, CV_AA);
	cv::imshow("color", img);

	//--------------------------------------------
	int maxSpeed = 180;
	if (speed > maxSpeed)
		speed = maxSpeed;
	if (speed < -maxSpeed)
		speed = -maxSpeed;
	char dataput[20];
	dataput[0] = 0xF8;
	dataput[1] = 4;
	*(short *)&dataput[2] = (short)steer;
	*(short *)&dataput[4] = (short)speed;
	dataput[6] = 0x8F;
	outputdata->datagram = QByteArray(dataput, 7);
	return 1;
}
