// You need to program this file.

#include "../NoEdit/ProcessorMulti_Processor_Core_PrivFunc.h"
#include <cmath>
#include <vector>

std::vector<cv::Point> carPath;

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

    vars->lastErr = 0;
    vars->sumErr = 0;
    vars->back = false;
    vars->IsTurningRight = false;
    vars->IsTurningLeft = false;
    vars->IsAvoiding = false;
    vars->AvoidRight = 0;
    vars->backori = 0;
    vars->turnstamp = QTime(0, 0, 0);

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
    short speed = 180; // [-180, 180]




    double posx = inputdata_0.front()->x;
	double posy = inputdata_0.front()->y;
    drawCarPositionOnWhiteBackground(posx, posy, 1600, 1200);

    qDebug()<<"posx: "<<posx<<endl;
    qDebug()<<"posy: "<<posy<<endl;

    int urgSize = inputdata_1.front()->datasize;
    double urgUnit = inputparams_1.front()->unit;
    short *urgData = inputdata_1.front()->data;
    int leftDatasum = 0, rightDatasum = 0, midDatasum = 0;

    for (int i = 0; i < urgSize; i++)
    {
        if (i < 120)
            rightDatasum += urgData[i];
        else if (i >= 120 && i < 240)
            midDatasum += urgData[i];
        else
            leftDatasum += urgData[i];
    }

    int subLeftRight = rightDatasum - leftDatasum;
    double err = subLeftRight / 1000;



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

    char dataput[20];
    dataput[0] = 0xF8;
    dataput[1] = 4;
    *(short *)&dataput[2] = (short)steer;
    *(short *)&dataput[4] = -1 * (short)speed;
    dataput[6] = 0x8F;
    outputdata->datagram = QByteArray(dataput, 7);
    return 1;
}

void drawCarPositionOnWhiteBackground(double x, double y, int width, int height) {
// 创建一个纯白背景图像
cv::Mat img = cv::Mat::ones(height, width, CV_8UC3) * 255;

// 将所有历史轨迹绘制到背景上
for (size_t i = 0; i < carPath.size(); ++i) {
cv::circle(img, carPath[i], 3, cv::Scalar(0, 0, 255), -1, 8); // 替换 LINE_AA 为 8
}

// 当前坐标点映射到图像坐标
int img_x = static_cast<int>(x /10); // 假设 1 单位 = 10 像素（比例可调整）
int img_y = static_cast<int>(y / 10000);

// 确保坐标点在图像范围内
img_x = std::max(0, std::min(img_x, width - 1));
img_y = std::max(0, std::min(img_y, height - 1));

// 添加当前点到轨迹列表
carPath.push_back(cv::Point(img_x, img_y)); // 使用 push_back 替代 emplace_back

// 绘制当前点
cv::circle(img, cv::Point(img_x, img_y), 3, cv::Scalar(0, 0, 255), -1, 8); // 替换 LINE_AA 为 8

// 显示图像
cv::imshow("Car Position Visualization", img);
}

