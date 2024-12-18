#ifndef LOCATE_HH
#define LOCATE_HH

#include <vector>
#include <QVector>

// 地图数据假设大小为 500x500，实际定义在主程序中
extern int g_map[][500];

// 激光雷达数据结构
struct LidarData {
    int timestamp;   // 时间戳
    int datasize;    // 数据大小
    short *data;     // 数据指针
};

// 激光雷达参数结构
struct LidarParam {
    double unit;  // 单位转换系数
    double res;   // 角度分辨率 (度)
};

// 粒子结构
struct Particle {
    double x;            // 粒子位置 X 坐标
    double y;            // 粒子位置 Y 坐标
    double orientation;  // 粒子方向 (弧度)
    double weight;       // 粒子权重
};

// 将实际坐标转换为地图栅格坐标
bool worldToMap(double x, double y, int &mx, int &my);

// 模拟激光雷达数据，根据粒子位置和方向，生成模拟激光雷达数据
std::vector<double> simulateLidarScan(double x, double y, double orientation);

// 将激光雷达原始数据转换为标准化的范围数据 (米为单位)
std::vector<double> convertRawLidarData(int urgSize, double urgUnit, short *urgData, double urgRes);

// 计算模拟数据与真实数据之间的误差，用于衡量相似度
double computeScanDifference(const std::vector<double> &simScan, const std::vector<double> &realScan);

// 使用重采样方法根据权重更新粒子分布
std::vector<Particle> resampleParticles(const std::vector<Particle> &particles);

// 使用简单运动模型更新粒子分布
void predictParticles(std::vector<Particle> &particles, double speed, double orientation, double odometry);

// 使用蒙特卡洛方法估计小车位置
void estimateCarPose(const QVector<QVector<void*>> &inputParams,
                     const QVector<QVector<void*>> &inputData,
                     double orientation, double speed, double odometry,
                     double &estX, double &estY, double &estOrientation);

#endif