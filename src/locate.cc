#include "locate.hh"

#include <cmath>
#include <algorithm>
#include <random>
#include <numeric>
#include <QVector>

// 全局地图假设大小为500x500，在主程序中定义并初始化
extern int g_map[500][500];

// 参数设定（可根据实际需要进行调整）
static const int MAP_WIDTH = 500;
static const int MAP_HEIGHT = 500;
// 地图分辨率(米/栅格)
static const double MAP_RESOLUTION = 0.1; 

// 激光雷达参数（此处先固定360度扫描）
static const int LIDAR_ANGLES = 360;
static double LIDAR_ANGLE_INCREMENT = M_PI / 180.0; // 若1度分辨率
static double LIDAR_MAX_RANGE = 30.0; // 最大探测距离30米

// 粒子数量
static const int NUM_PARTICLES = 1000;

static bool g_initialized = false;
static std::vector<Particle> g_particles;

inline bool worldToMap(double x, double y, int &mx, int &my) {
    mx = (int)(x / MAP_RESOLUTION);
    my = (int)(y / MAP_RESOLUTION);
    if (mx < 0 || mx >= MAP_WIDTH || my < 0 || my >= MAP_HEIGHT) {
        return false;
    }
    return true;
}

// 模拟激光雷达数据
std::vector<double> simulateLidarScan(double x, double y, double orientation) {
    std::vector<double> simulated(LIDAR_ANGLES, LIDAR_MAX_RANGE);
    // 尝试从 orientation -180度到 orientation+180度扫描
    double startAngle = orientation - M_PI; 
    for (int i = 0; i < LIDAR_ANGLES; i++) {
        double angle = startAngle + i * LIDAR_ANGLE_INCREMENT; 
        double dist = 0.0;
        const double step = 0.1; // 射线步长(米)
        bool hit = false;
        while (dist < LIDAR_MAX_RANGE && !hit) {
            dist += step;
            double rx = x + dist * cos(angle);
            double ry = y + dist * sin(angle);

            int mx, my;
            if (!worldToMap(rx, ry, mx, my)) {
                // 超出地图范围，视为无障碍
                break;
            }

            if (g_map[my][mx] == 1) {
                // 碰撞到障碍物
                simulated[i] = dist;
                hit = true;
            }
        }
        if (!hit) {
            simulated[i] = LIDAR_MAX_RANGE;
        }
    }
    return simulated;
}

double computeScanDifference(const std::vector<double> &simScan, const std::vector<double> &realScan) {
    double error = 0.0;
    int size = std::min((int)simScan.size(), (int)realScan.size());
    for (int i = 0; i < size; i++) {
        double diff = simScan[i] - realScan[i];
        error += diff * diff; 
    }
    return error;
}

// 重采样粒子
std::vector<Particle> resampleParticles(const std::vector<Particle> &particles) {
    std::vector<Particle> newParticles;
    newParticles.resize(particles.size());

    double sumW = 0.0;
    for (auto &p: particles) sumW += p.weight;
    // 正规化
    std::vector<Particle> normParticles = particles;
    for (auto &p: normParticles) {
        p.weight = p.weight / sumW;
    }

    std::vector<double> cdf(particles.size());
    cdf[0] = normParticles[0].weight;
    for (int i = 1; i < (int)particles.size(); i++) {
        cdf[i] = cdf[i-1] + normParticles[i].weight;
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    for (int i = 0; i < (int)particles.size(); i++) {
        double r = dist(gen);
        int idx = (int)(std::lower_bound(cdf.begin(), cdf.end(), r) - cdf.begin());
        if (idx >= (int)particles.size()) idx = (int)particles.size() - 1;
        newParticles[i] = normParticles[idx];
        newParticles[i].weight = 1.0 / particles.size();
    }

    return newParticles;
}

// 简单运动模型预测
void predictParticles(std::vector<Particle> &particles, double speed, double orientation, double odometry) {
    double dt = 1.0; 
    double dx = speed * std::cos(orientation) * dt;
    double dy = speed * std::sin(orientation) * dt;
    double dtheta = 0.0; // 可根据实际数据进行修正

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> noise_pos(0.0, 0.1); 
    std::normal_distribution<double> noise_ang(0.0, 0.01);

    for (auto &p: particles) {
        p.x += dx + noise_pos(gen);
        p.y += dy + noise_pos(gen);
        p.orientation += dtheta + noise_ang(gen);
    }
}

// 将激光雷达数据转换为米，并根据分辨率设置LIDAR_ANGLE_INCREMENT
std::vector<double> convertRawLidarData(int urgSize, double urgUnit, short *urgData, double urgRes) {
    // 假设 urgRes 是激光雷达的角分辨率(度)，将其转换为弧度
    LIDAR_ANGLE_INCREMENT = urgRes * M_PI / 180.0;

    // 假设数据大小为urgSize，若与LIDAR_ANGLES不匹配，需要插值或截断
    std::vector<double> ranges;
    for (int i = 0; i < urgSize; i++) {
        double dist_m = urgData[i] * urgUnit; // urgUnit 将数据从原单位转换为米
        if (dist_m > LIDAR_MAX_RANGE) {
            dist_m = LIDAR_MAX_RANGE;
        }
        ranges.push_back(dist_m);
    }

    // 若数据多于LIDAR_ANGLES，截断
    if ((int)ranges.size() > LIDAR_ANGLES) {
        ranges.resize(LIDAR_ANGLES);
    }
    // 若数据少于LIDAR_ANGLES，用最大值填充
    else if ((int)ranges.size() < LIDAR_ANGLES) {
        ranges.insert(ranges.end(), LIDAR_ANGLES - ranges.size(), LIDAR_MAX_RANGE);
    }

    return ranges;
}

void estimateCarPose(const QVector<QVector<void*>> &inputParams,
                     const QVector<QVector<void*>> &inputData,
                     double orientation, double speed, double odometry,
                     double &estX, double &estY, double &estOrientation)
{
    // 从 inputData[0] 和 inputParams[0] 中获取最新的激光数据和参数
    // 假设 inputData[0].front(), inputParams[0].front() 为当前帧数据(根据实际情况可能需要 .last())
    if (inputData.size() == 0 || inputData[0].size() == 0
        || inputParams.size() == 0 || inputParams[0].size() == 0) {
        // 无数据，给出默认值
        estX = 0.0;
        estY = 0.0;
        estOrientation = orientation;
        return;
    }

    // 获取激光雷达数据
    LidarData *currentLidar = (LidarData*)(inputData[0].front());
    int urgSize = currentLidar->datasize;
    short *urgData = currentLidar->data;

    // 获取激光雷达参数
    // 假设 inputParams[0].front() 指向一个包含 unit 和 res 的参数结构
    struct LidarParam {
        double unit;
        double res;
    };
    LidarParam *param = (LidarParam*)(inputParams[0].front());
    double urgUnit = param->unit;
    double urgRes = param->res;

    // 将数据转换为标准格式
    std::vector<double> realScan = convertRawLidarData(urgSize, urgUnit, urgData, urgRes);

    // 初始化粒子
    if (!g_initialized) {
        g_initialized = true;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> xdist(0.0, MAP_WIDTH*MAP_RESOLUTION);
        std::uniform_real_distribution<double> ydist(0.0, MAP_HEIGHT*MAP_RESOLUTION);
        std::uniform_real_distribution<double> odist(-M_PI, M_PI);

        g_particles.resize(NUM_PARTICLES);
        for (auto &p : g_particles) {
            p.x = xdist(gen);
            p.y = ydist(gen);
            p.orientation = odist(gen);
            p.weight = 1.0 / NUM_PARTICLES;
        }
    }

    // 预测步骤
    predictParticles(g_particles, speed, orientation, odometry);

    // 计算粒子权重
    for (auto &p : g_particles) {
        std::vector<double> simScan = simulateLidarScan(p.x, p.y, p.orientation);
        double error = computeScanDifference(simScan, realScan);
        p.weight = 1.0 / (1.0 + error); 
    }

    // 重采样
    g_particles = resampleParticles(g_particles);

    // 加权平均
    double sumx=0.0, sumy=0.0, sumo=0.0, sumw=0.0;
    for (auto &p: g_particles) {
        sumx += p.x * p.weight;
        sumy += p.y * p.weight;
        sumo += p.orientation * p.weight;
        sumw += p.weight;
    }

    if (sumw > 0) {
        estX = sumx / sumw;
        estY = sumy / sumw;
        estOrientation = sumo / sumw;
    } else {
        // 异常情况
        estX = 0.0;
        estY = 0.0;
        estOrientation = orientation;
    }
}