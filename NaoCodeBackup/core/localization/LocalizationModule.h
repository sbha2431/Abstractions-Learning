#pragma once

#include <Module.h>
#include <memory/MemoryCache.h>
#include <localization/LocalizationParams.h>
#include <localization/KalmanFilter.h>
#include <localization/KalmanFilter.cpp>
#include <memory/WorldObjectBlock.h>
#include <Eigen/Core>
#define STATEN_SIZE 4
#define OBS_SIZE 2


class ParticleFilter;
class Point2D;
// template <int m,int n>
// class KalmanFilter<m,n>;

class LocalizationModule : public Module {
  public:
    LocalizationModule();
    ~LocalizationModule();
    void specifyMemoryDependency();
    void specifyMemoryBlocks();
    void initSpecificModule();
    void initFromMemory();
    void initKalman(Eigen::Matrix4d Pbar,Eigen::Vector4d xbar);
    void initFromWorld();
    void reInit();
    bool resetFlag;
    void processFrame();
    void loadParams(LocalizationParams params);
    Pose2D displacement;
    void moveBall(const Point2D& position);
    void movePlayer(const Point2D& position, float orientation);
    Eigen::Vector4d propogateGoal(float goalx,Eigen::Vector4d xproj,Eigen::MatrixXd Pproj);
    map<WorldObjectType, std::pair<float,float> > hist_obs;
    std::deque<Pose2D> hist_pos;


  protected:
    MemoryCache cache_;
    TextLogger*& tlogger_;
    LocalizationParams params_;
    ParticleFilter* pfilter_;    
    KalmanFilter<OBS_SIZE,STATEN_SIZE>* kf_;
};
