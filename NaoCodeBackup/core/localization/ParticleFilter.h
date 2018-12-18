#pragma once

#include <math/Pose2D.h>
#include <math/Geometry.h>
#include <memory/MemoryCache.h>
#include <memory/LocalizationBlock.h>
#include <localization/Logging.h>
#include <common/WorldObject.h>
using namespace Eigen;

class ParticleFilter {
  public:
    ParticleFilter(MemoryCache& cache, TextLogger*& tlogger);
    void init(Point2D loc, float orientation);
    void motionModel(Pose2D vel,Particle& p,float dt);
    float measurementModel(Point2D beacon, std::pair<float,float> observation, Particle& p);
    void updateParticles(std::vector<std::pair<float,float>> observations,std::vector<Point2D> beacon_loc,double dt,Pose2D motion);
    void processFrame();
    void resample(std::vector<Particle> parts,std::vector<float> weights);
    void displayParticles();
    const Pose2D& pose(Pose2D curr_est) const;
    const Pose2D& covariance(Pose2D mean_) const;
    inline const std::vector<Particle>& particles() const {
      return cache_.localization_mem->particles;
    }
    double w_slow;
    double w_fast;

  protected:
    inline std::vector<Particle>& particles() {
      return cache_.localization_mem->particles;
    }
    

  private:
    MemoryCache& cache_;
    TextLogger*& tlogger_;

    mutable Pose2D mean_;
    mutable Pose2D cov_;
    mutable bool dirty_;
};
