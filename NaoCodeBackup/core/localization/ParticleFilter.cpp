#include <localization/ParticleFilter.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <common/Random.h>
#include <math.h>

#define R_dist 150.0
#define R_ang 0.10
#define P_xy 250
#define FIELD_X 2000
#define FIELD_Y 2500
#define num_particles 500
#define alpha_slow 0.8
#define alpha_fast 0.9
#define Q_xy_noise 20.0//100.0
#define Q_t_noise 0.25//1.5

ParticleFilter::ParticleFilter(MemoryCache& cache, TextLogger*& tlogger) 
  : cache_(cache), tlogger_(tlogger), dirty_(true) {
}

void ParticleFilter::init(Point2D loc, float orientation) {
  // std:cout << "Initializing filter "<< endl;
  particles().resize(num_particles);
  for(auto& p : particles()) {
    // motionModel(ctr_in,p);
    // measurementModel(z,p);
    // updateWeight();
    p.x = Random::inst().sampleU() * 2*FIELD_X-FIELD_X; //static_cast<int>(frame * 5), 250);
    p.y = Random::inst().sampleU() * 2*FIELD_Y-FIELD_Y; // 0., 250);
    p.t = Random::inst().sampleU() *2*M_PI-M_PI;   // Uniform distribution for all bearing angles
    p.w = 1.0/num_particles;
  }
  mean_.translation = loc;
  // cov_.translation = P_xy;
  mean_.rotation = orientation;
  // cov_.rotation = 1.0;
  w_slow =0;
  w_fast =0;
}

void ParticleFilter::processFrame() {
  // Indicate that the cached mean needs to be updated
  dirty_ = true;

  // std:cout << "P running "<< endl;
  // Retrieve odometry update - how do we integrate this into the filter?
  // const auto& disp = cache_.odometry->displacement;
  // log(41, "Updating particles from odometry: %2.f,%2.f @ %2.2f", disp.translation.x, disp.translation.y, disp.rotation * RAD_T_DEG);
  
  // Generate random particles for demonstration
  particles().resize(num_particles);
  for(auto& p : particles()) {
    // motionModel(ctr_in,p);
    // measurementModel(z,p);
    // updateWeight();
    p.x = Random::inst().sampleU() * 2*FIELD_X-FIELD_X; //static_cast<int>(frame * 5), 250);
    p.y = Random::inst().sampleU() * 2*FIELD_Y-FIELD_Y; // 0., 250);
    p.t = Random::inst().sampleU() *2*M_PI-M_PI;   // Uniform distribution for all bearing angles
    p.w = 1.0/num_particles;
  }
}

void ParticleFilter::motionModel(Pose2D vel,Particle& p,float dt){
  // std::cout <<"before " << p.t<<endl;
  p.x += (vel.translation.x*cos(p.t)+vel.translation.y*sin(p.t))*dt+Q_xy_noise*Random::inst().sampleN();
  p.y += (vel.translation.x*sin(p.t)+vel.translation.y*cos(p.t))*dt+Q_xy_noise*Random::inst().sampleN();
  p.t += fmod((vel.rotation*dt+Q_t_noise*Random::inst().sampleN()),2.0*M_PI);
  // std::cout <<"after " << p.t<<endl;
}

float ParticleFilter::measurementModel(Point2D beacon, std::pair<float,float> observation, Particle& p){
  float totalmass = 0;
  float probpos = 0;
  Point2D p_loc = Point2D(p.x,p.y);
  float r_p = p_loc.getDistanceTo(beacon);
  float prob_loc = exp(-1.0/pow(R_dist,2)*pow(r_p - observation.first,2));
  AngRad p_bearing = p_loc.getBearingTo(beacon,p.t);
  float prob_ang;
  if(abs(p_bearing)<M_PI){
    prob_ang = exp(-1.0/pow(R_ang,2)*pow(p_bearing-observation.second,2));
  }
  else{
    p.t *= -1; // Flip the sign
    p_bearing = p_loc.getBearingTo(beacon,p.t);
    prob_ang = exp(-1.0/pow(R_ang,2)*pow(p_bearing-observation.second,2));
  }
  // std::cout << "Prob loc "<< prob_loc<<", Proc ang "<<prob_ang<< endl;
  return prob_loc*prob_ang;
}

void ParticleFilter::updateParticles(std::vector<std::pair<float,float>> observations,std::vector<Point2D> beacon_loc,double dt,Pose2D motion){
   dirty_ = true;
   // displayParticles();
   double weight_sum;
   if (observations.size()>0){weight_sum = 0.0;}else{weight_sum = 1.0;}
   for(auto& p : particles()){
      //std:cout << "P at"<< p.x<<", "<<p.y<<", "<<p.t<<", weight "<<p.w<< endl;
      motionModel(motion,p,dt);
      float obj_prob = 1.0;
      for(int i=0; i < observations.size(); i++){
        obj_prob *= measurementModel(beacon_loc[i],observations[i],p);
      }
      // for(auto observation : observations) {
      //   obj_prob *= measurementModel(observation.second,observation.first,p);
      // }

      if(obj_prob!=1.0){
        weight_sum+=obj_prob;
        p.w = obj_prob;
      }
    }
    if (observations.size()>0){
      w_fast += alpha_fast*(weight_sum/num_particles-w_fast);
      w_slow += alpha_slow*(weight_sum/num_particles-w_slow);
      std::vector<Particle> temp_particles;
      std::vector<float> temp_weights;
      // std::cout << weight_sum<<endl;
      for(auto& p : particles()){
        if(weight_sum!=0.0){p.w /= weight_sum;}
        Particle t_p;
        t_p.x = p.x;t_p.y=p.y;t_p.t=p.t;t_p.w = p.w; 
        temp_particles.push_back(t_p);
        temp_weights.push_back(p.w);


      }
      pose(mean_);
      resample(temp_particles,temp_weights);
    }
    else{
      pose(mean_);
    }


    // std::cout << "End frame"<< endl; 
}

void ParticleFilter::displayParticles(){
  int i=0;
  for (auto& p : particles()){
    std:cout << "Patricle "<<i++<<" at "<< p.x<<", "<<p.y<<", "<<", "<<p.t<<": Weight "<<p.w<< endl;
  }
}


void ParticleFilter::resample(std::vector<Particle> parts,std::vector<float> weights){
  double weight_sum = 0;
  // std::cout << w_fast/w_slow<<endl;
  // std::cout << "Mean translation" <<mean_.translation <<" rotation "<<mean_.rotation<<endl;
  // std::cout << "Covariance translation" <<cov_.translation <<" rotation "<<cov_.rotation<<endl; 
  int i=0;
  for(auto& p : particles()){
    // Probability if for new random pose
    Particle temp_part;
    if(Random::inst().sampleB(std::max(0.0,1.0-w_fast/w_slow))){
      if(Random::inst().sampleB(0.01)){
        temp_part.x = Random::inst().sampleU(-2010.0,500.0);
        temp_part.y = Random::inst().sampleU(-1000.0,1000.0);
        temp_part.t = fmod(Random::inst().sampleN(mean_.rotation,float(cov_.rotation)),2.0*M_PI);
        temp_part.w = 1.0/float(num_particles);
      }
      else{
        temp_part.x = Random::inst().sampleN(mean_.translation.x,float(Q_xy_noise));
        temp_part.y = Random::inst().sampleN(mean_.translation.y,float(Q_xy_noise));
        temp_part.t = fmod(Random::inst().sampleN(mean_.rotation,float(Q_t_noise)),2.0*M_PI);
        temp_part.w = 1.0/float(num_particles);
      }
      i++;
    }
    else{
      temp_part = parts[Random::inst().sampleD(weights)];
    }
    // Otherwise do a sample from our distribution
    // int rand_i =;
    // std::cout << "Sample at "<<rand_i <<" Other sample would be at "<<distribution(gen) << endl;
    p.x = temp_part.x; p.y = temp_part.y; p.t = temp_part.t; p.w=1.0/float(num_particles);
    weight_sum += p.w;
  }
  // std::cout << "Resamples " <<i <<endl;
}

const Pose2D& ParticleFilter::pose(Pose2D curr_est) const {
  if(dirty_) {
    // Compute the mean pose estimate
    mean_ = curr_est; // Assign as the old one. Then update
    Pose2D mean_hold = Pose2D();
    using T = decltype(mean_hold.translation);
    for(const auto& p : particles()) {
      mean_hold.translation += T(p.w*p.x,p.w*p.y);
      mean_hold.rotation += p.w*p.t;
    }
    if(particles().size() > 0)
      mean_hold /= static_cast<float>(particles().size());
    dirty_ = false;
    if (mean_hold.rotation!=0.0 || mean_hold.translation.x!=0.0 || mean_hold.translation.y!=0.0){
      mean_ = mean_hold;
    }
  }
  return mean_;
}

const Pose2D& ParticleFilter::covariance(Pose2D mean_) const {
    cov_ = Pose2D();
    for(const auto& p : particles()) {
      cov_.translation.x += p.w*pow(p.x-mean_.translation.x,2);
      cov_.translation.y += p.w*pow(p.y-mean_.translation.y,2);
      cov_.rotation += p.w*pow(p.t-mean_.rotation,2);
    }
    if(particles().size() > 0) {
      cov_ /= static_cast<float>(particles().size());
      cov_.translation.x = sqrt(cov_.translation.x);
      cov_.translation.y = sqrt(cov_.translation.y);
      cov_.rotation = sqrt(cov_.rotation);
    };
  return cov_;
}