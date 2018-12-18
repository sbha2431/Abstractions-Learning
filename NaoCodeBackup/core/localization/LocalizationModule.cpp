#include <localization/LocalizationModule.h>
#include <memory/LocalizationBlock.h>
#include <memory/WalkRequestBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/RobotStateBlock.h>
#include <memory/WalkInfoBlock.h>
#include <localization/ParticleFilter.h>
#include <localization/Logging.h>
#include <Eigen/LU>
#include <Eigen/Core>
#include <cmath>
#include <unordered_map>
#define GOAL_LEFT_THRES 800
#define GOAL_RIGHT_THRES -750
#define GOAL_CENTER_THRES 110
#define GOAL_BACK_THRES 230

using namespace Eigen;

// Boilerplate
LocalizationModule::LocalizationModule() : tlogger_(textlogger), pfilter_(new ParticleFilter(cache_, tlogger_)) {
        map<WorldObjectType, std::pair<float,float> > hist_obs = {
        { WO_BEACON_BLUE_YELLOW,  std::pair<float,float>(0.0,0.0) },
        { WO_BEACON_YELLOW_BLUE,   std::pair<float,float>(0.0,0.0) },
        { WO_BEACON_BLUE_PINK,     std::pair<float,float>(0.0,0.0) },
        { WO_BEACON_PINK_BLUE,     std::pair<float,float>(0.0,0.0) },
        { WO_BEACON_PINK_YELLOW,   std::pair<float,float>(0.0,0.0) },
        { WO_BEACON_YELLOW_PINK,   std::pair<float,float>(0.0,0.0) }
        };
        hist_pos.resize(1);//,Pose2D(100,0),Pose2D(100,0),Pose2D(100,0)};
        // hist_pos.resize(10);
}

LocalizationModule::~LocalizationModule() {
  delete pfilter_;
}

// Boilerplate
void LocalizationModule::specifyMemoryDependency() {
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("localization");
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("game_state");
  requiresMemoryBlock("vision_odometry");
  requiresMemoryBlock("vision_walk_request");
}

// Boilerplate
void LocalizationModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(cache_.world_object,"world_objects");
  getOrAddMemoryBlock(cache_.localization_mem,"localization");
  getOrAddMemoryBlock(cache_.frame_info,"vision_frame_info");
  getOrAddMemoryBlock(cache_.robot_state,"robot_state");
  getOrAddMemoryBlock(cache_.game_state,"game_state");
  getOrAddMemoryBlock(cache_.odometry,"vision_odometry");
  getOrAddMemoryBlock(cache_.walk_request,"vision_walk_request");
}


// Load params that are defined in cfglocalization.py
void LocalizationModule::loadParams(LocalizationParams params) {
  params_ = params;
  printf("Loaded localization params for %s\n", params_.behavior.c_str());
}

// Perform startup initialization such as allocating memory
void LocalizationModule::initSpecificModule() {
  reInit();
}

// Initialize the localization module based on data from the LocalizationBlock
void LocalizationModule::initFromMemory() {
  reInit();
}

 void LocalizationModule::initKalman(Matrix4d Pbar,Vector4d xbar) {
  const int n = 4; // Number of states
  const int m = 2; // Number of measurements
  Matrix<double,n,n> Phi; // State transition matrix
  Matrix<double,m,n> H; // Measurement model
  Matrix<double,n,n> Q; // Process noise
  Matrix<double,m,m> R; // Measurement noise

  double dt = 1.0/30.0;
  Phi << 1,0, dt, 0,
       0, 1, 0, dt,
       0, 0, 0.994, 0, 
       0, 0, 0, 0.994;
  H << 1, 0, 0, 0,
     0, 1, 0, 0;

  // Reasonable covariance matrices
  Q << 1.0, 0.0, 0.0, 0.0,
     0.0, 1.0, 0.0, 0.0,
     0.0, 0.0, 1.0, 0.0,
     0.0, 0.0, 0.0, 100.0;

  R << 300,0,
     0,600;
  
  kf_ = new KalmanFilter<OBS_SIZE,STATEN_SIZE>(dt,H,R,Phi, Q,Pbar);
  double t = 0;
  kf_->init(t,xbar);
  resetFlag = false;

}


// Initialize the localization module based on data from the WorldObjectBlock
void LocalizationModule::initFromWorld() {
  reInit();
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];
  pfilter_->init(self.loc, self.orientation);
}

// Reinitialize from scratch
void LocalizationModule::reInit() {
  const int n = STATEN_SIZE; // Number of states
  const int m = OBS_SIZE; // Number of measurements
  Matrix4d Pbar;
  Vector4d xbar;
  pfilter_->init(Point2D(500.0,0.0), 0.0f);
  cache_.localization_mem->player_ = Point2D(0.0,0.0);
  cache_.localization_mem->state = decltype(cache_.localization_mem->state)::Zero();
  cache_.localization_mem->covariance = decltype(cache_.localization_mem->covariance)::Identity();
  // Construct the filter
  Pbar << 1000.0, 0.0, 0.0, 0.0,
     0.0, 1000.0, 0.0, 0.0,
     0.0, 0.0, 1000.0, 0.0, 
     0.0, 0.0, 0.0, 1000.0;
  xbar << 1500.0,0.0,0.0,0.0;
  initKalman(Pbar,xbar);
}

void LocalizationModule::moveBall(const Point2D& position) {
  // Optional: This method is called when the player is moved within the localization
  // simulator window.
}

void LocalizationModule::movePlayer(const Point2D& position, float orientation) {
  // Optional: This method is called when the player is moved within the localization
  // simulator window.
}

// void LocalizationModule::processFrame() {
//   auto& ball = cache_.world_object->objects_[WO_BALL];
//   auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];

//   // Process the current frame and retrieve our location/orientation estimate
//   // from the particle filter
//   pfilter_->processFrame();
//   self.loc = pfilter_->pose().translation;
//   self.orientation = pfilter_->pose().rotation;
//   log(40, "Localization Update: x=%2.f, y=%2.f, theta=%2.2f", self.loc.x, self.loc.y, self.orientation * RAD_T_DEG);
    
//   //TODO: modify this block to use your Kalman filter implementation
//   if(ball.seen) {
//     // Compute the relative position of the ball from vision readings
//     auto relBall = Point2D::getPointFromPolar(ball.visionDistance, ball.visionBearing);

//     // Compute the global position of the ball based on our assumed position and orientation
//     auto globalBall = relBall.relativeToGlobal(self.loc, self.orientation);

//     // Update the ball in the WorldObject block so that it can be accessed in python
//     ball.loc = globalBall;
//     ball.distance = ball.visionDistance;
//     ball.bearing = ball.visionBearing;
//     //ball.absVel = fill this in

//     // Update the localization memory objects with localization calculations
//     // so that they are drawn in the World window
//     cache_.localization_mem->state[0] = ball.loc.x;
//     cache_.localization_mem->state[1] = ball.loc.y;
//     cache_.localization_mem->covariance = decltype(cache_.localization_mem->covariance)::Identity() * 10000;
//   } 
//   //TODO: How do we handle not seeing the ball?
//   else {
//     ball.distance = 10000.0f;
//     ball.bearing = 0.0f;
//   }
// }

void LocalizationModule::processFrame() {
  auto& ball = cache_.world_object->objects_[WO_BALL];
  Pose2D motion = cache_.walk_request->speed_;

  motion.translation *= 300;
  // std:cout << "WalkInfo: "<< motion.rotation <<", translation: "<< motion.translation<<endl;
  // auto motion = Pose2D();//
  // motion.translation.x = 0.0;
  // motion.translation.y = 0.0;
  // motion.rotation = 0.0;
  // std:cout << "Rotation: "<< motion.rotation <<", translation: "<< motion.translation<<endl;
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];



  // Retrieve the robot's current location from localization memory
  // and store it back into world objects
  auto sloc = cache_.localization_mem->player_;

  // self.loc = sloc;
  const int n = STATEN_SIZE; // Number of states
  const int m = OBS_SIZE; // Number of measurements
  double dt = 1.0/15.0; // Time step
  // cache_.odometry->displacement.translation += motion.translation*dt;
  // cache_.odometry->displacement.rotation += motion.rotation*dt;

  self.displacement.translation += motion.translation*dt/3.15;
  self.displacement.rotation += motion.rotation*dt/1.56;

  // std:cout << "Displacement rotation: "<< self.displacement.rotation <<", translation: "<< self.displacement.translation<<endl;
  //TODO: modify this block to use your Kalman filter implementation
  if(ball.seen) {

    Vector2d z;
    Vector4d x;
    double t;
    
    // Compute the relative position of the ball from vision readings
    auto relBall = Point2D::getPointFromPolar(ball.visionDistance, ball.visionBearing);

    z << relBall.x,relBall.y;
    kf_->timeUpdate();
    kf_->measurementUpdate(z);
    x = kf_->state();
    //cout<<"flaval "<< resetFlag << endl;
    //cout<<"velnorm "<< x.tail(2).norm() << endl;
    if(x.tail(2).norm()>20 && !resetFlag && kf_->covariance().norm()<100){
      //cout<<"Resetting vel priors"<<endl;
      Matrix4d TopEye;
      Matrix4d Pbar;
      Vector4d xbar;
      TopEye << 1.0,0.0,0.0,0.0,
        0.0,1.0,0.0,0.0,
        0.0,0.0,0.0,0.0,
        0.0,0.0,0.0,0.0;
      Matrix4d VelVar;
      VelVar<< 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
        0.0,0.0,3000.0,0.0,
        0.0,0.0,0.0,3000.0;
      Pbar = TopEye*kf_->covariance()+VelVar;
      xbar = x;
      initKalman(Pbar,xbar);
      resetFlag = true;
    }
    Vector2d delta = z - x.head(2);
    if(delta.norm()>500){
      //cout<<"Resetting all priors"<<endl;
      Matrix4d Pbar;
      Vector4d xbar;
      Pbar << 5000.0, 0.0, 0.0, 0.0,
              0.0, 5000.0, 0.0, 0.0,
              0.0, 0.0, 5000.0, 0.0, 
              0.0, 0.0, 0.0, 5000.0;
      xbar << z(0),z(1),0,0;
      initKalman(Pbar,xbar);
      resetFlag = false;
    }
    t = kf_->time();
    ball.loc.x = x(0);
    ball.loc.y = x(1);
    ball.relVel.x = x(2);
    ball.relVel.y = x(3);
    //std::cout << "t = " << t << ", " << "Observations: " << z.transpose() << std::endl;
    //cout << "Norm of covariance = " << kf_->Pnorm() << endl;
    // Compute the global position of the ball based on our assumed position and orientation
    auto globalBall = relBall.relativeToGlobal(self.loc, self.orientation);

  // Update the ball in the WorldObject block so that it can be accessed in python
    ball.loc = globalBall;
    ball.distance = ball.visionDistance;
    ball.bearing = ball.visionBearing;

    Vector4d xproj = kf_->state();
    MatrixXd Pproj = kf_->covariance();
    xproj = propogateGoal(GOAL_BACK_THRES,xproj,Pproj);
    //std::cout << "Estimated state at goal : "<<xproj(0)<<", "<<xproj(1)<<endl;
    // std::cout<<"Ball proj "<< xproj(0) << endl;
    if(xproj(0)<GOAL_BACK_THRES ){
        if(xproj(1)<GOAL_LEFT_THRES && xproj(1)>GOAL_CENTER_THRES){
          //std::cout<<"Ball in: left"<<endl;
          ball.dirn=0;}    
        else if (xproj(1)>GOAL_RIGHT_THRES && xproj(1)<-GOAL_CENTER_THRES){
          //std::cout<<"Ball in: right"<<endl;
          ball.dirn=1;}
        else if (xproj(1)>-GOAL_CENTER_THRES && xproj(1)<GOAL_CENTER_THRES){
          //std::cout<<"Ball in: center"<<endl;
          ball.dirn=2;}
        else{ 
            if(xproj(1) > GOAL_LEFT_THRES){
              //std::cout<<"Ball missing left"<<endl;
              ball.dirn=3;}
            else if(xproj(1) < GOAL_RIGHT_THRES){
              //std::cout<<"Ball missing right"<<endl;
              ball.dirn=4;}
            else{std::cout<<"No idea what's happening"<<endl;
            ball.dirn=10;}
          }
    }
    else{
      // std::cout<<"Ball short "<< xproj(0) << endl;
      ball.dirn=5;}

                

    //ball.absVel = fill this in

    // Update the localization memory objects with localization calculations
    // so that they are drawn in the World window
    cache_.localization_mem->state[0] = ball.loc.x;
    cache_.localization_mem->state[1] = ball.loc.y;
    cache_.localization_mem->covariance = decltype(cache_.localization_mem->covariance)::Identity() * 10000;
    } 
    //TODO: How do we handle not seeing the ball?
    else {
    // std::cout<<"ProcessFrame running"<<endl;
    kf_->timeUpdate();
    }
    // Beacons for particle filter
    /*
    // static map<WorldObjectType, std::string, Eigen::vector<double>> beacon_list = {
    //     { WO_BEACON_BLUE_YELLOW,    "BLUE_YELLOW",{2000,1250}},
    //     { WO_BEACON_YELLOW_BLUE,    "YELLOW_BLUE",{2000,1250}},
    //     { WO_BEACON_BLUE_PINK,      "BLUE_PINK",{1500,-1250} },
    //     { WO_BEACON_PINK_BLUE,      "PINK_BLUE",{1500,-1250}},
    //     { WO_BEACON_PINK_YELLOW,    "PINK_YELLOW",{1000,1250}},
    //     { WO_BEACON_YELLOW_PINK,    "YELLOW_PINK",{1000,1250} }
    // };
    */

    // Particle filter
    static map<WorldObjectType, Point2D> beacon_list = {
        { WO_BEACON_BLUE_YELLOW, {-500,1250}},
        { WO_BEACON_YELLOW_BLUE, {-500,-1250}},
        { WO_BEACON_BLUE_PINK,   {-1500,1250}},
        { WO_BEACON_PINK_BLUE,   {-1500,-1250}},
        { WO_BEACON_PINK_YELLOW,  {-1500,550}},
        { WO_BEACON_YELLOW_PINK,  {-1500,-550}}//,
        //{WO_OWN_GOAL,{-1540,0}}
    };

    std::vector<Point2D> beacon_loc;
    std::vector<WorldObject> obs_objects;
    std::vector<std::pair<float,float>> observations;
    for(auto beacon : beacon_list) {
        auto& object = cache_.world_object->objects_[beacon.first];
        if(object.seen && !object.occluded) {
          beacon_loc.push_back(beacon.second);
          observations.push_back(std::make_pair(object.visionDistance,object.visionBearing));
          hist_obs[beacon.first] = std::pair<float,float>(object.visionDistance,object.visionBearing);
        }
        else{
          // if (motion.rotation!=0.0 || motion.translation.x!=0.0 || motion.translation.y!=0.0){
          //   hist_obs[beacon.first] = std::pair<float,float>(0.0,0.0);
          // } // History module turn off for diagnostics
          if (true){
            hist_obs[beacon.first] = std::pair<float,float>(0.0,0.0);
          } // Bypass history block
        } 
      }
  Pose2D curr_est = Pose2D();
  curr_est.translation = self.loc;
  curr_est.rotation = self.orientation;
  std::vector<std::pair<float,float>> observations_in;
  std::vector<Point2D> beacons_loc_in;
  for (auto beacon_h : hist_obs){
    if (beacon_h.second!=std::pair<float,float>(0.0,0.0) && pfilter_->pose(curr_est) != Pose2D()){
      observations_in.push_back(beacon_h.second);
      for (auto bz: beacon_list){
        if (bz.first==beacon_h.first){
          beacons_loc_in.push_back(bz.second);
          //std::cout<<"Observation "<< beacon_h.second.first<<" "<< beacon_h.second.second << " at " << bz.second << endl; 
        }
      }
    }
  }
  pfilter_->updateParticles(observations_in,beacons_loc_in,dt,motion);
  // std::cout<<"Last pos: "<<hist_pos.size()<<endl; 
  hist_pos.pop_front();
  hist_pos.push_back(pfilter_->pose(curr_est));
  Pose2D avg_pos;
  int h_cnt = 0;
  for (auto pos_l=hist_pos.cbegin();pos_l!=hist_pos.cend();++pos_l){
    avg_pos += *pos_l;
    h_cnt++;
  }
  avg_pos /= h_cnt;
  avg_pos.rotation = pfilter_->pose(curr_est).rotation;
  // self.loc = pfilter_->pose(curr_est).translation;
  // self.orientation = pfilter_->pose(curr_est).rotation;
  // cache_.localization_mem->player_ = pfilter_->pose(curr_est).translation;
  // self.sd = pfilter_->covariance(pfilter_->pose(curr_est)).translation;
  // self.sdOrientation = pfilter_->covariance(pfilter_->pose(curr_est)).rotation;
  // std::cout<<"State estimate pos: "<< hist_pos.front() << " State estimate2 : "<< hist_pos.back()<< endl; // <<"\nState2 "<< avg_pos.translation<< " ["<<avg_pos.rotation<<" ]" <<endl;
  self.loc = avg_pos.translation;
  self.orientation = avg_pos.rotation;
  cache_.localization_mem->player_ = avg_pos.translation;
  self.sd = pfilter_->covariance(avg_pos).translation;
  self.sdOrientation = pfilter_->covariance(avg_pos).rotation;
  
  // std::cout<<"State estimate2 pos: "<< sloc  << endl;
  // std::cout<<"State estimate pos: "<< self.loc << "State estimate heading: "<< self.orientation <<" Covariance "<< self.sd<< " ["<<self.sdOrientation<<" ]" <<endl;

}




Vector4d LocalizationModule::propogateGoal(float goalx,Vector4d xproj,MatrixXd Pproj){

  //std::cout<<"vel: "<< xproj.tail(2).norm()<<endl;
  if(xproj(2)<-10){
    //std::cout<<"Start project"<<endl;
    while(xproj(0)>goalx && xproj.tail(2).norm() > 1e-4){
      xproj = kf_->propogateState(1,xproj);
      Pproj = kf_->propogateCovariance(1,Pproj);
    }
    // std::cout<<"Projected Covariance: "<< Pproj(2,2) << endl;
  }
  return xproj;
}
