#ifndef PARTICLE_FILTER_NODE_H
#define PARTICLE_FILTER_NODE_H

// ROS lib (roscpp)
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

// ROS Img transport plugin
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// ROS Dynamic reconfigure
#include <dynamic_reconfigure/server.h>

// OpenCV for image processing, loading and display
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

// Boos filesystem to search files in a folder
#include <boost/filesystem.hpp>

// C++ Standard libraries (STL)
#include <vector>

// Num of particles
#include <std_msgs/Int32.h>
#include <quad_amcl/StampedInteger.h>
#include <quad_amcl/StampedArrayFloat.h>

#include "SatelliteManager.h"

// Name spaces
using namespace boost::filesystem;
using namespace cv;
using namespace std;

struct Particle
{
  uint id;
  double x;
  double y;
  double theta;
  double weight;
};

class QuadNetNode
{
private:

  enum PFComunicationState{
    WAITING_EVALUATION,
    READY
  };
  PFComunicationState pfComunicationState=READY;

  enum PFInitMode{
    PF_RANDOM_INI,
    PF_GUESS_INI
  };
  PFInitMode pfInitMode=PF_GUESS_INI;

  // ROS Stuffs
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  ros::Rate r;

  // Pose
  ros::Subscriber subPose;
  bool hasFirstPose=false;

  // Img Rank
  ros::Subscriber subRank;
  image_transport::Subscriber subSonImg;
  bool hasFirstSon=false;

  // Image subscribers...
  image_transport::ImageTransport it;

  image_transport::Publisher pubSatImgs;
  image_transport::Publisher pubSonImg;

  ros::Publisher pubNumParticles; // Current particle count
  ros::Publisher pubPose; // Best pose prediction
  ros::Publisher pubAvgPose5N;
  ros::Publisher pubAvgPose10N;
  ros::Publisher pubAvgPose15N;
  ros::Publisher pubOdom; // Publish odometry

  // Visualization
  image_transport::Publisher pubMapImg;
  image_transport::Publisher pubParticlesView;

  void publishPose(ros::Publisher &pub, const Particle &p, const ros::Time &time);

  void drawParticle(Mat &img,
            const Particle &p,
            const string &txt,
            const Scalar &color,
            int thickness);

  // === Odometry ====

  Odometry odom;


  // ===== Class control ======
  Mat lastSonSmall;
  double lastTime, lastX, lastY, lastYaw;
  double vx=0.0,vy=0.0,vYaw=0.0;
  double sonarRange=50.0, sonarFoV=130.0;
  Mat colors; // Color map (Used by draw functions)

  bool doTest=false;
  bool disableOrientation=true;
  bool removeSatteliteOffset=true;

  // ===== Satellite image stuff =====
  SatelliteManager sat;
  path mapPath;

  // Some time controls
  double sonImgEvalPeriod=10; // secs
  double lastSonEvalTime=0.0;

  double mapPubPeriod=1.0;
  double lastMapPubTime=0.0;

  // Methods
  bool setup();
  
  bool initPF();
  void iniPFRandom();
  void iniPFWithGuess(double x,double y, double theta);

  void sonCallback(const sensor_msgs::ImageConstPtr& msg);

  void poseCallback(const geometry_msgs::PoseStamped &msg);

  void rankCallback(const quad_amcl::StampedArrayFloat &msg);

  void checkSonEvalTimming();

  // Visualisation stuff...
  void updateMapView(const ros::Time &time);
  void updateParticlesView(const ros::Time &time);

  bool showImgMap=false;
  bool showParticlesView=false;

public:

  path imgFolder;

  QuadNetNode();

  int run();

  bool start();

  void getweights();

  void evaluateParticlesObservation(const ros::Time &time);
};

#endif // PARTICLE_FILTER_NODE_H
