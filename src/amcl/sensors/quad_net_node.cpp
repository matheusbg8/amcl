#include "amcl/sensors/quad_net_node.h"
//#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <random>

void QuadNetNode::drawParticle(Mat &img,
                                      const Particle &p,
                                      const string &txt,
                                      const Scalar &color,
                                      int thickness)
{
  // Get the sonar field of view polygon on satellite image
  int nPts = 15;
  Point pts[nPts];

  sat.getSonarPolyOnImg(Point2d(p.x,p.y),p.theta*180.0/M_PI,
                        pts,nPts,sonarRange,sonarFoV);

  int npts[] = {nPts};
  const Point* ppt[1] = { pts };

  polylines(img,
            ppt,npts,
            1,true,
            color,thickness);
  if(!txt.empty())
    putText(img,txt,
            pts[0],FONT_HERSHEY_DUPLEX,
            3.0,color,9);
}

bool QuadNetNode::setup()
{
  // Parameters
  string map_path;

  if(!pnh.getParam("map_path",map_path))
  {
    ROS_ERROR("Parameter ~map_path not defined!");
    return false;
  }
  mapPath = map_path;

  if(!sat.loadMap(mapPath.string()))
  {
    ROS_ERROR_STREAM("Could not load map " << map_path);
    return false;
  }

  if(removeSatteliteOffset)
    sat.removeOffset();

  // Topic Subscription
  subSonImg = it.subscribe("/son",1,
        &QuadNetNode::sonCallback,this);

  subPose = nh.subscribe("/pose",1,
        &QuadNetNode::poseCallback,this);

  subRank = nh.subscribe("/rank",1,
        &QuadNetNode::rankCallback,this);

  // Topic advertisement
  pubSatImgs = it.advertise("/sat_crop",300); // Particles view
  pubSonImg = it.advertise("/son_small",3); // Sonar image
  pubNumParticles = nh.advertise<quad_amcl::StampedInteger>("/n_particles",1);

  pubPose = nh.advertise<geometry_msgs::PoseStamped>("/pf_pose",1);
  pubAvgPose5N = nh.advertise<geometry_msgs::PoseStamped>("/pf_avg5_pose",1);
  pubAvgPose10N = nh.advertise<geometry_msgs::PoseStamped>("/pf_avg10_pose",1);
  pubAvgPose15N = nh.advertise<geometry_msgs::PoseStamped>("/pf_avg15_pose",1);

  pubOdom = nh.advertise<geometry_msgs::PoseStamped>("/odom_pose",1);

  pubMapImg = it.advertise("/map_img",1);
  pubParticlesView = it.advertise("/particles_view",1);

  return true;
}

void QuadNetNode::sonCallback(const sensor_msgs::ImageConstPtr &msg)
{
  ros::Time time_begin = ros::Time::now();

  if(!pf->initialized())
  {
    initPF();
    return ; // PF was not running
  }

  // Check comunication status
  checkSonEvalTimming();

  // Check timestamp
  double time = msg->header.stamp.toSec();

  // Only process image after specified time
  if(time - lastSonEvalTime < sonImgEvalPeriod)
    return ;

  // If the neural network is ready to a new evaluation
  if(pfComunicationState != READY)
    return; // Not ready to evaluate this acoustic image

  try
  {
    lastSonSmall = cv_bridge::toCvShare(msg, "mono8")->image;
//    son = cv_bridge::toCvShare(msg, "bgr8")->image;
//    cv::imshow("View son", lastSonSmall);
//    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }

  // Resize son to 256 columns 128 rows
  resize(lastSonSmall,lastSonSmall,Size(256,128));

  // Only process if the image has enought features
  if(countNonZero(lastSonSmall) < 750)
    return ; // Not enought features to match, skip the image

  cvtColor(lastSonSmall,lastSonSmall,CV_GRAY2BGR);

  // Update last processed image timestamp
  lastSonEvalTime=time;

  evaluateParticlesObservation(msg->header.stamp); // Publish msg using sonar timestamp

  ros::Time time_end = ros::Time::now();
  ros::Duration duration = time_end - time_begin;
  ROS_INFO("Soncalback %lf secs", duration.toSec());
}

void QuadNetNode::rankCallback(const quad_amcl::StampedArrayFloat &msg)
{
  ros::Time time_begin = ros::Time::now();

  pf->updateWeights(msg.data);
  pf->resample(sigma_pos);

  pfComunicationState = READY;
  updateParticlesView(msg.header.stamp);

  ros::Time time_end = ros::Time::now();
  ros::Duration duration = time_end - time_begin;
  ROS_INFO("RankCalback %lf secs", duration.toSec());

  //  ROS_INFO("PF Ready");
}

void QuadNetNode::checkSonEvalTimming()
{
  static int waitingIterationCount=0;

  // If it is waiting for the particles evaluation (communication)
  if(pfComunicationState == WAITING_EVALUATION)
  {
    waitingIterationCount++;
    // If it is waiting for too long
    if(waitingIterationCount>40)
    {
      // Stop to wait...
      pfComunicationState = READY;
      waitingIterationCount=0;
    }
  }
  else
    waitingIterationCount=0;
}

void QuadNetNode::updateMapView(const ros::Time &time)
{
  Mat screen = sat.getMapImg();
  string txt;
  const vector<Particle> &ps = pf->getParticles();

  // Particles Color version
//  double minW, maxW;
//  pf->getMinMaxWieght(minW,maxW);

  // Draw particles
  for(uint i = 0; i < ps.size();i++)
  {
    const Particle &p = ps[i];

    drawParticle(screen,
                 p,txt,
                 Scalar(255,0,255),2);

    // Particles Color version
//    double normW=1.0;
//    if(maxW != minW)
//      normW=(p.weight-minW)/(maxW-minW);
//    const Vec3b& cPix = colors.at<Vec3b>(0,normW*255);

//    drawParticle(screen,
//                 p,txt,
//                 Scalar(cPix[0],cPix[1],cPix[2]),2);
  }

  // Draw Odom
  {
    Particle p;
    odom.predict(time.toSec(),
                 p.x,p.y,p.theta);
    txt="ODOM";

    drawParticle(screen,
                 p,txt,
                 Scalar(128,255,128),6);
  }

  // Draw GT
  {
    Particle p;
    p.x = lastX; p.y = lastY;
    p.theta = lastYaw;
    txt="GT";

    drawParticle(screen,
                 p,txt,
                 Scalar(0,255,255),12);
  }

  // Draw best particle
  {
    Particle p;
//    pf->getBestParticle(p);
    pf->getBestAverageParticle(120,p);

    drawParticle(screen,
             p,format("PF_Best"),
             Scalar(0,0,0),12);
  }

  // Resize img
  double s = 900.0/screen.cols;
  resize(screen,screen,Size(),s,s);

  // Show img on screen
  if(showImgMap)
  {
    imshow("Test", screen);
    waitKey(10);
  }

  // Publish msg
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", screen).toImageMsg();
  msg->header.stamp = time;
  pubMapImg.publish(msg);

}

void QuadNetNode::updateParticlesView(const ros::Time &time)
{
  Mat screen(600,800,CV_8UC3, Scalar(0,0,0));

  vector<Particle> ps = pf->getParticles();
  sort(ps.begin(),ps.end(),descendingParticles);

  uint t = ps.size(),n,m;
  {
    double r = double(screen.cols)/double(screen.rows),
         ri = 256.0/128.0,
        rf= r/ri,
        dm=sqrt(t/rf),// +1, // Plus one line to fit son img
        dn = rf*dm;
        m = int(ceil(dm));
        n = int(ceil(dn));
  }

  uint cellW=screen.cols/n,
       cellH=screen.rows/m,
       k=0;

  Mat img;

  for(uint i = 0; i < m && k < t; i++)
  {
    for(uint j = 0; j < n && k < t; j++)
    {
      const Particle &p = ps[k++];
      uint x=j*cellW,y=i*cellH;

      img = sat.cropSonarFoV(Point2d(p.x,p.y),
                             p.theta*180/M_PI,
                             sonarRange,
                             130.0);

      if(img.empty())
      {
        ROS_ERROR("Could not crop sat img");
        continue;
      }

      resize(img,img,Size(cellW,cellH));
      putText(img,format("%.1f",p.weight*100.0),
              Point(img.cols/2-15*3,img.rows-15),
              FONT_HERSHEY_DUPLEX,
              1.0,Scalar(255,0,255),1);

      img.copyTo(screen(Rect(x,y,cellW,cellH)));
    }
  }

  if(showParticlesView)
  {
    imshow("Particles", screen);
    waitKey(10);
  }

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", screen).toImageMsg();

  msg->header.stamp = time;
  pubParticlesView.publish(msg);
}

QuadNetNode::QuadNetNode():
    pnh("~"),
    it(nh),
    r(4.0),
    colors(1,256,CV_8UC1)
{
  for(uint i =0 ;i < 256; i++)
    colors.at<uchar>(0,i) = i;

  applyColorMap(colors,colors,COLORMAP_JET);
}


bool QuadNetNode::start()
{
  // Setup ROS stuff
  // Load maps
  if(!setup()) return false;
//  ROS_INFO_STREAM("Setup completed!");

  initPF();

  ros::spin();

  return true;
}

void QuadNetNode::getweights()
{
  quad_amcl::StampedArrayFloat::ConstPtr msg = ros::topic::waitForMessage<quad_amcl::StampedArrayFloat>
      ("rank",nh,ros::Duration(60));
  if(msg != nullptr)
  {

  }

}

void QuadNetNode::evaluateParticlesObservation(const ros::Time &time)
{
  if(pfComunicationState != READY || lastSonSmall.empty())
  {
    ROS_WARN("ParticleFilter not ready to send msgs!");
    return;
  }

  const vector<Particle> &ps = pf->getParticles();

  // Predict particle pose on this instant
  pf->prediction(time.toSec(),sigma_pos);

  // Sending num. of particles
  {
    quad_amcl::StampedInteger msg;
    msg.header.stamp = time;
    msg.data = ps.size();
    pubNumParticles.publish(msg);
  }

  // Publishing son image
  {
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
                                                   "bgr8", lastSonSmall).toImageMsg();
    msg->header.stamp = time;
    pubSonImg.publish(msg);
  }

  // Publishing sat imgs (Particles)
  for(int i=0; i < ps.size(); i++)
  {
    const Particle &p = ps[i];


    Mat img = sat.cropSonarFoV(Point2d(p.x,p.y),
                               p.theta*180/M_PI,
                               sonarRange,
                               130.0);
    if(img.empty())
    {
      ROS_ERROR("Could not crop sat img");
      continue;
    }

    resize(img,img,Size(256,128));

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    msg->header.stamp = time;
    pubSatImgs.publish(msg);
  }


  pfComunicationState = WAITING_EVALUATION;
  ROS_INFO("PF Waiting img evaluation!");

}
