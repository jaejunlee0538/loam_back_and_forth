#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/shared_ptr.hpp>

const double PI = 3.1415926;
const double rad2deg = 180 / PI;
const double deg2rad = PI / 180;

double time_first_scan;
double dt_scan;
double time_sweep_start;
bool systemInited = false;

double time_current_scan = 0;
double time_last_scan = 0;

int laserRotDir = 1;
double laserAngleLast = 0;
double laserAngleCur = 0;

#define SKIP_FRAME_COUNT 3
int skipFrameCount = 0;

pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudFeaturesCurrent(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudLessExtreCur(new pcl::PointCloud<pcl::PointXYZHSV>());

sensor_msgs::PointCloud2 laserCloudExtreCur2;
sensor_msgs::PointCloud2 laserCloudLast;

boost::shared_ptr<ros::Publisher> pubLaserCloudExtreCurPointer;
boost::shared_ptr<ros::Publisher> pubLaserCloudLastPointer;

int cloudSortInd[800];
int cloudNeighborPicked[800];

int imuPointerFront = 0;
int imuPointerLast = -1;
const int IMU_BUFFER_SIZE = 400;
bool imuInited = false;

double roll_imu_start_sweep, pitch_imu_start_sweep, yaw_imu_start_sweep;
double roll_imu_current, pitch_imu_current, yaw_imu_current;

double vx_imu_start_sweep, vy_imu_start_sweep, vz_imu_start_sweep;
double x_imu_start_sweep, y_imu_start_sweep, z_imu_start_sweep;
double vx_imu_current, vy_imu_current, zx_imu_current;
double x_imu_current, y_imu_current, z_imu_current;

double imuShiftFromStartXCur, imuShiftFromStartYCur, imuShiftFromStartZCur;
double imuVeloFromStartXCur, imuVeloFromStartYCur, imuVeloFromStartZCur;

double imuTime[IMU_BUFFER_SIZE] = {0};
double imuRoll[IMU_BUFFER_SIZE] = {0};
double imuPitch[IMU_BUFFER_SIZE] = {0};
double imuYaw[IMU_BUFFER_SIZE] = {0};

double imuAccX[IMU_BUFFER_SIZE] = {0};
double imuAccY[IMU_BUFFER_SIZE] = {0};
double imuAccZ[IMU_BUFFER_SIZE] = {0};

double vx_from_imu[IMU_BUFFER_SIZE] = {0};
double vy_from_imu[IMU_BUFFER_SIZE] = {0};
double vz_from_imu[IMU_BUFFER_SIZE] = {0};

double x_from_imu[IMU_BUFFER_SIZE] = {0};
double y_from_imu[IMU_BUFFER_SIZE] = {0};
double z_from_imu[IMU_BUFFER_SIZE] = {0};

void TransformToStartIMU(pcl::PointXYZHSV *p)
{
  /**********************Transform it inertial frame********************************/
  double x1 = cos(roll_imu_current) * p->x - sin(roll_imu_current) * p->y;
  double y1 = sin(roll_imu_current) * p->x + cos(roll_imu_current) * p->y;
  double z1 = p->z;

  double x2 = x1;
  double y2 = cos(pitch_imu_current) * y1 - sin(pitch_imu_current) * z1;
  double z2 = sin(pitch_imu_current) * y1 + cos(pitch_imu_current) * z1;

  double x3 = cos(yaw_imu_current) * x2 + sin(yaw_imu_current) * z2;
  double y3 = y2;
  double z3 = -sin(yaw_imu_current) * x2 + cos(yaw_imu_current) * z2;
  /*********************************************************************************/
  /************Transform it to start sweep coordinate*******************************/
  double x4 = cos(yaw_imu_start_sweep) * x3 - sin(yaw_imu_start_sweep) * z3;
  double y4 = y3;
  double z4 = sin(yaw_imu_start_sweep) * x3 + cos(yaw_imu_start_sweep) * z3;

  double x5 = x4;
  double y5 = cos(pitch_imu_start_sweep) * y4 + sin(pitch_imu_start_sweep) * z4;
  double z5 = -sin(pitch_imu_start_sweep) * y4 + cos(pitch_imu_start_sweep) * z4;

  p->x = cos(roll_imu_start_sweep) * x5 + sin(roll_imu_start_sweep) * y5 + imuShiftFromStartXCur;
  p->y = -sin(roll_imu_start_sweep) * x5 + cos(roll_imu_start_sweep) * y5 + imuShiftFromStartYCur;
  p->z = z5 + imuShiftFromStartZCur;
  /*********************************************************************************/
}

/*
 *  publish feature point cloud(growing until sweep ends) with topic name of /laser_cloud_extre_cur
 *  publish last point cloud(feature+non-feature, in camera coordinates(when last sweep start)) with topic name of /laser_cloud_last
 *
 *  with 13.36 Hz
 */
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr&cloud_msg)
{
  /*
   * cloud_msg is a point cloud already transformed into camera coordinates.
   * (this laser scan implies encoder information)
   */
  if (!systemInited) {
    time_first_scan = cloud_msg->header.stamp.toSec();
    imuPointerFront = (imuPointerLast + 1) % IMU_BUFFER_SIZE;
    systemInited = true;
  }

  time_last_scan = time_current_scan;
  time_current_scan = cloud_msg->header.stamp.toSec();
  dt_scan = time_current_scan - time_first_scan;
  pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud_msg, *laserCloudIn);
  int cloudSize = laserCloudIn->points.size();

  pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZHSV>(cloudSize, 1));
  for (int i = 0; i < cloudSize; i++) {
    laserCloud->points[i].x = laserCloudIn->points[i].x;
    laserCloud->points[i].y = laserCloudIn->points[i].y;
    laserCloud->points[i].z = laserCloudIn->points[i].z;
    laserCloud->points[i].h = dt_scan;
    laserCloud->points[i].v = 0;
    cloudSortInd[i] = i;
    cloudNeighborPicked[i] = 0;
  }

  bool newSweep = false;
  laserAngleLast = laserAngleCur;
  laserAngleCur = atan2(laserCloud->points[cloudSize - 1].y - laserCloud->points[0].y,
                        laserCloud->points[cloudSize - 1].x - laserCloud->points[0].x);

  if (laserAngleLast > 0 && laserRotDir == 1 && laserAngleCur < laserAngleLast) {
    laserRotDir = -1;
    newSweep = true;
  } else if (laserAngleLast < 0 && laserRotDir == -1 && laserAngleCur > laserAngleLast) {
    laserRotDir = 1;
    newSweep = true;
  }

  /********Hold accumulated point cloud(features + non-feature points) during last sweep in [laserCloudLast]******************************/
  /*
   * Last 4 points represent transformation between start and end of the sweep.
   */

  if (newSweep) {
    time_sweep_start = time_last_scan - time_first_scan;
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZHSV>(4, 1));
    imuTrans->points[0].x = pitch_imu_start_sweep;
    imuTrans->points[0].y = yaw_imu_start_sweep;
    imuTrans->points[0].z = roll_imu_start_sweep;
    imuTrans->points[0].v = 10;

    imuTrans->points[1].x = pitch_imu_current;
    imuTrans->points[1].y = yaw_imu_current;
    imuTrans->points[1].z = roll_imu_current;
    imuTrans->points[1].v = 11;

    //imu relative position(wrt begin of sweep) at end of the sweep
    imuTrans->points[2].x = imuShiftFromStartXCur;
    imuTrans->points[2].y = imuShiftFromStartYCur;
    imuTrans->points[2].z = imuShiftFromStartZCur;
    imuTrans->points[2].v = 12;

    //imu velocity at the end of the sweep
    imuTrans->points[3].x = imuVeloFromStartXCur;
    imuTrans->points[3].y = imuVeloFromStartYCur;
    imuTrans->points[3].z = imuVeloFromStartZCur;
    imuTrans->points[3].v = 13;

    *laserCloudFeaturesCurrent += *laserCloudLessExtreCur;
    pcl::toROSMsg(*laserCloudFeaturesCurrent + *imuTrans, laserCloudLast);
    laserCloudLast.header.stamp = ros::Time().fromSec(time_last_scan);
    laserCloudLast.header.frame_id = "/camera";
    laserCloudFeaturesCurrent->clear();
    laserCloudLessExtreCur->clear();

    roll_imu_start_sweep = roll_imu_current;
    pitch_imu_start_sweep = pitch_imu_current;
    yaw_imu_start_sweep = yaw_imu_current;

    vx_imu_start_sweep = vx_imu_current;
    vy_imu_start_sweep = vy_imu_current;
    vz_imu_start_sweep = zx_imu_current;

    x_imu_start_sweep = x_imu_current;
    y_imu_start_sweep = y_imu_current;
    z_imu_start_sweep = z_imu_current;
  }
  /*****************************************************************************************/

  /*******************Has Effect only when imu is active**********************************/
  //<editor-fold desc="It's running always, but without imu data it has no effect.">
  /*
   * Without imu updated, these 6 terms have no effect(remaining 0 always).
   */
  roll_imu_current = 0; pitch_imu_current = 0; yaw_imu_current = 0;
  vx_imu_current = 0; vy_imu_current = 0; zx_imu_current = 0;
  x_imu_current = 0; y_imu_current = 0; z_imu_current = 0;

  if (imuPointerLast >= 0) {
    while (imuPointerFront != imuPointerLast) {
      if (time_current_scan < imuTime[imuPointerFront]) {
        break;
      }
      imuPointerFront = (imuPointerFront + 1) % IMU_BUFFER_SIZE;
    }

    if (time_current_scan > imuTime[imuPointerFront]) {
      roll_imu_current = imuRoll[imuPointerFront];
      pitch_imu_current = imuPitch[imuPointerFront];
      yaw_imu_current = imuYaw[imuPointerFront];

      vx_imu_current = vx_from_imu[imuPointerFront];
      vy_imu_current = vy_from_imu[imuPointerFront];
      zx_imu_current = vz_from_imu[imuPointerFront];

      x_imu_current = x_from_imu[imuPointerFront];
      y_imu_current = y_from_imu[imuPointerFront];
      z_imu_current = z_from_imu[imuPointerFront];
    } else {
      int imuPointerBack = (imuPointerFront + IMU_BUFFER_SIZE - 1) % IMU_BUFFER_SIZE;
      double ratioFront = (time_current_scan - imuTime[imuPointerBack])
                         / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
      double ratioBack = (imuTime[imuPointerFront] - time_current_scan)
                        / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

      roll_imu_current = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
      pitch_imu_current = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
      if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > PI) {
        yaw_imu_current = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * PI) * ratioBack;
      } else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -PI) {
        yaw_imu_current = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * PI) * ratioBack;
      } else {
        yaw_imu_current = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
      }

      vx_imu_current = vx_from_imu[imuPointerFront] * ratioFront + vx_from_imu[imuPointerBack] * ratioBack;
      vy_imu_current = vy_from_imu[imuPointerFront] * ratioFront + vy_from_imu[imuPointerBack] * ratioBack;
      zx_imu_current = vz_from_imu[imuPointerFront] * ratioFront + vz_from_imu[imuPointerBack] * ratioBack;

      x_imu_current = x_from_imu[imuPointerFront] * ratioFront + x_from_imu[imuPointerBack] * ratioBack;
      y_imu_current = y_from_imu[imuPointerFront] * ratioFront + y_from_imu[imuPointerBack] * ratioBack;
      z_imu_current = z_from_imu[imuPointerFront] * ratioFront + z_from_imu[imuPointerBack] * ratioBack;
    }

    if (!imuInited) {
      roll_imu_start_sweep = roll_imu_current;
      pitch_imu_start_sweep = pitch_imu_current;
      yaw_imu_start_sweep = yaw_imu_current;

      vx_imu_start_sweep = vx_imu_current;
      vy_imu_start_sweep = vy_imu_current;
      vz_imu_start_sweep = zx_imu_current;

      x_imu_start_sweep = x_imu_current;
      y_imu_start_sweep = y_imu_current;
      z_imu_start_sweep = z_imu_current;

      imuInited = true;
    }

  }


  /*************************************/
  double x1,y1,z1,x2,y2,z2;
  //translational shift of imu from begin of sweep.
  //TODO : what is '- vx_imu_start_sweep * (dt_scan - time_sweep_start)' term?
  imuShiftFromStartXCur = x_imu_current - x_imu_start_sweep - vx_imu_start_sweep * (dt_scan - time_sweep_start);
  imuShiftFromStartYCur = y_imu_current - y_imu_start_sweep - vy_imu_start_sweep * (dt_scan - time_sweep_start);
  imuShiftFromStartZCur = z_imu_current - z_imu_start_sweep - vz_imu_start_sweep * (dt_scan - time_sweep_start);
  x1 = cos(yaw_imu_start_sweep) * imuShiftFromStartXCur - sin(yaw_imu_start_sweep) * imuShiftFromStartZCur;
  y1 = imuShiftFromStartYCur;
  z1 = sin(yaw_imu_start_sweep) * imuShiftFromStartXCur + cos(yaw_imu_start_sweep) * imuShiftFromStartZCur;

  x2 = x1;
  y2 = cos(pitch_imu_start_sweep) * y1 + sin(pitch_imu_start_sweep) * z1;
  z2 = -sin(pitch_imu_start_sweep) * y1 + cos(pitch_imu_start_sweep) * z1;

  imuShiftFromStartXCur = cos(roll_imu_start_sweep) * x2 + sin(roll_imu_start_sweep) * y2;
  imuShiftFromStartYCur = -sin(roll_imu_start_sweep) * x2 + cos(roll_imu_start_sweep) * y2;
  imuShiftFromStartZCur = z2;
  /*************************************/

  /*************************************/
  imuVeloFromStartXCur = vx_imu_current - vx_imu_start_sweep;
  imuVeloFromStartYCur = vy_imu_current - vy_imu_start_sweep;
  imuVeloFromStartZCur = zx_imu_current - vz_imu_start_sweep;

  x1 = cos(yaw_imu_start_sweep) * imuVeloFromStartXCur - sin(yaw_imu_start_sweep) * imuVeloFromStartZCur;
  y1 = imuVeloFromStartYCur;
  z1 = sin(yaw_imu_start_sweep) * imuVeloFromStartXCur + cos(yaw_imu_start_sweep) * imuVeloFromStartZCur;

  x2 = x1;
  y2 = cos(pitch_imu_start_sweep) * y1 + sin(pitch_imu_start_sweep) * z1;
  z2 = -sin(pitch_imu_start_sweep) * y1 + cos(pitch_imu_start_sweep) * z1;

  imuVeloFromStartXCur = cos(roll_imu_start_sweep) * x2 + sin(roll_imu_start_sweep) * y2;
  imuVeloFromStartYCur = -sin(roll_imu_start_sweep) * x2 + cos(roll_imu_start_sweep) * y2;
  imuVeloFromStartZCur = z2;
  /*************************************/

  for (int i = 0; i < cloudSize; i++) {
    TransformToStartIMU(&laserCloud->points[i]);
  }
  //</editor-fold>
  /**************************************************************************************/

  //Compute 's' (in paper called 'c' value)
  for (int i = 5; i < cloudSize - 5; i++) {
   /*
     Point p = zero;
     for(int k=i-5; k<i+5;k++)
     {
       p = p + (laserCloud->points[k] - laserCloud->points[i]);
     }
     laserCloud->points[i].s = norm(p);
   */
    double diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x
                  + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x
                  + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x
                  + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
                  + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
                  + laserCloud->points[i + 5].x;
    double diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y
                  + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y
                  + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y
                  + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
                  + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
                  + laserCloud->points[i + 5].y;
    double diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z
                  + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z
                  + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z
                  + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
                  + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
                  + laserCloud->points[i + 5].z;

    laserCloud->points[i].s = diffX * diffX + diffY * diffY + diffZ * diffZ;
  }

  for (int i = 5; i < cloudSize - 6; i++) {
    double diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
    double diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
    double diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
    double diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

    //point_(i+1) is disconnected from point_i
    if (diff > 0.05) {

      double depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
                          laserCloud->points[i].y * laserCloud->points[i].y +
                          laserCloud->points[i].z * laserCloud->points[i].z);

      double depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x +
                          laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                          laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);

      if (depth1 > depth2) {
        /*
        point_i is behind of point_(i+1)
        point_i might be the boundary of occluded segment.
        */

        //normalize point_i such that it's length become depth2
        diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
        diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
        diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {
          cloudNeighborPicked[i - 5] = 1;
          cloudNeighborPicked[i - 4] = 1;
          cloudNeighborPicked[i - 3] = 1;
          cloudNeighborPicked[i - 2] = 1;
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i] = 1;
        }
      }
      else
      {
        /*
          point_(i+1) is behind of point_i
          point_(i+1) might be the boundary of occluded segment
        */
        //normalize point_(i+1) such that it's length become depth1
        diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
        diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
        diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
          //if lateral distance of two points is close enough.
          //it is considered point_i is occluded point
          cloudNeighborPicked[i + 1] = 1;
          cloudNeighborPicked[i + 2] = 1;
          cloudNeighborPicked[i + 3] = 1;
          cloudNeighborPicked[i + 4] = 1;
          cloudNeighborPicked[i + 5] = 1;
          cloudNeighborPicked[i + 6] = 1;
        }
      }
    }

    double diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
    double diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
    double diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
    double diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

    double dis = laserCloud->points[i].x * laserCloud->points[i].x
                + laserCloud->points[i].y * laserCloud->points[i].y
                + laserCloud->points[i].z * laserCloud->points[i].z;

    //What the hell is this?
    if (diff > (0.25 * 0.25) / (20 * 20) * dis && diff2 > (0.25 * 0.25) / (20 * 20) * dis) {
      cloudNeighborPicked[i] = 1;
    }
  }

  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsFlat(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZHSV>());

  //Divide single scan into 4 separated subregions(ignoring first and last 5 data points)
  int startPoints[4] = {5, 6 + int((cloudSize - 10) / 4.0),
                        6 + int((cloudSize - 10) / 2.0), 6 + int(3 * (cloudSize - 10) / 4.0)};
  int endPoints[4] = {5 + int((cloudSize - 10) / 4.0), 5 + int((cloudSize - 10) / 2.0),
                      5 + int(3 * (cloudSize - 10) / 4.0), cloudSize - 6};

  //Sort computed value('s' , in paper called 'c') in ascending order
  for (int i = 0; i < 4; i++) {
    int sp = startPoints[i];
    int ep = endPoints[i];

    for (int j = sp + 1; j <= ep; j++) {
      for (int k = j; k >= sp + 1; k--) {
        if (laserCloud->points[cloudSortInd[k]].s < laserCloud->points[cloudSortInd[k - 1]].s) {
          int temp = cloudSortInd[k - 1];
          cloudSortInd[k - 1] = cloudSortInd[k];
          cloudSortInd[k] = temp;
        }
      }
    }

    //Select features for edge
    int largestPickedNum = 0;
    for (int j = ep; j >= sp; j--) {
      if (cloudNeighborPicked[cloudSortInd[j]] == 0 &&
          laserCloud->points[cloudSortInd[j]].s > 0.1 &&
          (fabs(laserCloud->points[cloudSortInd[j]].x) > 0.3 ||
           fabs(laserCloud->points[cloudSortInd[j]].y) > 0.3 ||
           fabs(laserCloud->points[cloudSortInd[j]].z) > 0.3) &&
          fabs(laserCloud->points[cloudSortInd[j]].x) < 30 &&
          fabs(laserCloud->points[cloudSortInd[j]].y) < 30 &&
          fabs(laserCloud->points[cloudSortInd[j]].z) < 30) {

        largestPickedNum++;
        if (largestPickedNum <= 2) {
          //Maybe this is dominant feature(edge)?
          laserCloud->points[cloudSortInd[j]].v = 2;
          cornerPointsSharp->push_back(laserCloud->points[cloudSortInd[j]]);
        } else if (largestPickedNum <= 20) {
          //This is less dominant features(edge)?
          laserCloud->points[cloudSortInd[j]].v = 1;
          cornerPointsLessSharp->push_back(laserCloud->points[cloudSortInd[j]]);
        } else {
          //Stop.
          break;
        }

        //Mark point_i as selected
        cloudNeighborPicked[cloudSortInd[j]] = 1;

        //mark points which follow point_i selected.
        for (int k = 1; k <= 5; k++) {
          double diffX = laserCloud->points[cloudSortInd[j] + k].x
                        - laserCloud->points[cloudSortInd[j] + k - 1].x;
          double diffY = laserCloud->points[cloudSortInd[j] + k].y
                        - laserCloud->points[cloudSortInd[j] + k - 1].y;
          double diffZ = laserCloud->points[cloudSortInd[j] + k].z
                        - laserCloud->points[cloudSortInd[j] + k - 1].z;
          if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
            break;
          }

          cloudNeighborPicked[cloudSortInd[j] + k] = 1;
        }
        //mark points which preceding point_i selected.
        for (int k = -1; k >= -5; k--) {
          double diffX = laserCloud->points[cloudSortInd[j] + k].x
                        - laserCloud->points[cloudSortInd[j] + k + 1].x;
          double diffY = laserCloud->points[cloudSortInd[j] + k].y
                        - laserCloud->points[cloudSortInd[j] + k + 1].y;
          double diffZ = laserCloud->points[cloudSortInd[j] + k].z
                        - laserCloud->points[cloudSortInd[j] + k + 1].z;
          if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
            break;
          }

          cloudNeighborPicked[cloudSortInd[j] + k] = 1;
        }
      }
    }
    //Select features for surface
    int smallestPickedNum = 0;
    for (int j = sp; j <= ep; j++) {
      if (cloudNeighborPicked[cloudSortInd[j]] == 0 &&
          laserCloud->points[cloudSortInd[j]].s < 0.1 &&
          (fabs(laserCloud->points[cloudSortInd[j]].x) > 0.3 ||
           fabs(laserCloud->points[cloudSortInd[j]].y) > 0.3 ||
           fabs(laserCloud->points[cloudSortInd[j]].z) > 0.3) &&
          fabs(laserCloud->points[cloudSortInd[j]].x) < 30 &&
          fabs(laserCloud->points[cloudSortInd[j]].y) < 30 &&
          fabs(laserCloud->points[cloudSortInd[j]].z) < 30) {

        laserCloud->points[cloudSortInd[j]].v = -1;
        surfPointsFlat->push_back(laserCloud->points[cloudSortInd[j]]);

        smallestPickedNum++;
        if (smallestPickedNum >= 4) {
          break;
        }

        cloudNeighborPicked[cloudSortInd[j]] = 1;
        for (int k = 1; k <= 5; k++) {
          double diffX = laserCloud->points[cloudSortInd[j] + k].x
                        - laserCloud->points[cloudSortInd[j] + k - 1].x;
          double diffY = laserCloud->points[cloudSortInd[j] + k].y
                        - laserCloud->points[cloudSortInd[j] + k - 1].y;
          double diffZ = laserCloud->points[cloudSortInd[j] + k].z
                        - laserCloud->points[cloudSortInd[j] + k - 1].z;
          if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
            break;
          }

          cloudNeighborPicked[cloudSortInd[j] + k] = 1;
        }
        for (int k = -1; k >= -5; k--) {
          double diffX = laserCloud->points[cloudSortInd[j] + k].x
                        - laserCloud->points[cloudSortInd[j] + k + 1].x;
          double diffY = laserCloud->points[cloudSortInd[j] + k].y
                        - laserCloud->points[cloudSortInd[j] + k + 1].y;
          double diffZ = laserCloud->points[cloudSortInd[j] + k].z
                        - laserCloud->points[cloudSortInd[j] + k + 1].z;
          if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
            break;
          }

          cloudNeighborPicked[cloudSortInd[j] + k] = 1;
        }
      }
    }
  }

  //Normal points
  for (int i = 0; i < cloudSize; i++) {
    if (laserCloud->points[i].v == 0) {
      surfPointsLessFlat->push_back(laserCloud->points[i]);
    }
  }

  //Downsampling
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlatDS(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::VoxelGrid<pcl::PointXYZHSV> downSizeFilter;
  downSizeFilter.setInputCloud(surfPointsLessFlat);
  downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
  downSizeFilter.filter(*surfPointsLessFlatDS);

  //Accumulate feature points and normal points for single scan
  *laserCloudFeaturesCurrent += *cornerPointsSharp;
  *laserCloudFeaturesCurrent += *surfPointsFlat;
  *laserCloudLessExtreCur += *cornerPointsLessSharp;
  *laserCloudLessExtreCur += *surfPointsLessFlatDS;

  if (skipFrameCount >= SKIP_FRAME_COUNT) {
    //publish feature points(growing) and last point cloud(feature+non-feature, local) with 13.36 Hz
    skipFrameCount = 0;

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZHSV>(4, 1));
    imuTrans->points[0].x = pitch_imu_start_sweep;
    imuTrans->points[0].y = yaw_imu_start_sweep;
    imuTrans->points[0].z = roll_imu_start_sweep;
    imuTrans->points[0].v = 10;

    imuTrans->points[1].x = pitch_imu_current;
    imuTrans->points[1].y = yaw_imu_current;
    imuTrans->points[1].z = roll_imu_current;
    imuTrans->points[1].v = 11;

    imuTrans->points[2].x = imuShiftFromStartXCur;
    imuTrans->points[2].y = imuShiftFromStartYCur;
    imuTrans->points[2].z = imuShiftFromStartZCur;
    imuTrans->points[2].v = 12;

    imuTrans->points[3].x = imuVeloFromStartXCur;
    imuTrans->points[3].y = imuVeloFromStartYCur;
    imuTrans->points[3].z = imuVeloFromStartZCur;
    imuTrans->points[3].v = 13;

    sensor_msgs::PointCloud2 features_out_msg;
    pcl::toROSMsg(*laserCloudFeaturesCurrent + *imuTrans, features_out_msg);
    features_out_msg.header.stamp = ros::Time().fromSec(time_current_scan);
    features_out_msg.header.frame_id = "/camera";

    pubLaserCloudExtreCurPointer->publish(features_out_msg);
    pubLaserCloudLastPointer->publish(laserCloudLast);

    //ROS_INFO ("%d %d", laserCloudLast.width, features_out_msg.width);
  }
  skipFrameCount++;

//  printf("%+6.3lf %+6.3lf %+6.3lf %+6.3lf %+6.3lf %+6.3lf %+6.3lf %+6.3lf %+6.3lf\n",
//         vx_imu_current, vy_imu_current, zx_imu_current,
//         x_imu_current, y_imu_current, z_imu_current,
//  roll_imu_current, pitch_imu_current, yaw_imu_current);
//  fflush(stdout);
}

/*
 * Dead reckoning from raw accerlomter.
 */
void AccumulateIMUShift()
{
  double roll = imuRoll[imuPointerLast];
  double pitch = imuPitch[imuPointerLast];
  double yaw = imuYaw[imuPointerLast];
  double accX = imuAccX[imuPointerLast];
  double accY = imuAccY[imuPointerLast];
  double accZ = imuAccZ[imuPointerLast];

  /**********************Transform acceleration from local to reference frame****************************/
  double x1 = cos(roll) * accX - sin(roll) * accY;
  double y1 = sin(roll) * accX + cos(roll) * accY;
  double z1 = accZ;

  double x2 = x1;
  double y2 = cos(pitch) * y1 - sin(pitch) * z1;
  double z2 = sin(pitch) * y1 + cos(pitch) * z1;

  accX = cos(yaw) * x2 + sin(yaw) * z2;
  accY = y2;
  accZ = -sin(yaw) * x2 + cos(yaw) * z2;
  /*****************************************************************************************************/

  /********************Dead Reckoning.....**************************************************************/
  int imuPointerBack = (imuPointerLast -1 + IMU_BUFFER_SIZE) % IMU_BUFFER_SIZE;
  double dt = imuTime[imuPointerLast] - imuTime[imuPointerBack];
  if (dt < 0.1) //This if statement is meaningless.(imu is published with 400 hz)
  {
    x_from_imu[imuPointerLast] = x_from_imu[imuPointerBack] + vx_from_imu[imuPointerBack] * dt
                                + accX * dt * dt / 2;
    y_from_imu[imuPointerLast] = y_from_imu[imuPointerBack] + vy_from_imu[imuPointerBack] * dt
                                + accY * dt * dt / 2;
    z_from_imu[imuPointerLast] = z_from_imu[imuPointerBack] + vz_from_imu[imuPointerBack] * dt
                                + accZ * dt * dt / 2;

    vx_from_imu[imuPointerLast] = vx_from_imu[imuPointerBack] + accX * dt;
    vy_from_imu[imuPointerLast] = vy_from_imu[imuPointerBack] + accY * dt;
    vz_from_imu[imuPointerLast] = vz_from_imu[imuPointerBack] + accZ * dt;
  }
  /*****************************************************************************************************/
}

/*
 * Set current orientation(Roll,Pitch,Yaw) from imu data.
 * Set linear acceleration(gravity compensated) from raw imu data.
 */
void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  //remove gravity from raw accelerometer output.
  double accX = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
  double accY = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;
  double accZ = imuIn->linear_acceleration.x + sin(pitch) * 9.81;

  imuPointerLast = (imuPointerLast + 1) % IMU_BUFFER_SIZE;

  imuTime[imuPointerLast] = imuIn->header.stamp.toSec() - 0.1068;//TODO : what is 0.1068??
  imuRoll[imuPointerLast] = roll;
  imuPitch[imuPointerLast] = pitch;
  imuYaw[imuPointerLast] = yaw;
  imuAccX[imuPointerLast] = accX;
  imuAccY[imuPointerLast] = accY;
  imuAccZ[imuPointerLast] = accZ;

  AccumulateIMUShift();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle nh;

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
          ("/sync_scan_cloud_filtered", 2, laserCloudHandler);

  ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu>
          ("/imu/data", 5, imuHandler);

  pubLaserCloudExtreCurPointer.reset(new ros::Publisher(nh.advertise<sensor_msgs::PointCloud2>
          ("/laser_cloud_extre_cur", 2)));
  pubLaserCloudLastPointer.reset(new ros::Publisher(nh.advertise<sensor_msgs::PointCloud2>
          ("/laser_cloud_last", 2)));

  ros::spin();

  return 0;
}
