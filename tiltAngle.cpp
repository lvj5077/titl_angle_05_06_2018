#include "imu_reader.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

#define D2R(d) (((d)/180.)*M_PI)
#define R2D(r) ((r)*180./M_PI)

using namespace std;
int main(int argc, char* argv[])
{

  CIMUReader imu_reader; 

  double a_gy = 0; 
  double a_gz = 0; 

  int N = 100;
  float invN = 0.01; 

  for(int i=0; i< N; i++)
  {
    IMUData d = imu_reader.getNewIMUData(); 
    printf(" %d data: %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f\n", i, d.mroll, 
          d.mpitch, d.myaw, d.max, d.may, d.maz);
    a_gy += d.may; 
    a_gz += d.maz;
  }

  a_gy *= invN; 
  a_gz *= invN; 

  // cout <<"motor_intend: computeTilt a_gy: "<<a_gy<<" a_gz: "<<a_gz<<endl; 

  double m_alpha = D2R(25.); // 25.5 this value is measured by a gyro, placed on the padel 

  double ca = cos(-(M_PI/2. - m_alpha));
  double sa = sin(-(M_PI/2. - m_alpha)); 

  Eigen::Matrix3d Ro2i = Eigen::Matrix3d::Identity(); // coordinate Origin (located on the cane) to IMU
  Ro2i(1,1) = ca; Ro2i(1,2) = -sa;
  Ro2i(2,1) = sa; Ro2i(2,2) = ca; 
  Eigen::Vector3d gI{0., a_gy, a_gz}; 
  Eigen::Vector3d gO = Ro2i * gI; 

  // cout <<"motor_intend: gyO: "<<gO(1)<<" gzO: "<<gO(2)<<endl; 
  double theta = atan2(gO(1), gO(2)); 
  cout <<"motor_intend: tilt angle theta = "<<R2D(fabs(theta))<<endl;

  double m_theta = theta > 0? theta : -theta; 
  return 0; 
  
}