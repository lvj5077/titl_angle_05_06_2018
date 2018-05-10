#include "imu_reader.h"

#define D2R(d) (((d)/180.)*M_PI)

void CMotorIntend::computeTilt()
{
  class CIMUReader; 

  CIMUReader * mpIMU; 

  if(mpIMU == 0)
  {
    mpIMU = new CIMUReader; 
  }


  double a_gy = 0; 
  double a_gz = 0; 

  int N = 100;
  float invN = 0.01; 

  for(int i=0; i< N; i++)
  {
    IMUData d = mpIMU->getNewIMUData(); 
    a_gy += d.may; 
    a_gz += d.maz;
  }

  a_gy *= invN; 
  a_gz *= invN; 

  // cout <<"motor_intend: computeTilt a_gy: "<<a_gy<<" a_gz: "<<a_gz<<endl; 

  double m_alpha = D2R(25.); // 25.5 this value is measured by a gyro, placed on the padel 

  double ca = cos(-(M_PI/2. - m_alpha));
  double sa = sin(-(M_PI/2. - m_alpha)); 

  Ro2i = Eigen::Matrix3d::Identity(); // coordinate Origin (located on the cane) to IMU
  Ro2i(1,1) = ca; Ro2i(1,2) = -sa;
  Ro2i(2,1) = sa; Ro2i(2,2) = ca; 
  Eigen::Vector3d gI{0., a_gy, a_gz}; 
  Eigen::Vector3d gO = Ro2i * gI; 

  // cout <<"motor_intend: gyO: "<<gO(1)<<" gzO: "<<gO(2)<<endl; 
  double theta = atan2(gO(1), gO(2)); 
  cout <<"motor_intend: tilt angle theta = "<<R2D(fabs(theta))<<endl;

  m_theta = theta > 0? theta : -theta; 
  return ; 
  
}