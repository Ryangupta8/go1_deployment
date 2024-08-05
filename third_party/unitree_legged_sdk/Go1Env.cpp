#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
using namespace UNITREE_LEGGED_SDK;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Quaterniond;

class Go1Env{
public:

  Safety safe;
  UDP udp;

  LowCmd cmd = {0};
  LowState state = {0};
  xRockerBtnDataStruct _keyData;

  double kp = 30;
  double kd = 0.5;
  double ka = 0.25;

  double CONTROL_STEP = 0.002;
  double POLICY_STEP = 0.02;
  int H = 5;

  VectorXd q_stand(-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, 0.1, 1.0, -1.5);
  Vector3d vel_cmd, projected_gravity;
  Quaterniond body_quat;
  VectorXd = q, dq, a_cmd;
  double obs[42]

  bool init;
  

  Go1Env(uint8_t level) : safe(LeggedType::Go1), udp(level, 8080, "192.168.123.10", 8007)
  {
    udp.InitCmdData(cmd);
    Go1Env();
  }

  Go1Env(){
    q = Eigen::VectorXd::Zero(12);
    dq = Eigen::VectorXd::Zero(12);
    a_cmd = Eigen::VectorXd::Zero(12);
    
    vel_cmd = Eigen::VectorXd::Zero(3);
    projected_gravity = Eigen::VectorXd::Zero(3);

    init = 0;

  }

  void init_robot(){
    std::cout << "**************************************" << std::endl
              << "*********    IMPORTANT    ************" << std::endl
              << "    ENSURE STARTING CONFIG NONZERO    " << std::endl
              << "**************************************" << std::endl
              << "Press any key to continue ..." << std::endl;
    std::cin.ignore();
  }

  void UDPRecv(){
    udp.Recv();
  }

  void UDPSend(){
    udp.Send();
  }

  void set_vel_cmd(){
    memcpy(&_keyData, &state.wirelessRemote[0], 40);

    // todo -- test the ranges and set vel_cmd 
    if ((int)_keyData.btn.components.A == 1)
    {
      std::cout << "The key A is pressed, and the value of lx is " << _keyData.lx << std::endl;
    }
    std::cout << "value of lx = " << _keyData.lx << std::endl;
    std::cout << "value of ly = " << _keyData.ly << std::endl;
    std::cout << "value of rx = " << _keyData.rx << std::endl;
    std::cout << "value of ry = " << _keyData.ry << std::endl;
    std::cout << "value of L2 = " << _keyData.L2 << std::endl;
    
    vel_cmd[0] = _keyData.lx; // x
    vel_cmd[1] = _keyData.ly; // y
    vel_cmd[2] = _keyData.ry; // omega
  }

  
  void step(){
    udp.GetRecv(state);
    
    for(int i=0; i<12; ++i){
      q[i] = state.motorState[i].q;
      dq[i] = state.motorState[i].dq;
    }
    body_quat = Quaterniond(state.imu.quaternion[0], state.imu.quaternion[1],
                            state.imu.quaternion[2], state.imu.quaternion[3]); 
                            // (w,x,y,z)
    quat_rot_inv(body_quat);

    if(!init){
      init_robot();
      init = 1;
    }

    set_vel_cmd();

  }

  void quat_rot_inv(Quaterniond quat){
    double q_w = quat.w();
    Vector3d q_vec(quat.,x(), quat.y(), quat.z());
    Vector3d gravity(0., 0., -9.81);
    Vector3d a = gravity * (2.0*q_w*q_w - 1.0);
    Vector3d b = q_vec.cross(gravity) * q_w * 2.0;
    Vector3d c = q_vec * q_vec * gravity;
    projected_gravity = a - b + c;
  }

  
};


int main(void)
{
  Go1Env custom(LOWLEVEL);
  LoopFunc loop_control("control_loop", custom.CONTROL_STEP, boost::bind(&Go1Env::step, &custom));
  LoopFunc loop_udpSend("udp_send", custom.CONTROL_STEP, 3, boost::bind(&Go1Env::UDPSend, &custom));
  LoopFunc loop_udpRecv("udp_recv", custom.CONTROL_STEP, 3, boost::bind(&Go1Env::UDPRecv, &custom));

  loop_udpSend.start();
  loop_udpRecv.start();
  loop_control.start();

  while (1)
  {
    sleep(10);
  };

  return 0;
}