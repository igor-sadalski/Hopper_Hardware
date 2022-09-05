#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <chrono>
#include <cstdlib>
#include <signal.h>
#include <bitset>

#include "../inc/Types.h"

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/AccelStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/String.h"

#include <manif/manif.h>

#define MAX 48
#define STATE_SIZE 40
#define MSG_SIZE 48+4
#define PORT 8888
#define SA struct sockaddr

using namespace Eigen;
using namespace Hopper_t;

const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

vector_3t Kp_gains = {1,1,1};
vector_3t Kd_gains = {0,0,0}; //?

struct State {
  scalar_t x;
  scalar_t y;
  scalar_t z;
  scalar_t x_dot;
  scalar_t y_dot;
  scalar_t z_dot;
  scalar_t q_w;
  scalar_t q_x;
  scalar_t q_y;
  scalar_t q_z;
  scalar_t w_x;
  scalar_t w_y;
  scalar_t w_z;
} OptiState;

matrix_3t cross(vector_3t q) {
    matrix_3t c;
    c << 0, -q(2), q(1),
            q(2), 0, -q(0),
            -q(1), q(0), 0;
    return c;
}

void computeTorque(quat_t quat_a, quat_t quat_d, vector_3t omega_a, vector_3t omega_d, vector_3t &torque) {
    
    vector_3t delta_omega;
    vector_4t quat_d_vec;
    vector_4t quat_a_vec;
    
    quat_t quat_actuator = quat_t(0.8806, 0.3646, -0.2795, 0.1160);

    quat_d *= quat_actuator;
    quat_a *= quat_actuator;

    quat_d_vec << quat_d.w(), quat_d.x(), quat_d.y(), quat_d.z();
    quat_a_vec << quat_a.w(), quat_a.x(), quat_a.y(), quat_a.z();

    vector_3t delta_quat;
    delta_quat << quat_a_vec[0] * quat_d_vec.segment(1, 3) - quat_d_vec[0] * quat_a_vec.segment(1, 3) -
                  cross(quat_a_vec.segment(1, 3)) * quat_d_vec.segment(1, 3);
    matrix_3t Kp, Kd;
    Kp.setZero();
    Kd.setZero();
    Kp.diagonal() << Kp_gains;
    Kd.diagonal() << Kd_gains;

    vector_3t tau;
    delta_omega = -quat_actuator.inverse()._transformVector(omega_a) - omega_d;
    tau = -Kp * delta_quat - Kd * delta_omega;

    torque << tau;
};

void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  OptiState.x = msg->pose.position.x;
  OptiState.y = msg->pose.position.y;
  OptiState.z = msg->pose.position.z;
  OptiState.q_w = msg->pose.orientation.w;
  OptiState.q_x = msg->pose.orientation.x;
  OptiState.q_y = msg->pose.orientation.y;
  OptiState.q_z = msg->pose.orientation.z;
}

void signal_callback_handler(int signum) {
	std::cout << "Caught signal " << signum << std::endl;
   // Terminate program
   exit(signum);
}


int sockfd, connfd;
char buff[MSG_SIZE];
char buffMAX[MSG_SIZE];
char send_buff[34];
float states[10]; //states + 1 

vector_t getStateFromESP() {
  vector_t state_vec(10);

  //receive string states, ESP8266 -> PC
  char start_msg[2] = {0,0};
  std::bitset<8> x0(start_msg[0]);
  std::bitset<8> x1(start_msg[1]);
  //while ((x0 != 0b11111111) && (x1 != 0b11111111)){
  //  read(sockfd, start_msg, sizeof(start_msg));
  //  x0 = std::bitset<8>(start_msg[0]);
  //  x1 = std::bitset<8>(start_msg[1]);
  //}
  //if ((x0 != 0b11111111) && (x1 == 0b11111111)) {
  //  read(sockfd, start_msg, sizeof(char));
  //  read(sockfd, buff, sizeof(buff));
  //}
  //else if ((x0 == 0b11111111) && (x1 != 0b11111111)) {
  //  read(sockfd, buff+1, sizeof(buff)-sizeof(char));
  //  buff[0] = start_msg[1];
  //} else {
    read(sockfd, buff, sizeof(buff));
  //}
  char oneAdded[6];
  memcpy(oneAdded, buff+42, 6*sizeof(char));
  for (int i = 0; i < 6; i++) {
    for (int j = 1; j < 8; j++) {
      if(oneAdded[i] & (1 << (8-j))) {
        buff[i*7+(j-1)+2] = 0;
      }
    }
  }
  
  memcpy(&states, buff+2, 10*sizeof(float));
  state_vec << states[0], states[1], states[2], states[3], states[4], states[5], states[6], states[7], states[8], states[9];
   return state_vec;
}

int main(int argc, char **argv)
{
	quat_t quat_a, quat_d;
	vector_3t omega_a, omega_d, torque;
	scalar_t Q_w, Q_x, Q_y, Q_z;
	scalar_t w_x, w_y, w_z;
	//torques to currents, assumes torques is an array of floats
	scalar_t const_wheels = 0.083;
	bool init = false;
	vector_t state_init(10);

        bool fileWrite = true;
        std::string dataLog = "../data/data.csv";
	std::ofstream fileHandle;
        fileHandle.open(dataLog);

        // socket stuff
	struct sockaddr_in servaddr, cli;
	vector_t state_vec(10); //states + 1 

	// socket create and verification
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd == -1) {
		printf("socket creation failed...\n");
		exit(0);
	}
	else
		printf("Socket successfully created..\n");
	bzero(&servaddr, sizeof(servaddr));

	// assign IP, PORT
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = inet_addr("192.168.1.4"); 
	servaddr.sin_port = htons(PORT);

	// connect the client socket to server socket
	if (connect(sockfd, (SA*)&servaddr, sizeof(servaddr)) != 0) {
		printf("connection with the server failed...\n");
		exit(0);
	}
	else
		printf("connected to the server..\n");
	sleep(1);
	   
        std::chrono::high_resolution_clock::time_point t1;
        std::chrono::high_resolution_clock::time_point t2;
        std::chrono::high_resolution_clock::time_point tstart;
        tstart = std::chrono::high_resolution_clock::now();

	signal(SIGINT, signal_callback_handler);

	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/vrpn_client_node/hopper/pose", 200, chatterCallback);
	while(ros::ok()){
          ros::spinOnce();
	//while(1) {
          t1 = std::chrono::high_resolution_clock::now();
          
          state_vec = getStateFromESP();
          std::cout <<"Global state: " << OptiState.x << ", " << OptiState.y << ", " << OptiState.z <<std::endl;
          //t2 = std::chrono::high_resolution_clock::now();
          //std::cout <<"ESP Timing: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count()*1e-6 << "[ms]" << "\n";
          
          // ROS stuff
          if (!init) {
            state_init = state_vec;
            init = true;
          }
          
          std::cout << "state: " << state_vec.transpose().format(CSVFormat) << std::endl;
          w_x = state_vec[3];
          w_y = state_vec[4];
          w_z = state_vec[5];				
          Q_w = state_vec[6];
          Q_x = state_vec[7];
          Q_y = state_vec[8];
          Q_z = state_vec[9];	
          quat_d = Quaternion<scalar_t>(1,0,0,0);
          quat_a = Quaternion<scalar_t>(Q_w,Q_x,Q_y,Q_z);
          omega_d << 0,0,0; 
          omega_a << w_x, w_y, w_z;

	  vector_t state(21);

          
          //computeTorque(quat_a, quat_d, omega_a, omega_d, torque);
          //torque << 0,0,10.1234;
          //bzero(send_buff, sizeof(send_buff));
          ////sprintf(send_buff,"<{%.4f,%.4f,%.4f}>",torque(0)/const_wheels, torque(1)/const_wheels, torque(2))/const_wheels;	
	  //
	  
	  manif::SO3Tangent<scalar_t> delta_theta; 
	  delta_theta << 0,0,0*std::chrono::duration_cast<std::chrono::milliseconds>(t2-tstart).count()*1e-3;
	  quat_d = delta_theta.exp().quat();

          scalar_t qd_w = quat_d.w();
          scalar_t qd_x = quat_d.x();
          scalar_t qd_y = quat_d.y();
          scalar_t qd_z = quat_d.z();
          scalar_t wd_x = 0;
          scalar_t wd_y = 0;
          scalar_t wd_z = 0;
          
          float d_state[7];
          d_state[0] = qd_w;
          d_state[1] = qd_x;
          d_state[2] = qd_y;
          d_state[3] = qd_z;
          d_state[4] = wd_x;
          d_state[5] = wd_y;
          d_state[6] = wd_z;
          
          // encode send_buff
          send_buff[0] = 0b11111111;
          send_buff[1] = 0b11111111;
          memcpy(send_buff+2, d_state, 7*4);
          for (int i = 0; i < 4; i++) {
              char oneAdded = 0b00000001;
              for (int j = 1; j < 8; j++){
                if (send_buff[i*7+(j-1)+2] == 0b00000000) {
                  send_buff[i*7+(j-1)+2] = 0b00000001;
                  oneAdded += (1 << (8-j));
                }
              }
              memcpy(&send_buff[28+i+2], &oneAdded, 1);
          }
          
          write(sockfd, send_buff, sizeof(send_buff));		
          
          t2 = std::chrono::high_resolution_clock::now();
          std::cout <<"Timing: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count()*1e-6 << "[ms]" << "\n";
          
          if (fileWrite)
            fileHandle << std::chrono::duration_cast<std::chrono::nanoseconds>(t2-tstart).count()*1e-9 << "," << OptiState.x << "," << OptiState.y << "," << OptiState.z << "," << OptiState.q_w<< "," << OptiState.q_x << "," << OptiState.q_y << "," << OptiState.q_z << "," << state_vec.transpose().format(CSVFormat)<<std::endl;
	}
	close(sockfd);
}

//roslaunch vrpn_client_ros fast.launch server:=192.168.1.5
//ifconfig enx5c857e36c21a 169.254.10.80
