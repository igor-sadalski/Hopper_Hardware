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
#include<thread>

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
float states[10];

vector_t state(10);
float desstate[7];
std::mutex state_mtx;
std::mutex des_state_mtx;

void getStateFromESP() {
  while(1) {
  vector_t state_vec(10);

  //receive string states, ESP8266 -> PC
  char start_msg[2] = {0,0};
  std::bitset<8> x0(start_msg[0]);
  std::bitset<8> x1(start_msg[1]);
  read(sockfd, buff, sizeof(buff));
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
  {std::lock_guard<std::mutex> lck(state_mtx);
  state << states[0], states[1], states[2], states[3], states[4], states[5], states[6], states[7], states[8], states[9];
  }
 
  // encode send_buff
  send_buff[0] = 0b11111111;
  send_buff[1] = 0b11111111;
  {std::lock_guard<std::mutex> lck(des_state_mtx);
  memcpy(send_buff+2, desstate, 7*4);
  }
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
  }

  //return state_vec;
}

void setupSocket() {
        // socket stuff
        struct sockaddr_in servaddr, cli;

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
	state.setZero();
        vector_t state_vec(10); //states + 1

        bool fileWrite = true;
        std::string dataLog = "../data/data.csv";
	std::ofstream fileHandle;
        fileHandle.open(dataLog);

        std::chrono::high_resolution_clock::time_point t1;
        std::chrono::high_resolution_clock::time_point t2;
        std::chrono::high_resolution_clock::time_point tstart;
        tstart = std::chrono::high_resolution_clock::now();

	signal(SIGINT, signal_callback_handler);

	setupSocket();

	desstate[0] = 1;
        desstate[1] = 0;
        desstate[2] = 0;
        desstate[3] = 0;
        desstate[4] = 0;
        desstate[5] = 0;
        desstate[6] = 0;

	std::thread thread_object(getStateFromESP);

        // ROS stuff
	//ros::init(argc, argv, "listener");
	//ros::NodeHandle n;
	//ros::Subscriber sub = n.subscribe("/vrpn_client_node/hopper/pose", 200, chatterCallback);
	//while(ros::ok()){
          //ros::spinOnce();
	while(1) {
          t1 = std::chrono::high_resolution_clock::now();
          
	  { std::lock_guard<std::mutex> lck(state_mtx);
	    state_vec = state;
	  }

          std::cout <<"Global state: " << OptiState.x << ", " << OptiState.y << ", " << OptiState.z <<std::endl;
          
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
          
	  { std::lock_guard<std::mutex> lck(des_state_mtx);
          desstate[0] = qd_w;
          desstate[1] = qd_x;
          desstate[2] = qd_y;
          desstate[3] = qd_z;
          desstate[4] = wd_x;
          desstate[5] = wd_y;
          desstate[6] = wd_z;
	  }

          t2 = std::chrono::high_resolution_clock::now();
          std::cout <<"Timing: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count()*1e-6 << "[ms]" << "\n";
          
          if (fileWrite)
            fileHandle << std::chrono::duration_cast<std::chrono::nanoseconds>(t2-tstart).count()*1e-9 << "," << OptiState.x << "," << OptiState.y << "," << OptiState.z << "," << OptiState.q_w<< "," << OptiState.q_x << "," << OptiState.q_y << "," << OptiState.q_z << "," << state_vec.transpose().format(CSVFormat)<<std::endl;

	  std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	close(sockfd);
}

//roslaunch vrpn_client_ros fast.launch server:=192.168.1.5
//ifconfig enx5c857e36c21a 169.254.10.80
