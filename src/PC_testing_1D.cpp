#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <chrono>
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

#define MAX 48
#define STATE_SIZE 40
#define MSG_SIZE 48+4-2
#define PORT 8888
#define SA struct sockaddr

using namespace Eigen;
using namespace Hopper_t;

vector_3t Kp_gains = {1,1,1};
vector_3t Kd_gains = {0,0,0}; //?

struct State {
  scalar_t x;
  scalar_t y;
  scalar_t z;
  scalar_t q_w;
  scalar_t q_x;
  scalar_t q_y;
  scalar_t q_z;
} state;

matrix_3t cross(vector_3t q) {
    matrix_3t c;
    c << 0, -q(2), q(1),
            q(2), 0, -q(0),
            -q(1), q(0), 0;
    return c;
}

void TokenizeStringToFloats(char str[], float state[]){
    //char * pch;
    //pch = strtok (str,",");
    //int i = 0;
    //while (pch != NULL)
    //{
    //    state[i] = strtof (pch, NULL);
    //    i++;
    //    pch = strtok (NULL, ",");
    //}
   //
    memcpy(state, str, sizeof(str));
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
  state.x = msg->pose.position.x;
  state.y = msg->pose.position.y;
  state.z = msg->pose.position.z;
  state.q_w = msg->pose.orientation.w;
  state.q_x = msg->pose.orientation.x;
  state.q_y = msg->pose.orientation.y;
  state.q_z = msg->pose.orientation.z;
}

int main(int argc, char **argv)
{
	quat_t quat_d;
	quat_t quat_a;
	vector_3t omega_a;
	vector_3t omega_d;
	vector_3t torque;
	scalar_t Q_w, Q_x, Q_y, Q_z;
	scalar_t w_x, w_y, w_z;
	char buff[MSG_SIZE];
	char buffMAX[MSG_SIZE];
	char send_buff[34];
	//torques to currents, assumes torques is an array of floats
	scalar_t const_wheels = 0.083;
	bool init = false;
	vector_t state_init(10);
	quat_t quat_init;

        // socket stuff
	int sockfd, connfd;
	struct sockaddr_in servaddr, cli;
	float states[10]; //states + 1 
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
	//read(sockfd, buff, 32); // Weird thing to clear the buffer


	////cannot use spaces or special characters in the string message as the program otherwise freaks out
	
	   
        std::chrono::high_resolution_clock::time_point t1;

        std::chrono::high_resolution_clock::time_point t2;

  

	//write(sockfd, "ok", 3);        
        std::chrono::steady_clock::time_point begin;
        std::chrono::steady_clock::time_point end;


	//ros::init(argc, argv, "listener");
	//ros::NodeHandle n;
	//ros::Subscriber sub = n.subscribe("/vrpn_client_node/hopper/pose", 200, chatterCallback);
	//ros::spin();
	while(1){
		//ros::spinOnce();
		t1 = std::chrono::high_resolution_clock::now();
		//read(sockfd, buffMAX, sizeof(buffMAX));
		//bzero(buff, sizeof(buff));
		//std::cout << "Message: " << std::endl;
		//for (int i = 0; i < MSG_SIZE; i++) {
		//	std::bitset<8> x(buffMAX[i]);
		//	std::cout << x << " ";
		//}
		
		

		//receive string states, ESP8266 -> PC
		char start_msg[2] = {0,0};
		std::bitset<8> x0(start_msg[0]);
		std::bitset<8> x1(start_msg[1]);
		while ((x0 != 0b11111111) && (x1 != 0b11111111)){
		  read(sockfd, start_msg, sizeof(start_msg));
		  x0 = std::bitset<8>(start_msg[0]);
		  x1 = std::bitset<8>(start_msg[1]);
		  //std::cout << (x0 != 0b11111111) << ", " << (x1 != 0b11111111) << "," <<((x0 != 0b11111111) && (x1 != 0b11111111)) << std::endl;
		  std::cout << x0 << "," << x1 << std::endl;
                }
		if ((x0 != 0b11111111) && (x1 == 0b11111111)) {
		  read(sockfd, start_msg, sizeof(char));
		  read(sockfd, buff, sizeof(buff));
		}
		else if ((x0 == 0b11111111) && (x1 != 0b11111111)) {
		  read(sockfd, buff+1, sizeof(buff)-sizeof(char));
		  buff[0] = start_msg[1];
		} else {
		  read(sockfd, buff, sizeof(buff));
		}
		//std::cout << x0 << ", " << x1 << std::endl;
		//printf("%s \n", buff);	
		char oneAdded[6];
		memcpy(oneAdded, buff+40, 6*sizeof(char));
		//std::cout << "Message: " << std::endl;
		//for (int i = 0; i < 40; i++) {
		//	std::bitset<8> x(buff[i]);
		//	std::cout << x << " ";
		//}
		//std::cout << std::endl;
		//std::cout << "One added: " << std::endl;
		//for (int i = 0; i < 6; i++) {
		//	std::bitset<8> x(oneAdded[i]);
		//	std::cout << x << " ";
		//}
		//std::cout << std::endl;
		for (int i = 0; i < 6; i++) {
	          for (int j = 1; j < 8; j++) {
	            if(oneAdded[i] & (1 << (8-j))) {
 	              buff[i*7+(j-1)] = 0;
	            }
	          }
		}

		float tmp[10];
		memcpy(&tmp, buff, 10*sizeof(float));
		//std::cout << "Message: ";
		//for (int i = 0; i < 10; i++) {
     	        //  std::cout << tmp[i] << ", ";
		//}
		//std::cout << std::endl;
		//std::cout <<"Global state: " << state.x << ", " << state.y << ", " << state.z <<std::endl;


		// ROS stuff
		
		//assume tokenization on ,
		TokenizeStringToFloats(buff, states);
		if (!init) {
		  state_init << states[0], states[1], states[2], states[3], states[4], states[5], states[6], states[7], states[8], states[9]; // save the initial condition
		  quat_init = Quaternion<scalar_t>(states[6], states[7], states[8], states[9]);
		  init = true;
		}
		state_vec.setZero();
		state_vec << states[0], states[1], states[2], states[3], states[4], states[5], states[6], states[7], states[8], states[9];
		
		//std::cout << "state: " << state_vec.transpose() << std::endl;
		//w_x = states[3];
		//w_y = states[4];
		//w_z = states[5];				
		//Q_w = states[6];
		//Q_x = states[7];
		//Q_y = states[8];
		//Q_z = states[9];	
		//quat_d = Quaternion<scalar_t>(1,0,0,0);
		//quat_a = Quaternion<scalar_t>(Q_w,Q_x,Q_y,Q_z);
		//quat_a = quat_init.inverse()*quat_a;
		//omega_d << 0,0,0; 
		//omega_a << w_x, w_y, w_z;
		//computeTorque(quat_a, quat_d, omega_a, omega_d, torque);
		//torque << 0,0,10.1234;
		//bzero(send_buff, sizeof(send_buff));
		////sprintf(send_buff,"<{%.4f,%.4f,%.4f}>",torque(0)/const_wheels, torque(1)/const_wheels, torque(2))/const_wheels;	
		scalar_t qd_w = 0.0;
		scalar_t qd_x = 0.0;
		scalar_t qd_y = 0.0;
		scalar_t qd_z = 0.0;
		scalar_t wd_x = 0.0;
		scalar_t wd_y = 0.0;
		scalar_t wd_z = 0.0;
		float d_state[7];
		d_state[0] = qd_w;
		d_state[1] = qd_x;
		d_state[2] = qd_y;
		d_state[3] = qd_z;
		d_state[4] = wd_x;
		d_state[5] = wd_y;
		d_state[6] = wd_z;
		//sprintf(send_buff,"<{%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f}>",qd_w, qd_x, qd_y, qd_z, wd_x, wd_y, wd_z);	
		//std::cout << "torque: " << send_buff << std::endl;
		//
		//
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
		//usleep(20000);
		//read(sockfd, start_msg, sizeof(start_msg));
		//x0 = std::bitset<8>(start_msg[0]);
                //  x1 = std::bitset<8>(start_msg[1]);
                //  //std::cout << (x0 != 0b11111111) << ", " << (x1 != 0b11111111) << "," <<((x0 != 0b11111111) && (x1 != 0b11111111)) << std::endl;
                //  std::cout << x0 << "," << x1 << std::endl;
		//read(sockfd, start_msg, sizeof(start_msg));
		//x0 = std::bitset<8>(start_msg[0]);
                //  x1 = std::bitset<8>(start_msg[1]);
                //  //std::cout << (x0 != 0b11111111) << ", " << (x1 != 0b11111111) << "," <<((x0 != 0b11111111) && (x1 != 0b11111111)) << std::endl;
                //  std::cout << x0 << "," << x1 << std::endl;
		t2 = std::chrono::high_resolution_clock::now();
                std::cout <<"Timing: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count()*1e-6 << "[ms]" << "\n";
	}

	close(sockfd);
}

