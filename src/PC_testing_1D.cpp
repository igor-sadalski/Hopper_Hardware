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

#define MAX 128
#define PORT 8888
#define SA struct sockaddr

using namespace Eigen;
using namespace Hopper_t;

vector_3t Kp_gains = {10,10,10};
vector_3t Kd_gains = {1,1,1}; //?

matrix_3t cross(vector_3t q) {
    matrix_3t c;
    c << 0, -q(2), q(1),
            q(2), 0, -q(0),
            -q(1), q(0), 0;
    return c;
}

struct State {
  scalar_t x;
  scalar_t y;
  scalar_t z;
  scalar_t q_w;
  scalar_t q_x;
  scalar_t q_y;
  scalar_t q_z;
} state;

void TokenizeStringToFloats(char str[], float currents[]){
    char * pch;
    pch = strtok (str,",");
    int i = 0;
    while (pch != NULL)
    {
        currents[i] = strtof (pch, NULL);
        i++;
        pch = strtok (NULL, ",");
    }
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
void signal_callback_handler(int signum) {
	std::cout << "Caught signal " << signum << std::endl;
   // Terminate program
   exit(signum);
}

int main(int argc, char **argv)
{

	bool fileWrite = true;
        std::string dataLog = "../data/data.csv";
	std::ofstream fileHandle;
        fileHandle.open(dataLog);
	fileHandle << "t,x,y,z,q_w,q_x,q_y,q_z" << std::endl;


	quat_t quat_d;
	quat_t quat_a;
	vector_3t omega_a;
	vector_3t omega_d;
	vector_3t torque;
	scalar_t Q_w, Q_x, Q_y, Q_z;
	scalar_t w_x, w_y, w_z;
	char buff[MAX];
	char send_buff[MAX] = "<{0,0,0}>";
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


	//cannot use spaces or special characters in the string message as the program otherwise freaks out
	
	   
        std::chrono::high_resolution_clock::time_point t1;
        std::chrono::high_resolution_clock::time_point t_start;
        std::chrono::high_resolution_clock::time_point t2;

	t_start = std::chrono::high_resolution_clock::now();
	//ros::init(argc, argv, "listener");
	//ros::NodeHandle n;
	//ros::Subscriber sub = n.subscribe("/vrpn_client_node/hopper/pose", 200, chatterCallback);

	write(sockfd, "ok", 3);        

	signal(SIGINT, signal_callback_handler);
	while(1){
		//ros::spinOnce();
		t1 = std::chrono::high_resolution_clock::now();
		bzero(buff, sizeof(buff));
		
		//receive string states, ESP8266 -> PC
		read(sockfd, buff, sizeof(buff));
		//printf("%s \n", buff);	
		
		//assume tokenization on ,
		TokenizeStringToFloats(buff, states);
		if (!init) {
		  state_init << states[0], states[1], states[2], states[3], states[4], states[5], states[6], states[7], states[8], states[9]; // save the initial condition
		  quat_init = Quaternion<scalar_t>(states[6], states[7], states[8], states[9]);
		  init = true;
		}
		state_vec << states[0], states[1], states[2], states[3], states[4], states[5], states[6], states[7], states[8], states[9];

		std::cout <<"Global state: " << state.x << ", " << state.y << ", " << state.z <<std::endl;
		
		std::cout << "state: " << state_vec.transpose() << std::endl;
		w_x = states[3];
		w_y = states[4];
		w_z = states[5];				
		Q_w = states[6];
		Q_x = states[7];
		Q_y = states[8];
		Q_z = states[9];	
		quat_d = Quaternion<scalar_t>(1,0,0,0);
		quat_a = Quaternion<scalar_t>(Q_w,Q_x,Q_y,Q_z);
		quat_a = quat_init.inverse()*quat_a;
		omega_d << 0,0,0; 
		omega_a << w_x, w_y, w_z;
		computeTorque(quat_a, quat_d, omega_a, omega_d, torque);
		bzero(send_buff, sizeof(send_buff));
		sprintf(send_buff,"<{%f,%f,%f}>",torque(0)/const_wheels, torque(1)/const_wheels, torque(2))/const_wheels;	
		//std::cout << "torque: " << torque.transpose()/const_wheels << std::endl;
		
		write(sockfd, send_buff, sizeof(send_buff));		
		t2 = std::chrono::high_resolution_clock::now();
                std::cout <<"Timing: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count()*1e-6 << "[ms]" << "\n";

		if (fileWrite) {
		  fileHandle << std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t_start).count()*1e-9 << "," << state.x << ", " << state.y << ", " << state.z<<","<< state.q_w << ","<<state.q_x<<","<<state.q_y<<","<<state.q_z<<std::endl; 
		}
	}

	close(sockfd);
}

//roslaunch vrpn_client_ros fast.launch server:=192.168.1.5
