#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "std_msgs/Float32MultiArray.h"
#include <math.h>
#include <vector>

#define gpsXpos gpsPosition.pose.position.x
#define gpsYpos gpsPosition.pose.position.y
#define gpsZpos gpsPosition.pose.position.z

#define xPos pose.pose.position.x
#define yPos pose.pose.position.y
#define zPos pose.pose.position.z




mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
geometry_msgs::PoseStamped gpsPosition;
void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	gpsPosition = *msg;
}
std::vector<float> currentPath;
void getCurrentPath(const std_msgs::Float32MultiArray::ConstPtr& array){
	if(currentPath.size()==0){
		for(std::vector<float>::const_iterator it = array->data.begin() ; it != array->data.end() ; ++it){
			currentPath.push_back(*it);
		}
	}
}


int main(int argc, char **argv){

	ROS_INFO("Starting...");

	ros::init(argc, argv, "decolagem");

	ros::NodeHandle nodeHandler;

	ros::Rate rate(20.0);

//criando um subscriber para o path
	ros::Subscriber pathSubscriber = nodeHandler.subscribe<std_msgs::Float32MultiArray>("/path",1000,getCurrentPath);
//criando um subcriber para o estado do drone
	ros::Subscriber state_sub = nodeHandler.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

	ROS_INFO("Esperando conexao");

	while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    	}

	ROS_INFO("Conectado");

//criando um subscriber para o gps do drone
	ros::Subscriber gpsSubscriber = nodeHandler.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 100, gpsCallback);


//armar o drone
	//criando o client de armamento
	ros::ServiceClient armingClient = nodeHandler.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	//armando o drone
	mavros_msgs::CommandBool armCommand;
    	armCommand.request.value = true;
	armingClient.call(armCommand);
	//Mostrando que o drone foi armado
	if(armCommand.response.success){
		ROS_INFO("Armado");
	}
//decolar o drone
	//criando o client de modo de voo
	ros::ServiceClient flightModeClient = nodeHandler.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	//criando publisher do destino do drone
	ros::Publisher destinationPublisher = nodeHandler.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

	//setando as coordenadas de destino
	int i = currentPath.size()-1;
	geometry_msgs::PoseStamped pose;
	xPos = -0.5;
	yPos = 0.5;
	zPos = 2;
	//como a controladora acabou de receber o comando de armar, eh preciso esperar um tempo
	for(int i = 10; ros::ok() && i > 0; --i){
		destinationPublisher.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}
	//decolando o drone
	mavros_msgs::SetMode flightMode;
    	flightMode.request.custom_mode = "OFFBOARD";
	flightModeClient.call(flightMode);
	//Mostrando que o drone decolou
	if(flightMode.response.mode_sent){
		ROS_INFO("Decolando");
	}
	
	float interval = 0.2;
	while(ros::ok()){
		//esperar o drone chegar ao destino
		while(gpsXpos > xPos+interval || gpsXpos < xPos-interval ||
		      gpsYpos > yPos+interval || gpsYpos < yPos-interval ||
		      gpsZpos > zPos+interval || gpsZpos < zPos-interval ){
		      destinationPublisher.publish(pose);
		      ros::spinOnce();
		}
		ROS_INFO("Chegou ao destino");
		ROS_INFO("Estabelecendo um novo destino");
		//setar o novo destino
		
		//2 5 8 11 14
		ROS_INFO("I: %d",i);
		xPos = -0.5 + currentPath[i-2]/15;
		yPos = 0.5 + currentPath[i-1]/15;
		if(3 + currentPath[i]/15 <= 0){
			zPos = 0.5; //evita que o drone se arraste no chao
		}
		else{
			zPos = 3 + currentPath[i]/15;
		}
		
		if(i>2){
			i-=3;
		}	
		ROS_INFO("Novo destino: x: %f y: %f z: %f", pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
		ROS_INFO("Atualmente em: x: %f y: %f z: %f", gpsXpos,gpsYpos,gpsZpos);
      }
}

