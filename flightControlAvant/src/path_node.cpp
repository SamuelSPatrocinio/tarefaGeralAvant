#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"

#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <iostream>
#include "Pathfinding.h"
#include <cmath>
#include <vector>

//funcoes para converter valores X e Y em metros para coordenadas no plano cartesiano da malha
double conversor_X(double lat_drone, double X) {
    double x_coord = lat_drone + X * 0.000009;
    return x_coord;
}
double conversor_Y(double lon_drone, double Y) {
    double y_coord = lon_drone + Y * 0.000009;
    return y_coord;
}

//Vetor que recebera os pontos do pathfinding
vector <Point> N;

// ----------------------------------------------------------
// Global Variables
// ----------------------------------------------------------
double rotate_y = 0;
double rotate_x = 0;
double rotate_z = 0;

// ----------------------------------------------------------
// main() function
// ----------------------------------------------------------
vector<vector<float>> getPath() {

    vector<vector<float>> coordinates;

    //------ PARTE DO MAIN DE TESTE PARA PATHFINDING PRECISÃO DE +-8 METROS------ //
    cout.precision(10); //apenas aumentando a precisao do cout

    /*
    Sobre a malha default: *DESATUALIZADA PEDIR A CONTI PRA ATUALIZAR SE NECESSÁRIO*
    O tamanho da malha é definido em Point.cpp pelos valores de N_lat, N_lon e N_H
    A malha default colocamos N_lat = 10, N_lon = 10, N_H = 4, e terá (10*2 + 1)*(10*2 + 1)*(4*2 + 1) = 3969 pontos que formam a malha
    O espaçamento entre cada pontos é 15 metros, logo a malha em 2D corresponde à (10*2 + 1)*15*(10*2 + 1)*15 = 99225 m²
    Em 3D: 99225*(4*2 + 1)*15 = 13395375 m³
    Nesse caso, 315 m de largura, 315 m de comprimento e 135 m  de altura
    Válido lembrar que o drone, está sempre no centro desse paralelepípedo
    */


    //Mude para os valores desejados
    //Os valores X,Y,Z do drone sao sempre 0,0,0
    double latitude_drone = -19.871875;
    double longitude_drone = -43.970220;
    double altura_drone = 60;

    //insira os valores desejados de x,y em metros sendo que -600 < x,y < 600. Insira também a quantidade de obstáculos desejados
    double x_metros = -135;
    double y_metros = 135;
    double z_metros = 15;
    int quantidade_obstaculo = 0;

    //--------------------------------------------------------------------------------------------------------------------------

    double latitude_destino = conversor_X(latitude_drone, x_metros); // Essa latitude equivale ao X = 600, Extremo "esquerdo"
    double longitude_destino = conversor_Y(longitude_drone, y_metros); // Essa longitude equivale ao Y = 600, Extrema "cima"
    double altura_destino = z_metros; // Não mude a altura, já que está testando apenas 2D

    N = pathfinding_Astar(latitude_drone, longitude_drone, altura_drone, latitude_destino, longitude_destino, altura_destino, quantidade_obstaculo);

    //Funcao que imprime quais pontos o drone vai passar
    cout << "imprimindo o caminho" << endl;
    for (int i = 0; i < N.size(); i++) {
        if (N[i].get_obs() == 1) {
            cout << "Obstaculo em: \n";
            N[i].print_point();
            cout << endl;
        }
        else {
            N[i].print_point();
	    vector<float> point;
	    point.push_back((float)N[i].get_x());
	    point.push_back((float)N[i].get_y());
	    point.push_back((float)N[i].get_z());
	    coordinates.push_back(point);
            cout << endl;
        }
    }
    
    return coordinates;

}

int main(int argc, char **argv){

	ros::init(argc, argv, "path_node");

	ros::NodeHandle nodeHandler;

	ros::Rate rate(1.0);

	ros::Publisher pathPublisher = nodeHandler.advertise<std_msgs::Float32MultiArray>("path",1000);

	std_msgs::Float32MultiArray path;

	vector<vector<float>> coordinates = getPath();

	for(int i = 0; i < coordinates.size() ; i++){
		for(int j = 0; j < coordinates[i].size() ; j++){
			path.data.push_back(coordinates[i][j]);
		}
	}

	for(int i = 0; i < path.data.size() ; i++){
		ROS_INFO("%f",path.data[i]);
	}
	


	while(ros::ok()){
		pathPublisher.publish(path);
		ROS_INFO("Published");
		ros::spinOnce();
		//rate.sleep();
	}
}
