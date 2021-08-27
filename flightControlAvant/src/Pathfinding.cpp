#include "Pathfinding.h"

using namespace std;

vector <Point> pathfinding_Astar(double lat_drone, double lon_drone, double h_drone, double lat_dest, double lon_dest, double h_dest, int q_obs){
    //Fazer o construtor e destrutor copia da malha pode ser uma otima ideia

    //Iicializacao dos valores
    Malha M = transf(lat_drone, lon_drone, h_drone);
    Point destiny(lat_dest, lon_dest, h_dest);
    int arrived = 0; //colocar bool
    int pos = -1;
    int pos_aux;
    double dist_pts;
    double distance;
    double distance_aux;
    vector <Point> points_list;
    //vector <Point> dijkstra_list;

    //Colocando obstaculo aleatorio
    //gen_rand_obs(M, q_obs);

    //          

    // 20        * * * * * * * * * * * * * * * * * * * * *
    // 19        * e   *                                 *
    // 18        *     *                                 *
    // 17        *     *   * * * * * * * * * * * * *     *
    // 16        *     *               *           *     *
    // 15        *     * * * * * *           *     *     *
    // 14        *     *         * * * * * * * *   *     *
    // 13        *     *   *     *         *   *   *     *
    // 12        *     *   *     * * * * * *   *   *     *
    // 11        *     *   *             * *   *   *     *
    // 10        *     *   *         b   * *   *   *     *
    // 9         *     *   *             * *   *   *     *
    // 8         *     *   *   * * * * * * *       *     *
    // 7         *     *   *               *   * * *     *
    // 6         *     *   *               *   *         *
    // 5         *     *   * * * * * * * * *   *         *
    // 4         *     *                       *         *
    // 3         *     0 * * * * * * * * * * * *         *
    // 2         *                                       *
    // 1         *                                       *
    // 0         * * * * * * * * * * * * * * * * * * * * *

    int pontoInicial = 21 * 3 + 3;
    for (int i = 0; i < 7; i++) {
        // linha0
        // linha1
        // linha2 
        // linha3
        M.malha[pontoInicial + 21 * 0 + 0 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 0 + 1 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 0 + 2 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 0 + 3 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 0 + 4 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 0 + 5 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 0 + 6 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 0 + 7 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 0 + 8 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 0 + 9 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 0 + 10 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 0 + 11 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 0 + 12 + 441 * i].set_obs_true();
        // linha4
        M.malha[pontoInicial + 21 * 1 + 0 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 1 + 12 + 441 * i].set_obs_true();
        // linha5
        M.malha[pontoInicial + 21 * 2 + 0 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 2 + 2 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 2 + 3 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 2 + 4 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 2 + 5 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 2 + 6 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 2 + 7 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 2 + 8 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 2 + 9 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 2 + 10 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 2 + 12 + 441 * i].set_obs_true();
        // linha6
        M.malha[pontoInicial + 21 * 3 + 0 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 3 + 2 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 3 + 10 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 3 + 12 + 441 * i].set_obs_true();
        // linha7
        M.malha[pontoInicial + 21 * 4 + 0 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 4 + 2 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 4 + 10 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 4 + 12 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 4 + 13 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 4 + 14 + 441 * i].set_obs_true();
        // linha8
        M.malha[pontoInicial + 21 * 5 + 0 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 5 + 2 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 5 + 4 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 5 + 5 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 5 + 6 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 5 + 7 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 5 + 8 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 5 + 9 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 5 + 10 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 5 + 14 + 441 * i].set_obs_true();
        // linha9
        M.malha[pontoInicial + 21 * 6 + 0 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 6 + 2 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 6 + 9 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 6 + 10 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 6 + 12 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 6 + 14 + 441 * i].set_obs_true();
        // linha10
        M.malha[pontoInicial + 21 * 7 + 0 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 7 + 2 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 7 + 9 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 7 + 10 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 7 + 12 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 7 + 14 + 441 * i].set_obs_true();
        // linha11
        M.malha[pontoInicial + 21 * 8 + 0 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 8 + 2 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 8 + 9 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 8 + 10 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 8 + 12 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 8 + 14 + 441 * i].set_obs_true();
        // linha12
        M.malha[pontoInicial + 21 * 9 + 0 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 9 + 2 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 9 + 5 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 9 + 6 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 9 + 7 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 9 + 8 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 9 + 9 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 9 + 10 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 9 + 12 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 9 + 14 + 441 * i].set_obs_true();
        // linha13
        M.malha[pontoInicial + 21 * 10 + 0 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 10 + 2 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 10 + 5 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 10 + 10 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 10 + 12 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 10 + 14 + 441 * i].set_obs_true();
        // linha14
        M.malha[pontoInicial + 21 * 11 + 0 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 11 + 5 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 11 + 6 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 11 + 7 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 11 + 8 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 11 + 9 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 11 + 10 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 11 + 11 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 11 + 12 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 11 + 14 + 441 * i].set_obs_true();
        // linha15
        M.malha[pontoInicial + 21 * 12 + 0 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 12 + 1 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 12 + 2 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 12 + 3 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 12 + 4 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 12 + 5 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 12 + 11 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 12 + 14 + 441 * i].set_obs_true();
        // linha16
        M.malha[pontoInicial + 21 * 13 + 0 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 13 + 8 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 13 + 14 + 441 * i].set_obs_true();
        // linha17
        M.malha[pontoInicial + 21 * 14 + 0 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 14 + 2 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 14 + 3 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 14 + 4 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 14 + 5 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 14 + 6 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 14 + 7 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 14 + 8 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 14 + 9 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 14 + 10 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 14 + 11 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 14 + 12 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 14 + 13 + 441 * i].set_obs_true();
        M.malha[pontoInicial + 21 * 14 + 14 + 441 * i].set_obs_true();
        // linha18
        M.malha[pontoInicial + 21 * 15 + 0 + 441 * i].set_obs_true();
        // linha19
        M.malha[pontoInicial + 21 * 16 + 0 + 441 * i].set_obs_true();
        //linha20
    }
    //Vizinhos sempre são <posicao do vetor> + 1 / <pv> - 1 / <pv> + (2* n_lat +1) / <pv> - (2*n_lat + 1).
    //Drone começa na posicao do vetor (tamanho do vetor)/2 ou seja em x,y,z = 0,0,0.
    //M.print_malha();

    do{
        if(pos == -1){
            pos = M.get_malha().size()/2;
            M.malha[pos].set_peso(0); //Define o peso do primeiro ponto como 0.
            distance = dist_2pts(M.malha[pos], destiny);
            M.malha[pos].set_real_dist(distance);
        }
        else{

            cout << "Ponto sendo olhado: " << endl;
            M.malha[pos].print_point();
            cout << endl;

           cout << "Vizinhos: " << endl;

            //RELAXAMENTO DOS VIZINHOS?
            pos_aux = pos - 1; //ponto "a esquerda do drone"
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso(); //esse e o peso certo?
                distance_aux = dist_2pts(M.malha[pos_aux], destiny); //esse auxiliar precisa?
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){ //pq esse realdist?

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + 1; //ponto "a direita do drone"
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2 + 1); //ponto "atras do drone"
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2 + 1); //ponto "a frente do drone"
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2); //ponto "esquerda-atrás do drone"
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2 + 2); //ponto "direita-atrás do drone"
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2); //ponto "esquerda-frente do drone"
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2 + 2); //ponto "direita-frente do drone"
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2 + 1)*(M.get_N_lon()*2 + 1); //ponto acima do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2 + 2)*(M.get_N_lon()*2 + 1);//ponto acima e frente do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2 + 2)*(M.get_N_lon()*2 + 1) + 1; //ponto acima e diagonal frente-direita do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2 + 2)*(M.get_N_lon()*2 + 1) - 1; //ponto acima e diagonal frente-esquerda do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2)*(M.get_N_lon()*2 + 1); //ponto acima e atrás do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2)*(M.get_N_lon()*2 + 1) + 1; //ponto acima e giagonal atrás-direita do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2)*(M.get_N_lon()*2 + 1) - 1; //ponto acima e diagonal atrás-esquerda do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2 + 1)*(M.get_N_lon()*2 + 1) + 1; //ponto acima e direita do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2 + 1)*(M.get_N_lon()*2 + 1) - 1; //ponto acima e esquerda do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2 + 1)*(M.get_N_lon()*2 + 1);//ponto abaixo do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2 + 1)*(M.get_N_lon()*2 + 1) + 1; //ponto abaixo e direita do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2 + 1)*(M.get_N_lon()*2 + 1) - 1; //ponto abaixo e esquerda do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2 + 2)*(M.get_N_lon()*2 + 1); //ponto abaixo e atrás do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2 + 2)*(M.get_N_lon()*2 + 1) + 1; //ponto abaixo e atrás-direita do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2 + 2)*(M.get_N_lon()*2 + 1) - 1; //ponto abaixo e atrás-esquerda do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2)*(M.get_N_lon()*2 + 1); //ponto abaixo e frente do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2)*(M.get_N_lon()*2 + 1) + 1; //ponto abaixo e frente-direita do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2)*(M.get_N_lon()*2 + 1) - 1; //ponto abaixo e frente-esquerda do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            M.malha[pos].set_fechado(); //fechando o ponto

            //ele ta fazendo copia aqui????//dapra melhorar esse algoritmo
            pos = pega_pos(M.malha, pos); //Agora ele olha só na malha, procurando pelo menor peso e se é fechado ou não

            cout << "próximo ponto a ser olhado: " << endl;
            M.malha[pos].print_point();
            cout << endl;


            //M.malha[pos].print_point();

            distance = dist_2pts(M.malha[pos], destiny);

            cout << distance << endl;

            //system("pause");
           // cout << endl;

            //cout << distance << endl;

            //Condição de parada. Checar sobre distance < 8.
            if (distance < 8){
				arrived = 1;
			}

			//dijkstra_list.clear();
        }
    }while(arrived == 0);

    //Organizar o vetor de pontos, começando do destino e "retraçando" o caminho até o início.
    int pos_aux2 = -1;
    do{
        points_list.push_back(M.malha[pos]);
        pos = M.malha[pos].get_pto_ant();
        pos_aux2 = pos;
    }while(pos_aux2 != M.malha.size()/2);

    for (Point it : M.get_malha()) {
        if (it.get_obs() == 1) {
            points_list.push_back(it);
        }
    }

    return points_list;
}
