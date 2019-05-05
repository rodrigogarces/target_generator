#define FUNCAOPP
#define PARAM_PLF
#define FIREFLY

#include "TargetGenerator.h"

TargetGenerator::TargetGenerator(ros::NodeHandle *nodehandle) : nh_(*nodehandle)
{
    ROS_INFO("Inicializando nó TargetGenerator");
    initializeSubscribers();
    initializePublishers();

    dmin = (double *)malloc(2 * sizeof(double));
    dmax = (double *)malloc(2 * sizeof(double));

    destino_corrente[0] = 0;
    destino_corrente[1] = 0;

    MaxEpocas = 250; // 150
    NumFireflies = 40;

    first_time = true;
    Aval = 0;
}

void TargetGenerator::initializeSubscribers()
{
    ROS_INFO("inicializando os subscribers");

    map_sub_ = nh_.subscribe("/map", 1, &TargetGenerator::mapCallback, this); // Subscreve no topico que recebe o OG do ambiente

    curr_pose_sub_ = nh_.subscribe("/vrep_ros_interface/pose", 1,
                                   &TargetGenerator::currPoseCallback, this); // Subscreve tópico que recebe a posição corrente do robô
    costmap_sub_ = nh_.subscribe("/move_base_node/global_costmap/costmap", 1,
                                 &TargetGenerator::costmapCallback, this);
}

void TargetGenerator::initializePublishers()
{
    ROS_INFO("inicializando os publishers");
    path_pub_ = nh_.advertise<std_msgs::String>("/path_plan", 1, true); // Publica o caminho para o alvo

    target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, true); // Publica o caminho para o alvo

    finalize_pub_ = nh_.advertise<std_msgs::String>("/finalize_mapping", 1, true); // Publica flag que para o mapeamento
}

void TargetGenerator::initializeServices()
{
}

void TargetGenerator::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    w = map->info.width;                    // Largura do mapa
    h = map->info.height;                   // Altura do mapa
    pixel_size = map->info.resolution;      // resolução do mapa
    origin_x = map->info.origin.position.x; // coord. x da origem do mapa
    origin_y = map->info.origin.position.y; // coord. y da origem do mapa

    dmin[0] = 0;
    dmin[1] = 0;
    dmax[0] = pixel_size * h;
    dmax[1] = pixel_size * w;

    if (first_time)
    {
        inicializeStateMatrices();
        first_time = false;
    }

    dilateDangerRegion(map);
    loadAuxiliaryLists(map);
    avaliateStoppingCriteria();

#ifdef PRINTMAP
    printMap();
#endif

    getRobotPosition();

#ifdef SAVELANDSCAPE
    saveFunctionLandscape();
#endif

#ifdef SAVEMAP
    saveMap();
#endif

    int destino[2];
    double dis = 0.0;

    if (!first_time)
    {
        dis = sqrt((destino_corrente[0] - robot_position[0]) * (destino_corrente[0] - robot_position[0]) + (destino_corrente[1] - robot_position[1]) * (destino_corrente[1] - robot_position[1])); // distância entre o robô e o alvo
    }

    // -> Se a distância é menor que a distancia minima, o alvo não é atualizado
    if (dilatedMap[destino_corrente[1]][destino_corrente[0]] != 0)
        has_goal = false;

    //cout << endl
    //     << "distancia: " << dis << endl
    //     << endl;
    if (dis < DIST_MIN)
        has_goal = false;
    else
        has_goal = true;

    first_time = false;
    if (dilatedMap[destino_corrente[1]][destino_corrente[0]] != 0)
        has_goal = false;

    if (!has_goal)
    {

#ifdef BRUTEFORCE
        generateFunctionLandscape(w, h);
#endif

#ifdef FIREFLY
        FireflyAlgorithm();
#endif

#ifdef GENETICO
        GeneticAlgorithm();
#endif

#ifdef DOWNHILLSIMPLEX
#endif

#ifdef MULTISTARTGRANDIENT
#endif

        publishNewTarget();
    }
}

void TargetGenerator::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &cmap)
{
}

void TargetGenerator::currPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &curr_pose)
{
}

void TargetGenerator::finalizeExploration()
{
    std_msgs::String finalize;
    finalize.data = "1";
    finalize_pub_.publish(finalize);

    FILE *fpath = fopen("path.txt", "a");

    fprintf(fpath, "\n");
    fprintf(fpath, "[");
    for (int i = 0; i < vi; i++)
    {
        fprintf(fpath, "%d, %d; ", visitados[i][0], visitados[i][1]);
    }
    fprintf(fpath, "]\n");
    fclose(fpath);

    FILE *fpath2 = fopen("aval.txt", "a");
    fprintf(fpath2, "Exp 1 => pathlenght: %d \t function calls: %d \n", vi, Aval);
    fclose(fpath2);

    FILE *ftime = fopen("avg_time.txt", "a");
    fprintf(ftime, "%lf\n", avg_time / (double)qtdeCA);
    fclose(ftime);

    FILE *faval = fopen("avg_aval.txt", "a");
    fprintf(faval, "%lf\n", avg_aval / (double)qtdeCA);
    fclose(faval);

    cout << "\nO MAPEAMENTO FINALIZOU.\n";
    system("rosnode kill SLAM");  // finaliza node SLAM
                                            // Outros nodes precisam ser finalizados 
    ros::shutdown();
    return;
}

void TargetGenerator::inicializeStateMatrices()
{
    // - Inicialização das matrizes de estado -

    _map = (int **)malloc(h * sizeof(int *));
    _mapobj = (float **)malloc(h * sizeof(float *));
    _mapa = (float **)malloc(h * sizeof(float *));
    dilatedMap = (int **)malloc(h * sizeof(int *));

    for (int i = 0; i < h; i++)
    {
        _map[i] = (int *)malloc(w * sizeof(int));
        _mapobj[i] = (float *)malloc(w * sizeof(float));
        _mapa[i] = (float *)malloc(w * sizeof(float));
        dilatedMap[i] = (int *)malloc(w * sizeof(int));
    }

    // - Atualização das listas de pontos -
    fronteira = new int *[w * h];     // Lista de pontos de fronteira
    regiao_perigo = new int *[w * h]; // Lista de pontos da regiao de perigo (ocupados ou próximos de ocupados)
    visitados = new int *[w * h];
    vi = 0;                           // Lista de pontos visitados
    desconhecidos = new int *[w * h]; // Lista de pontos desconhecidos

    //cout << "FIRST TIME\n"
    //     << endl;
}

void TargetGenerator::dilateDangerRegion(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            int pixel = map->data[i * w + j];

            _map[i][j] = pixel;
            dilatedMap[i][j] = pixel;
        }
    }

    // nav_msgs::GridCells rdanger;
    // std::string frameid = "/map";
    // rdanger.header.frame_id = frameid.c_str();
    // //rdanger.header.stamp = ros::Time::now();
    // rdanger.cell_width = pixel_size;
    // rdanger.cell_height = pixel_size;
    // rdanger.cells.clear();

    int pixel;
    int aux;
    int max;
    int ci = 0;

    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            pixel = _map[i][j];
            aux = 4;

            max = 0;
            double dis;
            for (int ei = i - aux; ei <= i + aux; ++ei)
            {
                for (int ej = j - aux; ej <= j + aux; ++ej)
                {
                    if (ei > 0 && ej > 0 && ei < h && ej < w)
                    {
                        if (_map[ei][ej] > max)
                        {
                            dis = sqrt((ei - i) * (ei - i) + (ej - j) * (ej - j));
                            if (dis < aux)
                            {
                                max = _map[ei][ej];
                            }
                        }
                    }
                }
            }

            dilatedMap[i][j] = max;

            if (_map[i][j] > 0)
            {
                dilatedMap[i][j] = _map[i][j];
            }

            // if (_map[i][j] > 0)
            // {
            //     geometry_msgs::Point c;
            //     //c.x = (float)i; c.y = (float)j; c.z = 0;

            //     c.y = (float)i * pixel_size + origin_x; //path[l][0];
            //     c.x = (float)j * pixel_size + origin_y; //path[l][1];
            //     c.z = 0;

            //     rdanger.cells.push_back(c);
            // }
            //else dilatedMap[i][j] = pixel;

            if (_map[i][j] < 0)
                dilatedMap[i][j] = _map[i][j];
        }
    }

    //grid_cells_pub.publish(rdanger);
}

void TargetGenerator::loadAuxiliaryLists(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    fi = 0;
    ri = 0;
    di = 0;

    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            //int pixel = dilatedMap[i][j];

            int pixel = _map[i][j];

            //_map[i][j] = pixel;
            //dilatedMap[i][j] = pixel;

            if (pixel == -1)
            {
                desconhecidos[di] = new int[2]{i, j};
                di++;
            }
            else
            {
                if (pixel > 0)
                {
                    regiao_perigo[ri] = new int[2]{i, j};
                    ri++;
                }
                else
                { // Procura pontos desconhecidos adjacentes
                    int l = j > 0 ? map->data[i * w + (j - 1)] : 0;
                    int r = j < h - 1 ? map->data[i * w + (j + 1)] : 0;
                    int u = i > 0 ? map->data[(i - 1) * w + j] : 0;
                    int d = i < w - 1 ? map->data[(i + 1) * w + j] : 0;

                    if (l == -1 || r == -1 || u == -1 || d == -1)
                    {
                        int p[2] = {i, j};
                        fronteira[fi] = new int[2]{i, j};
                        fi++;
                    }
                }
            }
        }
    }

    cout << "\nTESTE DE PARADA.\n";
    cout << "numero de desconhecidos: " << di << endl;
}

void TargetGenerator::avaliateStoppingCriteria()
{
    // O critério de parada é: se a quantidade de desconhecidos é menor que um determinado valor;
    // O ideal seria qtde de desconhecidos == 0, mas devido a incerteza do SLAM sempre há pontos 
    // espalhados pelo mapa ainda desconhecidos. 

    int NoDesc;

#define AMB1
#ifdef AMB1
    NoDesc = 27270;
#endif

#ifdef AMB2
    NoDesc = 27890;
#endif

    if (di <= NoDesc)
    { // amb 1: 27250 / 27270 -- amb 2: 27890
        finalizeExploration();
    }
    else
    {
        std_msgs::String finalize;
        finalize.data = "0";
        finalize_pub_.publish(finalize);
        cout << "Publicou" << endl;
    }
}

void TargetGenerator::printMap()
{
    //fclose(fo);
    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            cout << _map[i][j] << " ";
        }
        cout << endl;
    }
}

void TargetGenerator::saveMap()
{
    FILE *fo = fopen("_map_.txt", "w");

    fprintf(fo, "[");
    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            if (_map[i][j] > 0)
                fprintf(fo, " %d", 1);
            else
                fprintf(fo, " %d", 0);
        }

        if (i < h - 1)
        {
            fprintf(fo, ";\n");
        }
    }
    fprintf(fo, "]");
    fclose(fo);
}

void TargetGenerator::getRobotPosition()
{
    tf::TransformListener listener;
    tf::StampedTransform transform;

    try
    {
        listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    geometry_msgs::Twist vel_msg;

    pose_x = transform.getOrigin().getX();
    pose_y = transform.getOrigin().getY();

    int grid_x = ((pose_x - origin_x) / pixel_size); // posição x no Ocuppancy Grid
    int grid_y = ((pose_y - origin_y) / pixel_size); // posição y no Ocuppancy Grid

    robot_position[0] = grid_x;
    robot_position[1] = grid_y;

    visitados[vi] = new int[2]{robot_position[1], robot_position[0]};
    vi++; // Adiciona um ponto a lista de visitados
}

void TargetGenerator::saveFunctionLandscape(int w, int h)
{
    FILE *fo = fopen("fobj.txt", "w");
    generateFunctionLandscape(w, h);

    fprintf(fo, "[");
    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            fprintf(fo, "\t %.4f", _mapobj[i][j]);
        }

        if (i < h - 1)
        {
            fprintf(fo, ";\n");
        }
    }
    fprintf(fo, "]");
    fclose(fo);
}

void TargetGenerator::generateFunctionLandscape(int w, int h)
{
    // Função Objetivo
    float melhor = 1000000;

    /*if (!has_goal)
    {
        destino_corrente[0] = robot_position[0];
        destino_corrente[1] = robot_position[1];
    }*/

    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            double pixel[2] = {(double)i, (double)j};

            if (_map[i][j] != -1)
            {
                float distF = 0;
                double rp[2] = {(double)robot_position[1], (double)robot_position[0]};
                double dp[2] = {(double)destino_corrente[1], (double)destino_corrente[0]};
                float dr = distance(pixel, rp);
                //std::cout << "DISTANCIA " << dr << std::endl;

                for (int f = 0; f < fi; ++f)
                {
                    double pf[2] = {(double)fronteira[f][0], (double)fronteira[f][1]};
                    double d = distance(pixel, pf);
                    distF += exp(-d / (2 * w1 * w1));
                }

                float distR = 0;
                for (int r = 0; r < ri; ++r)
                {
                    double pr[2] = {(double)regiao_perigo[r][0], (double)regiao_perigo[r][1]};
                    float d = distance(pixel, pr);
                    distR += exp(-d / (2 * w2 * w2));
                }

                float distV = 0;
                for (int v = 0; v < vi; ++v)
                {
                    double pv[2] = {(double)visitados[v][0], (double)visitados[v][1]};
                    float d = distance(pixel, pv);
                    distV += exp(-d / (2 * w3 * w3));
                }

#ifdef FUNCAOPP
                float lambda = 0.1;
                _mapobj[i][j] = (-alpha * distF + beta * distR + rho * distV) / (lambda * dr);
#endif

#ifdef FUNCAOPP_2
                float lambda = 0.1;
                _mapobj[i][j] = (-alpha * distF + beta * distR + rho * distV) / (lambda * dr);
#endif

#ifdef FUNCAOPC
                double k = 30;
                if (dr < k)
                    _mapobj[i][j] = -alpha1 * distF + beta * distR + rho * distV;
                else
                    _mapobj[i][j] = -alpha2 * distF + beta * distR + rho * distV;

#endif

#ifdef FUNCAOPL
                float lambda = 10;
                _mapobj[i][j] = -alpha * distF + beta * distR + rho * distV + lambda * dr;
#endif
            }
            else
            {
                _mapobj[i][j] = 100000000;
            }

            if (melhor > _mapobj[i][j])
            {
                melhor = _mapobj[i][j];

#ifdef BRUTEFORCE
                destino_corrente[0] = j;
                destino_corrente[1] = i;
#endif
            }
        }
    }

    cout << endl
         << "############ MELHOR #############" << endl
         << melhor << endl
         << "#################################" << endl;
}

void TargetGenerator::publishNewTarget()
{
    geometry_msgs::PoseStamped _msg;
    //msg.header.stamp = ros::Time();
    std::string frame = "/map";
    _msg.header.frame_id = frame.c_str();

    _msg.pose.position.x = destino_corrente[0] * pixel_size + origin_x; //path[l][0];
    _msg.pose.position.y = destino_corrente[1] * pixel_size + origin_y; //path[l][1];

    cout << _msg.pose.position.x << " " << _msg.pose.position.y << endl;
    _msg.pose.position.z = 1;
    _msg.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    target_pub_.publish(_msg);
}

double TargetGenerator::distance(double *p1, double *p2)
{
    double ssd = 0.0;
    for (int i = 0; i < 2; i++)
        ssd += (p1[i] - p2[i]) * (p1[i] - p2[i]);
    return sqrt(ssd) / pixel_size;
}

void TargetGenerator::FireflyAlgorithm()
{
    // Implementa Firefly's Algorithm

    double B0 = 1;      // 1
    double gamma = 0.4; // 0.4
    double alpha = 0.3; // 0.1

    double bestFitness = 10000000.0;
    double *bestPosition;

    Firefly *f;

    bestPosition = (double *)malloc(2 * sizeof(double));

    fast_srand(time(NULL));
    double r01;

    int ini = 0, i, j, k;

    bool hasChange;
    double beta;

    struct timeval inicio, final;
    double tmili;

    gettimeofday(&inicio, NULL);

    double newpos[2];

#ifdef FIREFLY2
    ini = NumClusters;
#endif

    for (i = 0; i < NumFireflies; i++)
    {
        if (i >= ini)
        {
            f = new Firefly();
            for (j = 0; j < 2; j++)
            {
                r01 = ((double)(fastrand() % 101) / 100);
                newpos[j] = (dmax[j] - dmin[j]) * r01 + dmin[j]; // inicializando as posicoes aleatoriamente
            }

            f->SetPosicao(newpos);
            f->bposition = newpos;

            populacao.push_back(f);
        }
    }

    // if (hc)
    // {
    //     populacao[NumFireflies] = campeao;
    // }

    gettimeofday(&final, NULL);
    tmili = (1 * (final.tv_sec - inicio.tv_sec) * 1000 + (final.tv_usec - inicio.tv_usec) / 1);

    int bepoca = 0;
    int epoca = 0;
    //n = 0;

    double r;
    double Ii;
    double Ij;

    gettimeofday(&inicio, NULL);

    Firefly *atual;

    while (epoca < MaxEpocas)
    {
        hasChange = false;

        // Evaluate:
        for (i = 0; i < NumFireflies; i++)
            populacao[i]->fitness = ObjectiveFunction(populacao[i]->GetPosicao(), _step);
        //n++;

        heapSort();

        // Move:
        for (i = 0; i < NumFireflies; i++)
        {
            double *ipos = populacao[i]->GetPosicao();
            for (j = 0; j < NumFireflies; j++)
            {
                double *jpos = populacao[j]->GetPosicao();

                r = (ipos[0] - jpos[0]) * (ipos[0] - jpos[0]) +
                    (ipos[1] - jpos[1]) * (ipos[1] - jpos[1]);

                r = sqrt(r) / pixel_size;

                Ii = populacao[i]->fitness;
                Ij = populacao[j]->fitness;

                if (Ij < Ii)
                {
                    hasChange = true;

                    beta = B0 * exp(-gamma * r * r);

                    for (k = 0; k < 2; k++)
                    {
                        //r01 = ((double)(fastrand() % 101) / 100);
                        ;

                        newpos[k] = ipos[k] + beta * (jpos[k] - ipos[k]) + alpha * (r01 - 0.5) * (dmax[k] - dmin[k]);

                        if (newpos[k] < dmin[k])
                            newpos[k] = (dmax[k] - dmin[k]) * ((double)(fastrand() % 101) / 100) + dmin[k];
                        if (newpos[k] > dmax[k])
                            newpos[k] = (dmax[k] - dmin[k]) * ((double)(fastrand() % 101) / 100) + dmin[k];
                    }
                    populacao[i]->SetPosicao(newpos);
                }

                if (populacao[i]->bfitness > populacao[i]->fitness)
                {
                    populacao[i]->bfitness = populacao[i]->fitness;
                    populacao[i]->bposition = populacao[i]->GetPosicao();
                }
            }

            if (populacao[i]->fitness < bestFitness)
            {
                bestFitness = populacao[i]->fitness;
                for (k = 0; k < 2; k++)
                    bestPosition[k] = populacao[i]->GetPosicao()[k];

                //campeao = populacao[i];
                //hc = true;
                bepoca = epoca;
            }
        }
        epoca++;
    }

    if (bepoca > maxiEpocas)
    {
        maxiEpocas = bepoca;
    }

    if (bepoca < minEpocas)
    {
        minEpocas = bepoca;
    }

#ifdef FIREFLY2
    // vector<Firefly *> newpop(NumFireflies);

    // // Evaluate:
    // for (i = 0; i < NumFireflies; i++)
    //     populacao[i]->fitness = ObjectiveFunction(populacao[i]->GetPosicao(), _step);
    // //n++;

    // //  Sort:
    // SortPop();

    // newpop[0] = populacao[0];
    // k = 0;
    // double RC = 130.0;
    // for (i = 1; i < NumFireflies; i++)
    // {
    //     if (newpop[k]->GetPosicao()[0] != populacao[i]->GetPosicao()[0] ||
    //         newpop[k]->GetPosicao()[1] != populacao[i]->GetPosicao()[1])
    //     {
    //         if (Distance(newpop[k]->GetPosicao(), populacao[i]->GetPosicao()) > RC)
    //         {
    //             k++;
    //             newpop[k] = populacao[i];
    //         }
    //     }
    // }

    // for (i = 0; i < k; i++)
    // {
    //     populacao[i] = newpop[i];
    // }

    // NumClusters = k;
#endif

    destino_corrente[0] = (int)bestPosition[1] / pixel_size;
    destino_corrente[1] = (int)bestPosition[0] / pixel_size;
    //n = 0;
}

double TargetGenerator::ObjectiveFunction(double posicao[], int step)
{
    Aval++;
    int i = posicao[0] / pixel_size; //cout << "i -> " << i << endl;
    int j = posicao[1] / pixel_size; //cout << "j -> " << j << endl;

    double pixel[2] = {(double)i, (double)j};

    if (i >= h || i < 0 || j >= w || j < 0)
    {
        return 100000000000;
    }

    if (_map[i][j] > -1)
    {
        double distF = 0;
        double rp[2] = {(double)robot_position[1], (double)robot_position[0]};
        double dp[2] = {(double)destino_corrente[1], (double)destino_corrente[0]};
        double dr = Distance(pixel, rp);

        for (int f = 0; f < fi; ++f)
        {
            double pf[2] = {(double)fronteira[f][0], (double)fronteira[f][1]};
            double d = Distance(pixel, pf);
            distF += exp(-d / (2 * w1 * w1));
        }

        double distR = 0;
        for (int r = 0; r < ri; ++r)
        {
            double pr[2] = {(double)regiao_perigo[r][0], (double)regiao_perigo[r][1]};
            double d = Distance(pixel, pr);
            distR += exp(-d / (2 * w2 * w2));
        }

        double distV = 0;
        //cout << "\n=> VI => " << vi << endl;
        for (int v = 0; v < vi; ++v)
        {
            double pv[2] = {(double)visitados[v][0], (double)visitados[v][1]};
            double d = Distance(pixel, pv);
            distV += exp(-d / (2 * w3 * w3));
        }
        /*
        double distD = 0;

        //cout << "\nhere" << endl;
        for (int ii = pixel[0] - 5; ii<pixel[0] + 5; ii++)
        { 
            if (ii < 0 || ii >= h)  
              continue;
            //cout << "\nii -> " << ii << endl;
            for (int jj = pixel[1] - 5; jj<pixel[1] + 5; jj++)
            {

                if (jj < 0 || jj >= w)
                  continue;
                //cout << "\njj -> " << jj << endl;
                if (_mapa[ii][jj] == 0)
                  distD += 1;
                else if (_mapa[ii][jj] < 0)
                  distD -= 1;
            } 
        }
        */
        //distD = exp(-distD / (2 * w3*w3));

        //cout << "distF: " << distF << endl;
        //cout << "distR: " << distR << endl;
        //cout << "F : " << -alpha*distF + beta*distR + 100* rho*distV  << endl;
        //cout << "distD: " << distD << endl;

#ifdef FUNCAOPP
        float lambda = 0.1;
        return (-alpha * distF + beta * distR + rho * distV) / (lambda * dr);
#endif

#ifdef FUNCAOPC
        double k = 20;
        if (dr < k)
            return -alpha1 * distF + beta * distR + rho * distV;
        else
            return -alpha2 * distF + beta * distR + rho * distV;
#endif

#ifdef FUNCAOPL
        float lambda = 10;
        return -alpha * distF + beta * distR + rho * distV + lambda * dr;
#endif
    }
    else
    {
        return 1000000000000000;
    }
}

double TargetGenerator::Distance(double *p1, double *p2)
{
    double ssd = 0.0;
    for (int i = 0; i < 2; i++)
        ssd += (p1[i] - p2[i]) * (p1[i] - p2[i]);
    return sqrt(ssd) / pixel_size;
}

void TargetGenerator::SortPop()
{
    int i, j;

    // initialization of indexes
    for (i = 0; i < NumFireflies; i++)
        _Index[i] = i;

    // Bubble sort
    for (i = 0; i < NumFireflies - 1; i++)
    {
        for (j = i + 1; j < NumFireflies; j++)
        {
            if (populacao[i]->fitness > populacao[j]->fitness)
            {
                double z = populacao[i]->fitness;
                populacao[i]->fitness = populacao[j]->fitness;
                populacao[i]->fitness = z;

                int k = _Index[i]; // exchange indexes
                _Index[i] = _Index[j];
                _Index[j] = k;
            }
        }
    }
}

void TargetGenerator::ReplacePop()
{
    int i, j;
    vector<Firefly *> popTmp = populacao;

    for (i = 0; i < NumFireflies; i++)
    {
        populacao[i] = popTmp[_Index[i]];
    }
}

void TargetGenerator::heapSort()
{
    // Build heap (rearrange array)
    for (int i = NumFireflies / 2 - 1; i >= 0; i--)
        heapify(NumFireflies, i);

    // One by one extract an element from heap
    for (int i = NumFireflies - 1; i >= 0; i--)
    {
        // Move current root to end
        swap(populacao[0], populacao[i]);

        // call max heapify on the reduced heap
        heapify(i, 0);
    }
}

void TargetGenerator::heapify(int n, int i)
{
    int largest = i;   //initiaze largest as root
    int l = 2 * i + 1; // left = 2*i + 1
    int r = 2 * i + 2; // right = 2*i + 2

    // If right child is larger than largest so far
    if (r < n && populacao[r]->fitness > populacao[largest]->fitness)
        largest = r;

    // If largest is not root
    if (largest != i)
    {
        swap(populacao[i], populacao[largest]);

        // Recursively heapify the affected sub-tree
        heapify(n, largest);
    }
}

void TargetGenerator::GeneticAlgorithm()
{
    double bestFitness = 10000000.0;
    double *bestPosition;

    bestPosition = (double *)malloc(2 * sizeof(double));
    clock_t start, end;

    FILE *saida = NULL, *arq = NULL;
    char nomarq[50];

    int MaxIt = PASSOS, numGeracoes = MAXGER, numCruza;
    float pMuta = PMUTAC, xalfa = XALFA;
    double erro;

    Populacao P;
    int pa1, pa2;
    double fit, dvp, med;

    int i, j;

    int semente = 0;

#ifdef LINHA
    funcao = atoi(argv[2]);
    strcpy(nomarq, GARE);
    strcat(nomarq, "f.o");
    strcat(&nomarq[strlen("f.o.")], argv[1]);
    if (!(saida = fopen(nomarq, "w")))
    {
        perror("");
        exit(-1);
    }

    funcao = atoi(argv[2]);
    semente = atoi(argv[3]);
#endif

#ifdef NOLINHA
    saida = stdout;
#endif

    /* randomico ou não */
    srand((unsigned)time(0) + semente);

    IniciaPop(&P, MAXPOP, MAXVAR);
    erro = (double)P.indiv[P.melhor].fit - SOLUCAO;

    printf("\n***   (%s) por ACMO/CAP/LAC/INPE    ***", GARE);
    printf("\n      Evoluindo %s com %d vars", "f.o.", MAXVAR);
    while (numGeracoes-- && Aval < MAXAVA && erro > (double)0.0F)
    {
        numCruza = NUMCRU;
        while (numCruza--)
        {

            pa1 = Selecao(&P);
            pa2 = Selecao(&P);
            if (pa1 != pa2)
                CruzaBlend(&P, pa1, pa2, P.pior, xalfa);
            //                        CruzaEsfe(&P, pa1, pa2, P.pior);
            //                        CruzaGeom(&P, pa1, pa2, P.pior);
            else
                P.iguais++;
#ifdef SIMPLEXO

            if (pMuta > rand() % 100 && (MAXGER - numGeracoes) > (0.95 * MAXGER))
            {
                
                //                        fit = ObjectiveFunction(P.indiv[P.pior].var, P.tamInd);
                fit = SimplexFixo(P.indiv[P.pior].var, P.tamInd, MaxIt, funcao);
                P.numMuta++;
                
            }
            else
            {
                fit = ObjectiveFunction(P.indiv[P.pior].var, P.tamInd);
            }
#endif
#ifdef SIMPLEND
            if (pMuta > rand() % 100)
            {
                fit = SimplexRand(P.indiv[P.pior].var, P.tamInd, MaxIt);
                P.numMuta++;
            }
            else
            {
                fit = ObjectiveFunction(P.indiv[P.pior].var, P.tamInd);
            }
#endif
#ifdef NOSIMPLE
            if (pMuta > rand() % 100)
            {
                nu_mutate(P.indiv[P.pior].var, P.tamInd, P.tamPop, MAXGER - numGeracoes, MNUNI);
                P.numMuta++;
            }
            //                P.indiv[P.pior].var[0]=2.25;
            //                P.indiv[P.pior].var[1]=1.6;

            fit = ObjectiveFunction(P.indiv[P.pior].var, P.tamInd);
#endif
            AtualizaPop(&P, P.pior, fit, MAXGER - numGeracoes);
#ifdef PASSO
            if (!(numGeracoes % 20))
                printf("\r\t %d) Minimo = %.16f ", MAXGER - numGeracoes, P.indiv[P.melhor].fit);
#endif
        }
        erro = (double)P.indiv[P.melhor].fit - SOLUCAO;
    }

    

    /* *******************RESULTADOS ******************** */

    med = P.sumFit / P.tamPop;
    dvp = 0;
    for (i = 0; i < P.tamPop; i++)
        dvp += pow(P.indiv[i].fit - med, 2);
    dvp = sqrt(dvp / (P.tamPop - 1));

    bestPosition[0] = P.indiv[P.melhor].var[0];
    bestPosition[1] = P.indiv[P.melhor].var[1];

#ifdef XLS

    fprintf(saida, "\n%s)Min = %.10f; Aval= %d; Tempo = %.4f; Med = %.4f; Dpd = %.4f; Ger = %d",
            nomarq, P.indiv[P.melhor].fit, Aval, (double)(end - start) / 118, med, dvp, P.gerMelhor);

#endif

#ifdef CONSO
    printf("\n\t Variaveis ...");
    for (i = 0; i < P.tamInd; i++)
    {
        printf("\n\t\t%.6f", P.indiv[P.melhor].var[i]);
    }

    printf("\n\t Minimo = %.16f; \n\t Na ger = %d; \n\t mutacoes = %d \n\t ; aval = %d\n\t; erro = %f\n\t; (%.4f, %.4f)\n",
           P.indiv[P.melhor].fit, P.gerMelhor, P.numMuta, Aval, erro, med, dvp);
    printf("\n\t em tempo = %.4f, ", (double)(end - start) / CLOCKS_PER_SEC);
    //getchar();
#endif

    destino_corrente[0] = (int)bestPosition[1] / pixel_size;
    destino_corrente[1] = (int)bestPosition[0] / pixel_size;
}

void TargetGenerator::IniciaPop(Populacao *p, int m, int n)
{
    int i, j, pior, melhor;
    double soma, fit;

    for (i = 0, soma = 0, pior = 0, melhor = 0; i < m; i++)
    {
        p->indiv[i].var[0] = (double)randgen(dmin[0], dmax[0]);
        p->indiv[i].var[1] = (double)randgen(dmin[1], dmax[1]);

        fit = ObjectiveFunction(p->indiv[i].var, n);

        p->indiv[i].fit = fit;
        if (fit > p->indiv[pior].fit)
            pior = i;
        if (fit < p->indiv[melhor].fit)
            melhor = i;
        soma += (fit);
    }

    p->tamPop = m;
    p->tamInd = n;
    p->sumFit = soma;
    p->melhor = melhor;
    p->pior = pior;
    p->numMuta = 0;
    p->iguais = 0;
}

int TargetGenerator::Selecao(Populacao *p)
{
    int i;
    double val, spin_val;

    val = 0.0;
    spin_val = (rand() % 101) / 100. * p->sumFit;
    i = rand() % p->tamPop;

    do
    {
        i = (i < p->tamPop - 1) ? i + 1 : 0;
        val = val + fabs(FATOR * p->sumFit) / (1 + p->indiv[i].fit - p->indiv[p->melhor].fit);
    } while (val < spin_val);
    return i; // posição do estouro
}

void TargetGenerator::CruzaBlend(Populacao *p, int pai, int mae, int filho, float alfa)
{
    float a, b, r;
    int i;
    double ajuste, nureal, interv;
    int nuinte, voltas;

    a = -alfa;
    b = 1 + alfa;
    r = rand() % 101 / 100.;
    r = a + r * (b - a);

    for (i = 0; i < p->tamInd; i++)
    {
        p->indiv[filho].var[i] =
            p->indiv[pai].var[i] + r * (p->indiv[mae].var[i] - p->indiv[pai].var[i]);
        if (p->indiv[filho].var[i] < dmin[0])
        {
            interv = dmax[0] - dmin[0];
            nureal = (dmin[0] - p->indiv[filho].var[i]) / interv;
            nuinte = nureal;
            ajuste = (nureal - nuinte) * interv;
            voltas = nuinte % 2;
            p->indiv[filho].var[i] = dmin[0] + voltas * interv - (2 * voltas - 1) * ajuste;
        }
        else if (p->indiv[filho].var[i] > dmax[0])
        {
            interv = dmax[0] - dmin[0];
            nureal = (p->indiv[filho].var[i] - dmax[0]) / interv;
            nuinte = nureal;
            ajuste = (nureal - nuinte) * interv;
            voltas = 1 - nuinte % 2;
            p->indiv[filho].var[i] = dmin[0] + voltas * interv - (2 * voltas - 1) * ajuste;
        }
    }
}

void TargetGenerator::CruzaGeom(Populacao *p, int pai, int mae, int filho)
{
    int i;
    float alfa;

    alfa = rand() % 101 / 100.;

    for (i = 0; i < p->tamInd; i++)
    {
        p->indiv[filho].var[i] =
            pow((double)p->indiv[pai].var[i], (double)alfa) * pow(p->indiv[mae].var[i], 1 - (double)alfa);
    }
}

void TargetGenerator::CruzaEsfe(Populacao *p, int pai, int mae, int filho)
{
    int i;
    float alfa;

    alfa = rand() % 101 / 100.;

    for (i = 0; i < p->tamInd; i++)
    {
        p->indiv[filho].var[i] =
            sqrt(alfa * pow(p->indiv[pai].var[i], 2) + (1 - alfa) * pow(p->indiv[mae].var[i], 2));
    }
}

void TargetGenerator::AtualizaPop(Populacao *p, int pos, double fit, int ger)
{
    int i, j, max;

    p->sumFit -= (p->indiv[pos].fit); //fabs
    p->sumFit += (fit);               // fabs
    p->indiv[pos].fit = fit;

    if (fit < p->indiv[p->melhor].fit)
    {
        p->melhor = pos;
        p->gerMelhor = ger;
    }
    /* ***** procura um outro pior ******** */
    max = PATUAL * p->tamPop / 100;
    //        p->pior=p->melhor == p->tamPop ? p->melhor - 1: p->melhor +1;
    for (i = 0; i < max; i++)
    {
        j = rand() % p->tamPop;
        if (j == pos || j == p->melhor)
            continue;
        if (p->indiv[j].fit > p->indiv[p->pior].fit)
        {
            p->pior = j;
            i += PATUAL;
        }
    }
}

float TargetGenerator::randgen(float fLlim, float fUlim)
{
    float fRandomVal;

    fRandomVal = rand() % 101 / 100.; // rand entre 0 e 1

    return (fLlim + (float)(fRandomVal * (fUlim - fLlim)));
}

void TargetGenerator::SimplexAlgorithm()
{
}

double TargetGenerator::Simplex(double start[], int n, double EPSILON,
                                double scale, int MAX_IT, double ALPHA, double BETA, double GAMMA)
{

    int vs; /* vertex with smallest value */
    int vh; /* vertex with next smallest value */
    int vg; /* vertex with largest value */

    int i, j, m, row;
    int k;   /* track the number of function evaluations */
    int itr; /* track the number of iterations */

    double **v;    /* holds vertices of simplex */
    double pn, qn; /* values used to create initial simplex */
    double *f;     /* value of function at each vertex */
    double fr;     /* value of function at reflection point */
    double fe;     /* value of function at expansion point */
    double fc;     /* value of function at contraction point */
    double *vr;    /* reflection - coordinates */
    double *ve;    /* expansion - coordinates */
    double *vc;    /* contraction - coordinates */
    double *vm;    /* centroid - coordinates */
    double min;
    double *vb; /* best vertice */

    double fsum, favg, s, cent;

    /* dynamically allocate arrays */

    /* allocate the rows of the arrays */
    v = (double **)malloc((n + 1) * sizeof(double *));
    f = (double *)malloc((n + 1) * sizeof(double));
    vr = (double *)malloc(n * sizeof(double));
    ve = (double *)malloc(n * sizeof(double));
    vc = (double *)malloc(n * sizeof(double));
    vm = (double *)malloc(n * sizeof(double));
    vb = (double *)malloc(n * sizeof(double));
    

    /* allocate the columns of the arrays */
    for (i = 0; i <= n; i++)
    {
        v[i] = (double *)malloc(n * sizeof(double));
    }

    /* create the initial simplex */
    /* assume one of the vertices is 0,0 */

    pn = scale * (sqrt((double)n + 1) - 1 + n) / (n * sqrt((double)2));
    qn = scale * (sqrt((double)n + 1) - 1) / (n * sqrt((double)2));

    for (i = 0; i < n; i++)
    {
        v[0][i] = start[i];
    }

    for (i = 1; i <= n; i++)
    {
        for (j = 0; j < n; j++)
        {
            if (i - 1 == j)
            {
                v[i][j] = pn + start[j];
            }
            else
            {
                v[i][j] = qn + start[j];
            }
        }
    }

    /* find the initial function values */
    for (j = 0; j <= n; j++)
    {
        f[j] = ObjectiveFunction(v[j], n);
    }

    k = n + 1;

    /* print out the initial values
  fprintf("Initial Values\n");
  for (j=0;j<=n;j++) {
  fprintf("%f %f %f\n",v[j][0],v[j][1],f[j]);
  }                               */

    /* begin the main loop of the minimization */
    for (itr = 1; itr <= MAX_IT; itr++)
    {
        /* find the index of the largest value */
        vg = 0;
        for (j = 0; j <= n; j++)
        {
            if (f[j] > f[vg])
            {
                vg = j;
            }
        }

        /* find the index of the smallest value */
        vs = 0;
        for (j = 0; j <= n; j++)
        {
            if (f[j] < f[vs])
            {
                vs = j;
            }
        }

        /* find the index of the second largest value */
        vh = vs;
        for (j = 0; j <= n; j++)
        {
            if (f[j] > f[vh] && f[j] < f[vg])
            {
                vh = j;
            }
        }

        /* calculate the centroid */
        for (j = 0; j <= n - 1; j++)
        {
            cent = 0.0;
            for (m = 0; m <= n; m++)
            {
                if (m != vg)
                {
                    cent += v[m][j];
                }
            }
            vm[j] = cent / n;
        }

        /* reflect vg to new vertex vr */
        for (j = 0; j <= n - 1; j++)
        {
            /*vr[j] = (1+ALPHA)*vm[j] - ALPHA*v[vg][j];*/
            vr[j] = vm[j] + ALPHA * (vm[j] - v[vg][j]);
        }
        fr = ObjectiveFunction(vr, n);
        k++;

        if (fr < f[vh] && fr >= f[vs])
        {
            for (j = 0; j <= n - 1; j++)
            {
                v[vg][j] = vr[j];
            }
            f[vg] = fr;
        }

        /* investigate a step further in this direction */
        if (fr < f[vs])
        {
            for (j = 0; j <= n - 1; j++)
            {
                /*ve[j] = GAMMA*vr[j] + (1-GAMMA)*vm[j];*/
                ve[j] = vm[j] + GAMMA * (vr[j] - vm[j]);
            }
            fe = ObjectiveFunction(ve, n);
            k++;

            /* by making fe < fr as opposed to fe < f[vs],
      Rosenbrocks function takes 63 iterations as opposed
      to 64 when using double variables. */

            if (fe < fr)
            {
                for (j = 0; j <= n - 1; j++)
                {
                    v[vg][j] = ve[j];
                }
                f[vg] = fe;
            }
            else
            {
                for (j = 0; j <= n - 1; j++)
                {
                    v[vg][j] = vr[j];
                }
                f[vg] = fr;
            }
        }

        /* check to see if a contraction is necessary */
        if (fr >= f[vh])
        {
            if (fr < f[vg] && fr >= f[vh])
            {
                /* perform outside contraction */
                for (j = 0; j <= n - 1; j++)
                {
                    /*vc[j] = BETA*v[vg][j] + (1-BETA)*vm[j];*/
                    vc[j] = vm[j] + BETA * (vr[j] - vm[j]);
                }
                fc = ObjectiveFunction(vc, n);
                k++;
            }
            else
            {
                /* perform inside contraction */
                for (j = 0; j <= n - 1; j++)
                {
                    /*vc[j] = BETA*v[vg][j] + (1-BETA)*vm[j];*/
                    vc[j] = vm[j] - BETA * (vm[j] - v[vg][j]);
                }
                fc = ObjectiveFunction(vc, n);
                k++;
            }

            if (fc < f[vg])
            {
                for (j = 0; j <= n - 1; j++)
                {
                    v[vg][j] = vc[j];
                }
                f[vg] = fc;
            }
            /* at this point the contraction is not successful,
      we must halve the distance from vs to all the
      vertices of the simplex and then continue.
      10/31/97 - modified to account for ALL vertices.
      */
            else
            {
                for (row = 0; row <= n; row++)
                {
                    if (row != vs)
                    {
                        for (j = 0; j <= n - 1; j++)
                        {
                            v[row][j] = v[vs][j] + (v[row][j] - v[vs][j]) / 2.0;
                        }
                    }
                }
                f[vg] = ObjectiveFunction(v[vg], n);
                k++;
                f[vh] = ObjectiveFunction(v[vh], n);
                k++;
            }
        }

        /* print out the value at each iteration
    fprintf("Iteration %d\n",itr);
    for (j=0;j<=n;j++) {
    fprintf("%f %f %f\n",v[j][0],v[j][1],f[j]);
    }                                        */

        /* test for convergence */
        fsum = 0.0;
        for (j = 0; j <= n; j++)
        {
            fsum += f[j];
        }
        favg = fsum / (n + 1);
        s = 0.0;
        for (j = 0; j <= n; j++)
        {
            s += pow((f[j] - favg), 2.0) / (n);
        }
        s = sqrt(s);
        if (s < EPSILON)
            break;
    }
    /* end main loop of the minimization */

    /* find the index of the smallest value */
    vs = 0;
    for (j = 0; j <= n; j++)
    {
        if (f[j] < f[vs])
        {
            vs = j;
        }
    }

    //  fprintf("The minimum was found at\n");
    for (j = 0; j < n; j++)
    {
        // fprintf("%e\n",v[vs][j]);
        start[j] = v[vs][j];
    }
    min = ObjectiveFunction(v[vs], n);
    k++;
    //  fprintf("%d Function Evaluations\n",k);
    //  fprintf("%d Iterations through program\n",itr);

    for (i = 0; i < n+1; i++)
    {
        free(v[i]);
    }

    free(f);
    free(vr);
    free(ve);
    free(vc);
    free(vm);
    free(v);
    free(vb);
    return min;
}

double TargetGenerator::SimplexFixo(double *indiv, int tam, int passos)
{
    double erro = TAXERR;
    //double Alfa=1.1, Beta=0.5, Gama=2.2;       /* expansion coefficient */
    double Alfa = VLALFA, Beta = VLBETA, Gama = VLGAMA;
    double min;

    min = Simplex(indiv, tam, erro, SCALE, passos, Alfa, Beta, Gama);
    return min;
}

double TargetGenerator::SimplexRand(double *indiv, int tam, int passos)
{
    double erro = TAXERR;
    double Alfa, Beta, Gama;
    double min;

    Alfa = randgen(VLALFA - 0.5, VLALFA + 0.5);
    Beta = randgen(VLBETA - 0.5, VLBETA + 0.5);
    Gama = randgen(VLGAMA - 0.5, VLGAMA + 0.5);

    min = Simplex(indiv, tam, erro, SCALE, passos, Alfa, Beta, Gama);
    return min;
}

void TargetGenerator::IniciaSct(Populacao *p, int m, int n, int nfun)
{
    int i, j, k;
    int pior, melhor;
    int ipd, dim;
    double inf, sup, pas;
    double soma, fit;
    static double lista[MAXVAR];

    inf = dmin[0];
    sup = dmax[1];
    ipd = pow(m, 1.0F / (n));
    pas = (double)(sup - inf) / ipd;
    for (i = 0; i < n; i++)
        lista[i] = inf;
    dim = 0;
    i = 0;
    soma = 0;
    pior = 0;
    melhor = 0;
    while (i < m)
    {
        for (j = 0; j < n; j++)
            p->indiv[i].var[j] = (double)randgen(lista[j], lista[j] + pas);
        lista[dim] = (lista[dim] + 2 * pas > sup ? sup - pas : lista[dim] + pas);
        fit = ObjectiveFunction(p->indiv[i].var, n);
        p->indiv[i].fit = fit;
        if (fit > p->indiv[pior].fit)
            pior = i;
        if (fit < p->indiv[melhor].fit)
            melhor = i;
        soma += (fit);
        if ((++i % ipd) == 0)
        {
            do
            {
                lista[dim] = dmax[0];
                dim++;
                lista[dim] = lista[dim] + pas;
            } while (lista[dim] + pas > sup && dim < MAXVAR - 1);
            if (lista[dim] + pas > sup)
                lista[dim] = sup - pas;
            dim = 0;
        }
    }
    p->tamPop = m;
    p->tamInd = n;
    p->sumFit = soma;
    p->melhor = melhor;
    p->pior = pior;
    p->numMuta = 0;
    p->iguais = 0;
}
