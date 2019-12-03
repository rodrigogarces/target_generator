#pragma once
#include"Node.h"
#include<vector>
using namespace std;
class AStar
{
	vector<Node> listaAberta;
	vector<Node> listaFechada;
	int** mapa;
	float** mapaObjetivo;
	int width;
	int heigth;

public:
	AStar(int width, int heigth, int **mapa);
	~AStar();
	vector< vector<int> > FindPath(int origem[2], int destino[2]);

private:
	int Exists(vector<Node> lista, int pos[2]);
	int Find(vector<Node> lista, int pos[2]);
	
};

