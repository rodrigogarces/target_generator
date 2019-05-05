#include "AStar.h"
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <algorithm> 

struct compare_with {
    int *attr_value;
    compare_with(int *attr_value) : attr_value(attr_value) { }

    bool operator ()(Node const& obj) const { return obj.posicao[0] == attr_value[0] 
														&& obj.posicao[1] == attr_value[1]; }
};

AStar::AStar(int width, int heigth, int **mapa)
{
	this->width = width;
	this->heigth = heigth;
	this->mapa = mapa;
}

AStar::~AStar()
{
}

vector< vector<int> > AStar::FindPath(int origem[2], int destino[2])
{
	Node root = Node();
	float H = sqrt( (destino[0] - origem[0])*(destino[0] - origem[0]) + (destino[1] - origem[1])*(destino[1] - origem[1]) ); // Distância Euclidiana
	Node from = Node(origem[0], origem[1], root, H);
	//root.G = 0;
	from.tag = "origem";

	listaAberta.push_back(from);

//#define PATH_VIEW
#ifdef PATH_VIEW
	int ** m = (int**) malloc(sizeof(int *) * this->heigth);
	for (int i = 0; i < this->heigth; i++){
		m[i] = (int *) malloc(sizeof(int) * this->width);
		for (int j = 0; j < this->width; j++)
		{
				if(mapa[j][i] > 0 || mapa[j][i] == -3)
				{
					m[i][j] = 2;
				}
				else if (mapa[j][i] == -2)
				{
					m[i][j] = 1 ;
				}
				else 
					m[i][j] = 0;
		}
	}
#endif

	while (true)
	{
		float menor = 10000000;
		if (listaAberta.size()==0)
		{
			cout << "lista vazia" << endl;
			break;
		}

		Node corrente = (Node)listaAberta.front();
		int ic = 0;

		for (int i = 0; i < listaAberta.size(); i++)
		{
			int x = listaAberta[i].posicao[0];
			int y = listaAberta[i].posicao[1];
    
			int p = this->mapa[y][x] < -1 ? 10000000000 : 0;
			float f = listaAberta[i].getF() + p;
			if (f < menor)
			{
				menor = f;
				corrente = (Node)listaAberta[i];
				ic = i;
			}
		}

		listaAberta.erase(listaAberta.begin()+ic);
		listaFechada.push_back(corrente);

		//cout << corrente.posicao[0] << ", " << corrente.posicao[1] << endl;
		if (corrente.posicao[0] == destino[0] && corrente.posicao[1] == destino[1])
		{
			//cout << "encontrou o destino" << endl;
			break;
		}

		for (int i = corrente.posicao[0] - 1; i <= corrente.posicao[0] + 1; i++)
		{
			for (int j = corrente.posicao[1] - 1; j <= corrente.posicao[1] + 1; j++)
			{
				if ((i != corrente.posicao[0] || j != corrente.posicao[1]) && (i >= 0 && i < width) 
					&& (j >= 0 && j < heigth) && (mapa[j][i] != -1) && ( mapa[j][i] <= 0 || mapa[j][i] == -2 ))
				{
					int pos[2]; pos[0] = i; pos[1] = j;
					vector<Node>::iterator it = find_if(listaFechada.begin(), listaFechada.end(), compare_with(pos) );

					if (it == listaFechada.end()) // Verifica a existencia do elemento na lista fechada
					{
						it = find_if(listaAberta.begin(), listaAberta.end(), compare_with(pos));
						if (it == listaAberta.end()) // Verifica a existencia do elemento na lista aberta
						{
							Node pai = (Node)corrente;
							float H = sqrt( (destino[0] - pos[0])*(destino[0] - pos[0]) + (destino[1] - pos[1])*(destino[1] - pos[1])); // Distância Euclidiana
							Node filho = Node(pos[0], pos[1], pai, H);
							listaAberta.push_back(filho);
						}
						else
						{
							float p = this->mapa[j][i] < -1 ? (-1)*this->mapa[j][i]*100 : 0;
							float G = corrente.getG() + p;
							
							if ((*it).posicao[0] != corrente.posicao[0] && (*it).posicao[1] != corrente.posicao[1])
								G += 14;
							else
								G += 10;

							if (G < (*it).getG())
							{
								// update cost
								Node pai = (Node)corrente;
								*((*it).pai) = pai;
								(*it).updateG();

								// update priority
								float H = sqrt( (destino[0] - pos[0])*(destino[0] - pos[0]) + (destino[1] - pos[1])*(destino[1] - pos[1])); // Distância Euclidiana
								(*it).updateF(H);
							}
						}
					}
				}
			}
		}
	}

#ifdef PATH_VIEW
	cout << "EU PASSEI AQUI!";
	FILE *fo = fopen("path_test.txt", "w");
	for (int i = 0; i < this->heigth; i++){
		for (int j = 0; j < this->width; j++)
		{
			fprintf(fo, "%d", m[i][j]);
		}
		fprintf(fo, "\n");
	}
	fclose(fo);
#endif

	vector< vector<int> > caminho;
	if (listaAberta.size() <= 0)
		return caminho; 

	Node curr = listaFechada.back();

	while (true)
	{
		vector<int> p;
		p.push_back(curr.posicao[0]);
		p.push_back(curr.posicao[1]);

		//m[curr.posicao[0]][curr.posicao[1]] = 1;
		
		caminho.push_back(p);
		curr = *(curr.pai);

		if (curr.tag == "origem")
			break;
	}


	

	return caminho;
}

int AStar::Exists(vector<Node> lista, int pos[2])
{
	for (int i = 0; i < lista.size(); i++)
	{
		if (lista[i].posicao[0] == pos[0] 
			&& lista[i].posicao[1] == pos[1])
		{
			return 1;
		}
	}
	return 0;
}

int AStar::Find(vector<Node> lista, int pos[2])
{
	for (int i = 0; i < lista.size(); i++)
	{
		if (lista[i].posicao[0] == pos[0] 
			&& lista[i].posicao[1] == pos[1])
		{
			return i;
		}
	}
	return -1;
}