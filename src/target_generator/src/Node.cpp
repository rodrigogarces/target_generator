#include "Node.h"
#include<cmath>

Node::Node()
{
	F = 1000000000000000;
	G = 0;
}

Node::Node(int i, int j, Node pai, float H)
{
	posicao[0] = i; posicao[1] = j;

	this->pai = new Node();

	if (&pai != NULL)
	{
		*(this->pai) = pai;
	}

	tag = "";
	
	G = pai.getG();
	
	if(i != pai.posicao[0] && j != pai.posicao[1])
		G += 14;
	else G += 10;

	F = G + H;
}


Node::~Node()
{
}

float Node::getG()
{
	return G;
}

void Node::updateG()
{
	G = pai->getG();
	if(posicao[0] != pai->posicao[0] && posicao[1] != pai->posicao[1])
		G += 14;
	else G += 10;
}

float Node::getF()
{
	return F;
}

void Node::updateF(float H)
{
	F = G + H;
}

float Node::getF(int destino[2], float fitness)
{
	if (pai == NULL)
		return 0;
		
	//int H = abs(destino[0] - posicao[0]) + abs(destino[1] - posicao[1]); // Heuristica de Manhanttan
	int H = sqrt( (destino[0] - posicao[0])*(destino[0] - posicao[0]) + (destino[1] - posicao[1])*(destino[1] - posicao[1]) ); // Distância Euclidiana
	G = pai->G;

	if (posicao[0] != pai->posicao[0] && posicao[1] != pai->posicao[1]){
		//if (fitness > 0) G += 14 * fitness;
		//else
		 G += 14;
	}
	else
		G += 10;

	F = G + H;

	F += fitness;

	return F;
}
