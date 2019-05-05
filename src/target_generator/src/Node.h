#pragma once
#include<string>
#include<vector>

using namespace std;
class Node
{
public:
	int posicao[2];
	Node *pai;
	float F;
	float G;
	string tag;

	Node();
	Node(int i, int j, Node pai, float H);
	~Node();
	float getF(int destino[2], float fitness);
	float getG();
	void updateG();
	float getF();
	void updateF(float H);

private:
	float CalculaCusto(int destino[2], float fitness);
};

