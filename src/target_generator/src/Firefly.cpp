#include "Firefly.h"
#include "stdlib.h"

Firefly::Firefly()
{
	this->fitness = 0.0;
	this->intensity = 0.0;
	this->bfitness = 1000000000.0;
	this->position = (double *)malloc(2 * sizeof(double));
}


Firefly::~Firefly()
{
}

void Firefly::SetPosicao(double pos[2])
{
	this->position[0] = pos[0];
	this->position[1] = pos[1];
}

double* Firefly::GetPosicao()
{
	return position;
}

int Firefly::CompareTo(Firefly other)
{
	if (this->fitness < other.fitness)
		return -1;
	else if (this->fitness > other.fitness)
		return +1;
	else
		return 0;
}
