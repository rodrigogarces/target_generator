#pragma once
class Firefly
{
private:
	double * position;

public:
	double bfitness;
	double * bposition;
	double fitness;
	double intensity;

	Firefly();
	~Firefly();
	int CompareTo(Firefly other);
	void SetPosicao(double pos[2]);
	double* GetPosicao();
};

