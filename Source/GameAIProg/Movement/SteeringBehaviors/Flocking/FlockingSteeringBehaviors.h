#pragma once
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
class Flock;

//COHESION - FLOCKING
//*******************
class Cohesion final : public Seek
{
public:
	Cohesion(Flock* const pFlock) : m_pFlock(pFlock) {};

	//Cohesion Behavior
	SteeringOutput CalculateSteering(float deltaT, ASteeringAgent& Agent) override;

private:
	Flock* m_pFlock = nullptr;
};

//SEPARATION - FLOCKING
//*********************
class Separation : public Evade
{
public:
	Separation(Flock* const pFlock) : Evade(), m_pFlock(pFlock) {};

	// Separation Behavior
	SteeringOutput CalculateSteering(float deltaT, ASteeringAgent& Agent) override;

private:
	Flock* m_pFlock = nullptr;
};

//VELOCITY MATCH - FLOCKING
//************************
class VelocityMatch : public Evade
{
public:
	VelocityMatch(Flock* const pFlock) :Evade(), m_pFlock(pFlock) {};

	//Velocity Match / Alignment Behavior
	SteeringOutput CalculateSteering(float deltaT, ASteeringAgent& Agent) override;

private:
	Flock* m_pFlock = nullptr;
};
