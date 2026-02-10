#pragma once

#include <Movement/SteeringBehaviors/SteeringHelpers.h>
#include "Kismet/KismetMathLibrary.h"

class ASteeringAgent;

// SteeringBehavior base, all steering behaviors should derive from this.
class ISteeringBehavior
{
public:
	ISteeringBehavior() = default;
	virtual ~ISteeringBehavior() = default;

	// Override to implement your own behavior
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent & Agent) = 0;

	void SetTarget(const FTargetData& NewTarget) { Target = NewTarget; }
	
	template<class T, std::enable_if_t<std::is_base_of_v<ISteeringBehavior, T>>* = nullptr>
	T* As()
	{ return static_cast<T*>(this); }

protected:
	FTargetData Target;
};

// Your own SteeringBehaviors should follow here...
class Seek : public ISteeringBehavior
{
public:
	Seek() = default;
	virtual ~Seek() = default;

	virtual SteeringOutput CalculateSteering(float deltaT, ASteeringAgent& Agent) override;
};

class Flee : public ISteeringBehavior
{
public:
	Flee() = default;
	virtual ~Flee() = default;

	virtual SteeringOutput CalculateSteering(float deltaT, ASteeringAgent& Agent) override;
};

class Arrive : public ISteeringBehavior
{
public:
	Arrive() = default;
	virtual ~Arrive() = default;

	virtual SteeringOutput CalculateSteering(float deltaT, ASteeringAgent& Agent) override;

	void SetSlowRadius(float radius) { if (radius >= 0) m_SlowRadius = radius;}
	void SetTargetRadius(float radius) { if (radius >= 0) m_TargetRadius = radius; }
private:
	float m_SlowRadius = 550.f;
	float m_TargetRadius = 75.f;
};

class Face : public ISteeringBehavior
{
public:
	Face() = default;
	virtual ~Face() = default;

	virtual SteeringOutput CalculateSteering(float deltaT, ASteeringAgent& Agent) override;
};

class Pursuit : public ISteeringBehavior
{
public:
	Pursuit() = default;
	virtual ~Pursuit() = default;

	virtual SteeringOutput CalculateSteering(float deltaT, ASteeringAgent& Agent) override;
};

class Evade : public Pursuit
{
public:
	Evade() = default;
	virtual ~Evade() = default;

	virtual SteeringOutput CalculateSteering(float deltaT, ASteeringAgent& Agent) override;

	void SetRadius(float radius) { if (radius >= 0) m_Radius = radius; }
protected:
	float m_Radius = 300;
};


class Wander : public Seek
{
public:
	Wander() = default;
	virtual ~Wander() = default;

	//Wander Behavior
	virtual SteeringOutput CalculateSteering(float deltaT, ASteeringAgent& Agent) override;

	void SetWanderOffset(float offset) { m_OffsetDistance = offset; };
	void SetWanderRadius(float radius) { m_Radius = radius; };
	void SetMaxAngleChange(float rad) { m_MaxAngleChange = rad; };

protected:
	float m_OffsetDistance = 100.f;
	float m_Radius = 150.f;
	float m_MaxAngleChange = 45.f; // in degrees!
	float m_WanderAngle = 0.f;
};
