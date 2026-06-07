#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	steering.LinearVelocity = (m_pFlock->GetAverageNeighborPos() - Agent.GetPosition()).GetSafeNormal();
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= Agent.GetMaxLinearSpeed();
	return steering;
}

//*********************
//SEPARATION (FLOCKING)

SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	// Move away from neighbors with a speed that's inversly proportionla (y=1/x) to the distance to that neighbor.
	// The closer the neighbor is, the more impact it should have on the output velocity.

	for (int i{}; i < m_pFlock->GetNrOfNeighbors(); ++i)
	{
		FVector2D direction = Agent.GetPosition() - m_pFlock->GetNeighbors()[i]->GetPosition();
		float distance = direction.Size();

		if (distance > 0.0001f)
		{
			FVector2D normalizedDir = direction / distance;
			steering.LinearVelocity += normalizedDir / distance;
		}
	}

	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= Agent.GetMaxLinearSpeed();

	return steering;
}

//*************************
//VELOCITY MATCH (FLOCKING)

SteeringOutput VelocityMatch::CalculateSteering(float deltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	steering.LinearVelocity = m_pFlock->GetAverageNeighborVelocity();
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= Agent.GetMaxLinearSpeed();
	return steering;
}
