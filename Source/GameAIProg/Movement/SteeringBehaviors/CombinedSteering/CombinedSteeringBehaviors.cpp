
#include "CombinedSteeringBehaviors.h"
#include <algorithm>
#include "../SteeringAgent.h"

BlendedSteering::BlendedSteering(const std::vector<WeightedBehavior>& WeightedBehaviors)
	:WeightedBehaviors(WeightedBehaviors)
{};

//****************
//BLENDED STEERING
SteeringOutput BlendedSteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput blendedSteering = {};
	float totalWeight = 0.f; // imgui cannot guarantee that the total weight is 1!!

	for (const BlendedSteering::WeightedBehavior& behavior : WeightedBehaviors)
	{
		const SteeringOutput steering = behavior.pBehavior->CalculateSteering(DeltaT, Agent);
		blendedSteering.LinearVelocity += steering.LinearVelocity * behavior.Weight;
		blendedSteering.AngularVelocity += steering.AngularVelocity * behavior.Weight;
		totalWeight += behavior.Weight;
	}

	if (totalWeight > 0.f) blendedSteering /= totalWeight;
	
	if (Agent.GetDebugRenderingEnabled())
	{
		FVector2D dir{};
		double length{};

		blendedSteering.LinearVelocity.ToDirectionAndLength(dir, length);
		DrawDebugDirectionalArrow(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), FVector(Agent.GetPosition() + dir * 200, 0), 150, FColor(0, 0, 255), false, -1.f, 0U, 10.f);
	}

	return blendedSteering;
}

float* BlendedSteering::GetWeight(ISteeringBehavior* const SteeringBehavior)
{
	auto it = find_if(WeightedBehaviors.begin(),
		WeightedBehaviors.end(),
		[SteeringBehavior](const WeightedBehavior& Elem)
		{
			return Elem.pBehavior == SteeringBehavior;
		}
	);

	if(it!= WeightedBehaviors.end())
		return &it->Weight;
	
	return nullptr;
}

//*****************
//PRIORITY STEERING
SteeringOutput PrioritySteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering = {};

	for (ISteeringBehavior* const pBehavior : m_PriorityBehaviors)
	{
		Steering = pBehavior->CalculateSteering(DeltaT, Agent);

		if (Steering.IsValid)
			break;
	}

	//If non of the behavior return a valid output, last behavior is returned
	return Steering;
}