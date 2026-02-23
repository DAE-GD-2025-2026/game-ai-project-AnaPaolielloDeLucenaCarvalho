
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
	SteeringOutput BlendedSteering = {};
	BlendedSteering.IsValid = true;

	// Calculate the weighted average steering behavior
	for (const auto& weightedBehavior : WeightedBehaviors) // Loop through each behavior and calculate contribution to blended steering
	{
		if (weightedBehavior.pBehavior)
		{
			SteeringOutput result = weightedBehavior.pBehavior->CalculateSteering(DeltaT, Agent);

			BlendedSteering.LinearVelocity += (result.LinearVelocity * weightedBehavior.Weight);
			BlendedSteering.AngularVelocity += (result.AngularVelocity * weightedBehavior.Weight);
		}
	}

	if (BlendedSteering.LinearVelocity.Size() > Agent.GetMaxLinearSpeed()) // Don't exceed agent max speed
	{
		BlendedSteering.LinearVelocity.Normalize();
		BlendedSteering.LinearVelocity *= Agent.GetMaxLinearSpeed();
	}

	return BlendedSteering;
}

//*****************
//PRIORITY STEERING
SteeringOutput PrioritySteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering = {};

	for (ISteeringBehavior* const pBehavior : m_PriorityBehaviors)
	{
		if (pBehavior)
		{
			Steering = pBehavior->CalculateSteering(DeltaT, Agent);

			// If valid return it immediately
			if (Steering.IsValid)
			{
				return Steering;
			}
		}
	}

	// If none of the behavior return a valid output, last behavior is returned
	return Steering;
}
