#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"

FlockingSteeringBehaviors::FlockingSteeringBehaviors(ASteeringAgent* pAgent, Flock* pFlock)
{
    m_pBlendedSteering = new BlendedSteering();

    m_pCohesion = new Cohesion(pFlock);
    m_pSeparation = new Separation(pFlock);
    m_pAlignment = new VelocityMatch(pFlock);

    // Add them to the blender (Behavior, Weight)
    m_pBlendedSteering->AddBehaviour({ m_pCohesion, 0.2f });
    m_pBlendedSteering->AddBehaviour({ m_pSeparation, 0.5f });
    m_pBlendedSteering->AddBehaviour({ m_pAlignment, 0.3f });
}

SteeringOutput FlockingSteeringBehaviors::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
    // The blender handles the weighted sum of all behaviors [cite: 188]
    return m_pBlendedSteering->CalculateSteering(deltaT, pAgent);
}

//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	// Move towards the average position of your neighbors

	FVector2D avgPos = pFlock->GetAverageNeighborPos();
	m_Target = FSteeringParams{ avgPos };
	return Seek::CalculateSteering(deltaT, pAgent);
}

//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	// Move away from your neighbors

    SteeringOutput steering = {};
    const auto& neighbors = pFlock->GetNeighbors();
    int nrNeighbors = pFlock->GetNrOfNeighbors();

    for (int i = 0; i < nrNeighbors; ++i)
    {
        FVector2D fromNeighbor = pAgent.GetLocation() - neighbors[i]->GetLocation();
        float distance = fromNeighbor.Size();

		if (distance > 0) // Avoid division by zero
        {
            steering.LinearVelocity += fromNeighbor.GetSafeNormal() / distance; // Push harder if closer (1/distance)
        }
    }

    steering.LinearVelocity.Normalize();
    steering.LinearVelocity *= pAgent.GetMaxLinearSpeed();
    return steering;
}

//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput VelocityMatch::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
    // Match the average velocity of the group
    SteeringOutput steering = {};
    steering.LinearVelocity = pFlock->GetAverageNeighborVelocity();
    return steering;
}