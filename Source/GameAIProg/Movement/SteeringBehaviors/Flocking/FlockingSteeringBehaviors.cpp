#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"

FlockingSteeringBehaviors::FlockingSteeringBehaviors(ASteeringAgent* pAgent, Flock* pFlock)
{
    m_pBlendedSteering = new BlendedSteering(std::vector<BlendedSteering::WeightedBehavior>{});

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
    this->SetTarget(FSteeringParams{ avgPos }); 
    return Seek::CalculateSteering(deltaT, pAgent);
}

//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
    SteeringOutput steering = {};
    const auto& neighbors = pFlock->GetNeighbors();
    int nrNeighbors = pFlock->GetNrOfNeighbors();

    for (int i = 0; i < nrNeighbors; ++i)
    {
        FVector2D agentPos = FVector2D(pAgent.GetActorLocation());
        FVector2D neighborPos = FVector2D(neighbors[i]->GetActorLocation());
        
        FVector2D fromNeighbor = agentPos - neighborPos;
        float distance = fromNeighbor.Size();

        if (distance > 0 && distance < pFlock->GetNeighborhoodRadius()) 
        {
            steering.LinearVelocity += fromNeighbor.GetSafeNormal() / distance; 
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