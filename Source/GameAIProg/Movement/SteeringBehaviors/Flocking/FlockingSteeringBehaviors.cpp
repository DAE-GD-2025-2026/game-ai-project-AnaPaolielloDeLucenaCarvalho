#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"

FlockingSteeringBehaviors::FlockingSteeringBehaviors(ASteeringAgent* pAgent, Flock* pFlock)
    : m_pFlock(pFlock)
{
    m_pPrioritySteering = new PrioritySteering(std::vector<ISteeringBehavior*>{});
    m_pBlendedSteering = new BlendedSteering(std::vector<BlendedSteering::WeightedBehavior>{});
    m_pEvade = new Evade();

    m_pCohesion = new Cohesion(pFlock);
    m_pSeparation = new Separation(pFlock);
    m_pAlignment = new VelocityMatch(pFlock);
    m_pSeek = new Seek();
    m_pWander = new Wander();

    // Flocking Blender 
    m_pBlendedSteering->AddBehaviour({ m_pCohesion, 0.2f });
    m_pBlendedSteering->AddBehaviour({ m_pSeparation, 0.5f });
    m_pBlendedSteering->AddBehaviour({ m_pAlignment, 0.3f });
    m_pBlendedSteering->AddBehaviour({ m_pSeek, 0.3f });
    m_pBlendedSteering->AddBehaviour({ m_pWander, 0.3f });

    // Priority Steering 
    m_pPrioritySteering->AddBehaviour(m_pEvade);
    m_pPrioritySteering->AddBehaviour(m_pBlendedSteering);
}

SteeringOutput FlockingSteeringBehaviors::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
    m_pSeek->SetTarget(this->Target);

    if (m_pFlock && m_pFlock->GetAgentToEvade())
    {
        FTargetData EvadeTarget;
        EvadeTarget.Position = FVector2D(m_pFlock->GetAgentToEvade()->GetActorLocation());
        m_pEvade->SetTarget(EvadeTarget);
    }

    return m_pPrioritySteering->CalculateSteering(deltaT, pAgent);
}

//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
    // If no neighbors, exert no cohesion force
    if (pFlock->GetNrOfNeighbors() == 0) return SteeringOutput{};

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
    if (pFlock->GetNrOfNeighbors() == 0) return SteeringOutput{};

    SteeringOutput steering = {};
    steering.LinearVelocity = pFlock->GetAverageNeighborVelocity();
    
    steering.LinearVelocity.Normalize();
    steering.LinearVelocity *= pAgent.GetMaxLinearSpeed();
    
    return steering;
}