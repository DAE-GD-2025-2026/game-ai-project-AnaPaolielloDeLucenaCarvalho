#include "PathFollowSteeringBehavior.h"
#include "../SteeringAgent.h"

PathFollow::PathFollow()
{
	pSeek = new Seek();
	pArrive = new Arrive();
	pArrive->SetTargetRadius(10.0f);
}

PathFollow::~PathFollow()
{
	delete pArrive;
	delete pSeek;
}

void PathFollow::SetPath(std::vector<FVector2D>& path)
{
	pathVec = path;  
	
	currentPathIndex = -1;
	GotoNextPathPoint();
}

SteeringOutput PathFollow::CalculateSteering(float DeltaTime, ASteeringAgent& Agent)
{
	if (currentPathIndex < static_cast<int>(pathVec.size()))
	{
		float agentRadius = Agent.GetCapsuleRadius();
		FVector2D ToPathPoint{pathVec[currentPathIndex] - Agent.GetPosition()};
		
		if (ToPathPoint.SizeSquared() < agentRadius * agentRadius)
		{
			//Reached point of the path
			GotoNextPathPoint();
		}
	}

	if (pCurrentSteering != nullptr)
	{
		return pCurrentSteering->CalculateSteering(DeltaTime, Agent);
	}
	return SteeringOutput{};
}

void PathFollow::GotoNextPathPoint()
{
	++currentPathIndex;
	if (currentPathIndex >= static_cast<int>(pathVec.size())) return;
	
	if (currentPathIndex == pathVec.size() -1)
	{
		FTargetData PathTarget{pathVec[currentPathIndex]};
		//We have reached the last node
		pArrive->SetTarget(PathTarget);
		pCurrentSteering = pArrive;
	}
	else
	{
		FTargetData PathTarget{pathVec[currentPathIndex]};
		//Move to the next node
		pSeek->SetTarget(PathTarget);
		pCurrentSteering = pSeek;
	}
}
