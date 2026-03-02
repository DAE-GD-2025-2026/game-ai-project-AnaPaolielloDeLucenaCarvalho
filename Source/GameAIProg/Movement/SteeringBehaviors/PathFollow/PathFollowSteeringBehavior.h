#pragma once

#include <vector>

#include "../Steering/SteeringBehaviors.h"

class PathFollow : public ISteeringBehavior
{
public:
	PathFollow();
	virtual ~PathFollow() override;
	void SetPath(std::vector<FVector2D>& path);
	virtual SteeringOutput CalculateSteering(float DeltaTime, ASteeringAgent & Agent) override;

private:
	Seek* pSeek = nullptr;
	Arrive* pArrive = nullptr;
	ISteeringBehavior* pCurrentSteering = nullptr;
	std::vector<FVector2D> pathVec = {};
	int currentPathIndex = 0;

	void GotoNextPathPoint();
};