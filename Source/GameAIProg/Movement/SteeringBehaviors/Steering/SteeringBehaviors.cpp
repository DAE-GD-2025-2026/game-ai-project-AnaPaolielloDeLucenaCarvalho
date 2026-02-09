#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//*******
// TODO: Do the Week01 assignment :^)
//*******

//SEEK
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent & Agent)
{
	SteeringOutput Steering{};

	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	
	//TODO - Add debug rendering (the lines, points and circles)

	return Steering;
}

//FLEE
SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Fleeing{};

	Fleeing.LinearVelocity = Target.Position + Agent.GetPosition();

	//TODO - Add debug rendering (the lines, points and circles)

	return Fleeing;
}