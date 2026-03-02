#pragma once
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "Movement/SteeringBehaviors/CombinedSteering/CombinedSteeringBehaviors.h"

class Flock;
class BlendedSteering;

//COHESION - FLOCKING
//*******************
class Cohesion final : public Seek
{
public:
	Cohesion(Flock* const pFlock) :pFlock(pFlock) {};

	//Cohesion Behavior
	virtual SteeringOutput CalculateSteering(float deltaT, ASteeringAgent& pAgent) override;

private:
	Flock* pFlock = nullptr;
};

//SEPARATION - FLOCKING
//*********************
class Separation final : public Seek
{
public:
	Separation(Flock* const pFlock) :pFlock(pFlock) {};

	//Cohesion Behavior
	virtual SteeringOutput CalculateSteering(float deltaT, ASteeringAgent& pAgent) override;

private:
	Flock* pFlock = nullptr;
};

//VELOCITY MATCH - FLOCKING
//************************
class VelocityMatch final : public Seek
{
public:
	VelocityMatch(Flock* const pFlock) :pFlock(pFlock) {};

	//Cohesion Behavior
	virtual SteeringOutput CalculateSteering(float deltaT, ASteeringAgent& pAgent) override;

private:
	Flock* pFlock = nullptr;
};

// FLOCKING STEERING BEHAVIOR
class FlockingSteeringBehaviors : public ISteeringBehavior
{
public:
	FlockingSteeringBehaviors(ASteeringAgent* pAgent, Flock* pFlock);
	virtual SteeringOutput CalculateSteering(float deltaT, ASteeringAgent& pAgent) override;

	BlendedSteering* GetBlendedSteering() const { return m_pBlendedSteering; }
private:
	// The blender that does the math
	BlendedSteering* m_pBlendedSteering = nullptr;

	// Individual behavior pointers to pass to the blender
	Cohesion* m_pCohesion = nullptr;
	Separation* m_pSeparation = nullptr;
	VelocityMatch* m_pAlignment = nullptr;
	Seek* m_pSeek = nullptr;
	Wander* m_pWander = nullptr;
	
	PrioritySteering* m_pPrioritySteering = nullptr;
	Evade* m_pEvade = nullptr;
	Flock* m_pFlock = nullptr;
};