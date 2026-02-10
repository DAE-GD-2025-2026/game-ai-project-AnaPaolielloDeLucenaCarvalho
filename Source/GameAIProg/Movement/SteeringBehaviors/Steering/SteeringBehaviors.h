#pragma once

#include <Movement/SteeringBehaviors/SteeringHelpers.h>
#include "Kismet/KismetMathLibrary.h"

class ASteeringAgent;

// SteeringBehavior base, all steering behaviors should derive from this.
class ISteeringBehavior
{
public:
	ISteeringBehavior() = default;
	virtual ~ISteeringBehavior() = default;

	// Override to implement your own behavior
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent & Agent) = 0;

	void SetTarget(const FTargetData& NewTarget) { Target = NewTarget; }
	
	template<class T, std::enable_if_t<std::is_base_of_v<ISteeringBehavior, T>>* = nullptr>
	T* As()
	{ return static_cast<T*>(this); }

protected:
	FTargetData Target;
};

// Your own SteeringBehaviors should follow here...
class Seek : public ISteeringBehavior
{
public:
	Seek() = default;
	virtual ~Seek() = default;

	//Steering
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

class Flee : public ISteeringBehavior
{
public:
	Flee() = default;
	virtual ~Flee() = default;

	//Fleeing - the opposite of seeking (reuse seek behavior and invert)
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

class Arrive : public ISteeringBehavior
{
public:
	Arrive() = default;
	virtual ~Arrive() = default;

	//Arrive - similar to seek but with slowing down when approaching the target (2 radiuses SlowRadius & TargetRadius)
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;

protected:
	float m_OriginalMaxSpeed = -1.f;
};

class Face : public ISteeringBehavior
{
public:
	Face() = default;
	virtual ~Face() = default;

	//Face - rotates the agent to face the target (dont use SetRotation())
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

class Pursuit : public ISteeringBehavior
{
public:
	Pursuit() = default;
	virtual ~Pursuit() = default;

	//Pursuit - similar to seek but predicts the future position of the target based on its velocity/time and seeks to that point
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;

protected:
	FVector2D m_LastTargetPosition = FVector2D::ZeroVector;
	FVector2D m_CurrentVelocity = FVector2D::ZeroVector;
};

class Evade : public ISteeringBehavior
{
public:
	Evade() = default;
	virtual ~Evade() = default;

	//Evade - opposite of pursuit/similar to flee, predicts the future position of the target based on its velocity/time and flees that point
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;

protected:
	FVector2D m_LastTargetPosition = FVector2D::ZeroVector;
	FVector2D m_CurrentVelocity = FVector2D::ZeroVector;
};

class Wander : public Seek
{
public:
	Wander() = default;
	virtual ~Wander() = default;

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
    
	void SetWanderOffset(float offset) { m_OffsetDistance = offset; }
	void SetWanderRadius(float radius) { m_Radius = radius; }
	void SetWanderMaxAngleChange(float rad) { m_MaxAngleChange = rad; }

protected:
	float m_OffsetDistance = 100.f;
	float m_Radius = 80.f;
	float m_MaxAngleChange = 45.f * PI / 180.f;
	float m_WanderAngle = 0.f;
};