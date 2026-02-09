#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//*******
// Week01 assignment
//*******

// Helper function to draw common debug visuals for steering behaviors
void DrawBaseSteeringDebug(ASteeringAgent& Agent, const FVector2D& LinearVelocity)
{
    UWorld* World = Agent.GetWorld();
    FVector start = FVector(Agent.GetPosition(), 0);

    // Yellow: Agent boundary circle
    DrawDebugCircle(World, start, 25.f, 50, FColor::Yellow, false, -1.f, 0, 2.f, FVector(1, 0, 0), FVector(0, 1, 0), false);

    // Magenta: Orientation line (Forward)
    float rot = Agent.GetRotation();
    FVector forward = start + FVector(cos(rot), sin(rot), 0) * 50.f;
    DrawDebugLine(World, start, forward, FColor::Magenta, false, -1.f, 0, 2.f);

    // Cyan: Desired Velocity direction
    if (!LinearVelocity.IsNearlyZero())
    {
        FVector velocityDir = start + FVector(LinearVelocity.GetSafeNormal(), 0) * 75.f;
        DrawDebugLine(World, start, velocityDir, FColor::Cyan, false, -1.f, 0, 2.f);
    }
}

// SEEK
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	// Calculate the desired velocity towards the target
    SteeringOutput Steering{};
    Steering.LinearVelocity = Target.Position - Agent.GetPosition();

    // Debug
    DrawDebugLine(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), FVector(Target.Position, 0), FColor::Green, false, -1.f, 0, 2.f);
    DrawDebugPoint(Agent.GetWorld(), FVector(Target.Position, 0), 15.f, FColor::Red, false, -1.f);
    DrawBaseSteeringDebug(Agent, Steering.LinearVelocity);

    return Steering;
}

// FLEE
SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	// Calculate the desired velocity away from the target
    SteeringOutput Fleeing{};
    Fleeing.LinearVelocity = Agent.GetPosition() - Target.Position;

    // Debug
    DrawDebugLine(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), FVector(Target.Position, 0), FColor::Green, false, -1.f, 0, 2.f);
    DrawDebugPoint(Agent.GetWorld(), FVector(Target.Position, 0), 15.f, FColor::Red, false, -1.f);

    DrawBaseSteeringDebug(Agent, Fleeing.LinearVelocity);

    return Fleeing;
}

// ARRIVE
SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	// Store the original max speed on the first call
    if (m_OriginalMaxSpeed < 0.f)
    {
        m_OriginalMaxSpeed = Agent.GetMaxLinearSpeed();
    }

	// Calculate the distance to the target and adjust speed based on proximity
    FVector2D toTarget = Target.Position - Agent.GetPosition();
    float Distance = toTarget.Size();
    const float SlowRadius = 300.f;
    const float TargetRadius = 50.f;

	// If within target radius, stop. If within slow radius, slow down proportionally. Otherwise, move at max speed.
    if (Distance < TargetRadius)
    {
        Agent.SetMaxLinearSpeed(0.f);
    }
    else if (Distance < SlowRadius)
    {
        float mappedSpeed = m_OriginalMaxSpeed * ((Distance - TargetRadius) / (SlowRadius - TargetRadius));
        Agent.SetMaxLinearSpeed(mappedSpeed);
    }
    else
    {
        Agent.SetMaxLinearSpeed(m_OriginalMaxSpeed);
    }

	// Debug
    FVector CenterPos(Agent.GetPosition(), 0);
    DrawDebugCircle(Agent.GetWorld(), CenterPos, TargetRadius, 50, FColor::Orange, false, -1.f, 0, 5.f, FVector(1, 0, 0), FVector(0, 1, 0), false);
    DrawDebugCircle(Agent.GetWorld(), CenterPos, SlowRadius, 50, FColor::Blue, false, -1.f, 0, 5.f, FVector(1, 0, 0), FVector(0, 1, 0), false);

	// Use Seek behavior to calculate the steering towards the target
    Seek DebugSeek;
    DebugSeek.SetTarget(Target);
    return DebugSeek.CalculateSteering(DeltaT, Agent);
}

// FACE
SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};
    FVector2D const direction = Target.Position - Agent.GetPosition();

    if (direction.SizeSquared() < 100.f) return Steering;

    float const targetRotation = FMath::Atan2(direction.Y, direction.X);
    float const currentRotation = Agent.GetRotation();

    float angleDiff = targetRotation - currentRotation;

    while (angleDiff > PI) angleDiff -= 2 * PI;
    while (angleDiff < -PI) angleDiff += 2 * PI;

    float degreesDiff = FMath::RadiansToDegrees(angleDiff);
    const float angleThreshold = 1.5f;

    if (FMath::Abs(degreesDiff) < angleThreshold)
    {
        Steering.AngularVelocity = 0.f;
    }
    else
    {
        Steering.AngularVelocity = (degreesDiff > 0) ? 1.f : -1.f;
    }

    DrawBaseSteeringDebug(Agent, FVector2D::ZeroVector);
    return Steering;
}

// PURSUIT
SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	// Calculate the distance to the target and estimate the future position based on the target's velocity
    float distance = FVector2D::Distance(Target.Position, Agent.GetPosition());
    float pursuerSpeed = Agent.GetMaxLinearSpeed();

    float lookAheadTime = (pursuerSpeed > 0) ? (distance / pursuerSpeed) : 0.f;

    FVector2D predictedPosition = Target.Position + (Target.LinearVelocity * lookAheadTime);

    SteeringOutput Steering{};
    Steering.LinearVelocity = predictedPosition - Agent.GetPosition();

	// Debug
    DrawDebugLine(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), FVector(predictedPosition, 0), FColor::Green, false, -1.f, 0, 2.f);
    DrawDebugPoint(Agent.GetWorld(), FVector(predictedPosition, 0), 15.f, FColor::Yellow, false, -1.f);
    DrawBaseSteeringDebug(Agent, Steering.LinearVelocity);

    return Steering;
}

// EVADE
SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	// Similar to Pursuit, but we want to steer away from the predicted position of the target
    float distance = FVector2D::Distance(Target.Position, Agent.GetPosition());
    float pursuerSpeed = Agent.GetMaxLinearSpeed();
    float lookAheadTime = (pursuerSpeed > 0) ? (distance / pursuerSpeed) : 0.f;
    lookAheadTime = FMath::Min(lookAheadTime, 1.5f);

    FVector2D predictedPosition = Target.Position + (Target.LinearVelocity * lookAheadTime);

    SteeringOutput Steering{};
    Steering.LinearVelocity = Agent.GetPosition() - predictedPosition;

    // Debug
    DrawDebugPoint(Agent.GetWorld(), FVector(predictedPosition, 0), 15.f, FColor::Orange, false, -1.f);
    DrawBaseSteeringDebug(Agent, Steering.LinearVelocity);

    return Steering;
}

// WANDER
SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	// Randomly adjust the wander angle within the specified maximum angle change
    m_WanderAngle += ((float)rand() / RAND_MAX * 2.f - 1.f) * m_MaxAngleChange;

    FVector2D agentPos = Agent.GetPosition();
    float agentRot = Agent.GetRotation();
    FVector2D agentDir = FVector2D(cos(agentRot), sin(agentRot));

    FVector2D circleCenter = agentPos + agentDir * m_OffsetDistance;
    float totalAngle = agentRot + m_WanderAngle;
    FVector2D wanderTarget = circleCenter + FVector2D(cos(totalAngle), sin(totalAngle)) * m_Radius;

	// Debug
    DrawBaseSteeringDebug(Agent, wanderTarget - agentPos);

    DrawDebugCircle(Agent.GetWorld(), FVector(circleCenter, 0), m_Radius, 50, FColor::Blue, false, -1.f, 0, 2.f, FVector(1, 0, 0), FVector(0, 1, 0), false);
    DrawDebugPoint(Agent.GetWorld(), FVector(wanderTarget, 0), 15.f, FColor::Green, false, -1.f);

    FTargetData wanderData;
    wanderData.Position = wanderTarget;
    this->SetTarget(wanderData);

    return Seek::CalculateSteering(DeltaT, Agent);
}