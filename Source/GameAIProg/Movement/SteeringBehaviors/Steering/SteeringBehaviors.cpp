#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//*******
// Week01 assignment
//*******

// Helper function to draw debug visuals
void DrawBaseSteeringDebug(ASteeringAgent& Agent, const FVector2D& CurrentVelocity, const FVector2D& DesiredVelocity)
{
    UWorld* World = Agent.GetWorld();
    FVector start = FVector(Agent.GetPosition(), 0);

    // Magenta: Orientation line (Forward)
    float rotRad = FMath::DegreesToRadians(Agent.GetRotation());
    FVector forward = start + FVector(cos(rotRad), sin(rotRad), 0) * 50.f;
    DrawDebugLine(World, start, forward, FColor::Magenta, false, -1.f, 0, 2.f);

    // Green: current velocity
    if (!CurrentVelocity.IsNearlyZero())
    {
        FVector cur = FVector(CurrentVelocity, 0) * 0.2f;
        DrawDebugLine(World, start, start + cur, FColor::Green, false, -1.f, 0, 2.f);
    }

    // Cyan: desired velocity
    if (!DesiredVelocity.IsNearlyZero())
    {
        FVector des = FVector(DesiredVelocity.GetSafeNormal(), 0);
        DrawDebugLine(World, start, start + des * 100.f, FColor::Cyan, false, -1.f, 0, 2.f);
    }
}

// SEEK
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	// Calculate the desired velocity towards the target
    SteeringOutput Steering{};
    Steering.LinearVelocity = (Target.Position - Agent.GetPosition()).GetSafeNormal();

    FVector dir = FVector(Steering.LinearVelocity.GetSafeNormal(), 0) * 75.f;

    // Debug
    DrawDebugPoint(Agent.GetWorld(), FVector(Target.Position, 0), 15.f, FColor::Red, false, -1.f);
    DrawBaseSteeringDebug( Agent, Agent.GetLinearVelocity(), Steering.LinearVelocity);

    return Steering;
}

// FLEE
SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	// Calculate the desired velocity away from the target
    SteeringOutput Steering{};
    Steering.LinearVelocity = (Agent.GetPosition() - Target.Position).GetSafeNormal();

    // Debug
    DrawDebugPoint(Agent.GetWorld(), FVector(Target.Position, 0), 15.f, FColor::Red, false, -1.f);
    DrawBaseSteeringDebug(Agent, Agent.GetLinearVelocity(), Steering.LinearVelocity);

    return Steering;
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

    DrawDebugPoint(Agent.GetWorld(), FVector(Target.Position, 0), 15.f, FColor::Red, false, -1.f);
    
    SteeringOutput Steering{};
    Steering.LinearVelocity = toTarget.GetSafeNormal();
    DrawBaseSteeringDebug(Agent, Agent.GetLinearVelocity(), Steering.LinearVelocity);

    return Steering;
}

// FACE
SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    if (Target.Position.IsNearlyZero())
    {
        return SteeringOutput{};
    }

    SteeringOutput Steering{};
    FVector2D toTarget = Target.Position - Agent.GetPosition();

    if (toTarget.IsNearlyZero())
    {
        return Steering;
    }

    float desired = FMath::Atan2(toTarget.Y, toTarget.X);
    float current = FMath::DegreesToRadians(Agent.GetRotation());

    float delta = desired - current;
    delta = FMath::UnwindRadians(delta);

    float maxAngular = Agent.GetMaxAngularSpeed();
    Steering.AngularVelocity = FMath::Clamp(FMath::RadiansToDegrees(delta), -maxAngular, maxAngular);

    // Debug Rendering
    DrawDebugPoint( Agent.GetWorld(), FVector(Target.Position, 0), 12, FColor::Red, false, -1.f );
    DrawBaseSteeringDebug(Agent, Agent.GetLinearVelocity(), FVector2D::ZeroVector);

    return Steering;
}

// PURSUIT
SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};

    if (m_LastTargetPosition.IsZero())
    {
        m_LastTargetPosition = Target.Position;
        return Steering;
    }

    FVector2D moveDelta = Target.Position - m_LastTargetPosition;
    FVector2D rawVelocity = FVector2D::ZeroVector;

    if (DeltaT > 0.0001f)
    {
        rawVelocity = moveDelta / DeltaT;
    }

    if (!rawVelocity.IsNearlyZero(1.f))
    {
        m_CurrentVelocity = FMath::Lerp(m_CurrentVelocity, rawVelocity, 10.f * DeltaT);
    }
    else
    {
        m_CurrentVelocity = m_CurrentVelocity * 0.95f;
    }

    m_LastTargetPosition = Target.Position;

    FVector2D toTarget = Target.Position - Agent.GetPosition();
    float distance = toTarget.Size();
    float agentSpeed = Agent.GetMaxLinearSpeed();

    float t = 0.f;
    if (agentSpeed > 1.f)
    {
        t = distance / agentSpeed;
    }

    t = FMath::Clamp(t, 0.f, 3.f);

    FVector2D predictedPos = Target.Position + (m_CurrentVelocity * t);

    Steering.LinearVelocity = (predictedPos - Agent.GetPosition()).GetSafeNormal();

    GEngine->AddOnScreenDebugMessage(1, 0.0f, FColor::Yellow, FString::Printf(TEXT("Smooth Vel: %s"), *m_CurrentVelocity.ToString()));

    DrawDebugPoint(Agent.GetWorld(), FVector(Target.Position, 0), 15.f, FColor::Red, false, -1.f);
    DrawDebugPoint(Agent.GetWorld(), FVector(predictedPos, 0), 15.f, FColor::Purple, false, -1.f);
    DrawBaseSteeringDebug(Agent, Agent.GetLinearVelocity(), Steering.LinearVelocity);

    return Steering;
}

// EVADE
SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};

    if (m_LastTargetPosition.IsZero())
    {
        m_LastTargetPosition = Target.Position;
        return Steering;
    }

    FVector2D moveDelta = Target.Position - m_LastTargetPosition;
    FVector2D rawVelocity = FVector2D::ZeroVector;

    if (DeltaT > 0.0001f)
    {
        rawVelocity = moveDelta / DeltaT;
    }

    if (!rawVelocity.IsNearlyZero(1.f))
    {
        m_CurrentVelocity = FMath::Lerp(m_CurrentVelocity, rawVelocity, 10.f * DeltaT);
    }
    else
    {
        m_CurrentVelocity = m_CurrentVelocity * 0.95f;
    }
    m_LastTargetPosition = Target.Position;

    FVector2D toTarget = Target.Position - Agent.GetPosition();
    float distance = toTarget.Size();
    float agentSpeed = Agent.GetMaxLinearSpeed();

    float t = 0.f;
    if (agentSpeed > 1.f)
    {
        t = distance / agentSpeed;
    }
    t = FMath::Clamp(t, 0.f, 2.f);

    FVector2D predictedPos = Target.Position + (m_CurrentVelocity * t);

    Steering.LinearVelocity = (Agent.GetPosition() - predictedPos).GetSafeNormal();

    DrawDebugPoint(Agent.GetWorld(), FVector(Target.Position, 0), 12, FColor::Red, false, -1.f);
    DrawDebugPoint(Agent.GetWorld(), FVector(predictedPos, 0), 15.f, FColor::Purple, false, -1.f);
    DrawBaseSteeringDebug(Agent, Agent.GetLinearVelocity(), Steering.LinearVelocity);

    return Steering;
}

// WANDER
SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	// Randomly adjust the wander angle within the specified maximum angle change
    m_WanderAngle += ((float)rand() / RAND_MAX * 2.f - 1.f) * m_MaxAngleChange;

    FVector2D agentPos = Agent.GetPosition();
    float agentRotRad = FMath::DegreesToRadians(Agent.GetRotation());
    FVector2D agentDir = FVector2D(cos(agentRotRad), sin(agentRotRad));

    FVector2D circleCenter = agentPos + agentDir * m_OffsetDistance;
    float totalAngle = agentRotRad + m_WanderAngle;
    FVector2D wanderTarget = circleCenter + FVector2D(cos(totalAngle), sin(totalAngle)) * m_Radius;

	// Debug
    DrawBaseSteeringDebug(Agent, Agent.GetLinearVelocity(), (wanderTarget - agentPos).GetSafeNormal());
    DrawDebugCircle(Agent.GetWorld(), FVector(circleCenter, 0), m_Radius, 50, FColor::Blue, false, -1.f, 0, 2.f, FVector(1, 0, 0), FVector(0, 1, 0), false);
    DrawDebugPoint(Agent.GetWorld(), FVector(wanderTarget, 0), 15.f, FColor::Red, false, -1.f);

    FTargetData wanderData;
    wanderData.Position = wanderTarget;
    this->SetTarget(wanderData);

    return Seek::CalculateSteering(DeltaT, Agent);
}