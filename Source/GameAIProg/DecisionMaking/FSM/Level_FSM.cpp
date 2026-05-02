// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_FSM.h"

#include "FSMComponent.h" // not needed anymore
#include "States/GuardStates.h" // not needed anymore
#include "DecisionMaking/GameAIController.h"


// Sets default values
ALevel_FSM::ALevel_FSM()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_FSM::BeginPlay()
{
	Super::BeginPlay();
	
	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

	// Spawn guard
	Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{0, 0, 110}, FRotator::ZeroRotator, SpawnParams);
	if (Agent) { Agent->SetDebugRenderingEnabled(false); }
    
	// Spawn thief
	ThiefAgent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{800, -500, 110}, FRotator::ZeroRotator, SpawnParams);	
	
	if (ThiefAgent)
	{
		if (AController* ThiefAI = ThiefAgent->GetController())
		{
			ThiefAI->UnPossess();
			ThiefAI->Destroy();
		}
	}	
}

void ALevel_FSM::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	if (ThiefAgent)
	{
		FVector MousePos3D = FVector(MouseTarget.Position.X, MouseTarget.Position.Y, ThiefAgent->GetActorLocation().Z);
		FVector ToTarget = MousePos3D - ThiefAgent->GetActorLocation();
        
		if (ToTarget.Length() > 50.0f)
		{
			FVector Direction = ToTarget.GetSafeNormal();
			FVector NewLocation = ThiefAgent->GetActorLocation() + (Direction * 500.0f * DeltaTime);
			ThiefAgent->SetActorLocation(NewLocation, true);
			ThiefAgent->SetActorRotation(Direction.Rotation());
		}
	}
}

