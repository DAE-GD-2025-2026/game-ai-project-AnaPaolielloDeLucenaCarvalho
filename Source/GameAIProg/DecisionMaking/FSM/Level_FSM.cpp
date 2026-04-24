// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_FSM.h"

#include "FSMComponent.h"
#include "DecisionMaking/GameAIController.h"
#include "States/GuardStates.h"


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
	
	// spawn parameters to override collision check
	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

	// spawn guard
	Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{0, 0, 110}, FRotator::ZeroRotator, SpawnParams);
	if (Agent)
	{
		Agent->SetDebugRenderingEnabled(false);
	}
    
	// spawn thief
	ThiefAgent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{800, -500, 110}, FRotator::ZeroRotator, SpawnParams);	
	
	// patrol route
	PatrolRoute.Empty();
	//PatrolRoute.Add(FVector{-225, -225, 110});
	//PatrolRoute.Add(FVector{225, -225, 110});
	//PatrolRoute.Add(FVector{225, 775, 110});
	//PatrolRoute.Add(FVector{-225, 775, 110});
	PatrolRoute.Add(FVector{-200, -200, 110});
	PatrolRoute.Add(FVector{200, -200, 110});
	PatrolRoute.Add(FVector{200, 200, 110});
	PatrolRoute.Add(FVector{-200, 200, 110});
	
	if (AGameAIController* AIController = Cast<AGameAIController>(Agent->GetController()))
	{
		if (UFSMComponent* FSM = Cast<UFSMComponent>(AIController->GetBrainComponent()))
		{
			auto GuardBlackboard = std::make_shared<GameAI::FSM::Blackboard>();
			
			GuardBlackboard->SetData("TargetAgent", ThiefAgent);
			GuardBlackboard->SetData("PatrolRoute", PatrolRoute);
			
			// states
			auto Patrol = std::make_unique<GameAI::FSM::PatrolState>(Agent, GuardBlackboard.get());
			auto Chase = std::make_unique<GameAI::FSM::ChaseState>(Agent, GuardBlackboard.get());
			auto Search = std::make_unique<GameAI::FSM::SearchState>(Agent, GuardBlackboard.get());
			
			// store raw pointers for transitions before moving
			GameAI::FSM::State* pPatrol = Patrol.get();
			GameAI::FSM::State* pChase = Chase.get();
			GameAI::FSM::State* pSearch = Search.get();
			
			// add states to the FSM
			FSM->AddState(std::move(Patrol));
			FSM->AddState(std::move(Chase));
			FSM->AddState(std::move(Search));

			// patrol -> chase
			FSM->AddTransition(pPatrol, pChase, [this]() 
			{ 
				if (!IsValid(Agent) || !IsValid(ThiefAgent))
				{
					return false;
				}
				return FVector::Distance(Agent->GetActorLocation(), ThiefAgent->GetActorLocation()) < 600.0f;
			});
			
			// chase -> search
			FSM->AddTransition(pChase, pSearch, [this]() 
			{ 
				if (!IsValid(Agent) || !IsValid(ThiefAgent))
				{
					return true;
				}
				return FVector::Distance(Agent->GetActorLocation(), ThiefAgent->GetActorLocation()) >= 600.0f;
			});
			
			// search -> chase
			FSM->AddTransition(pSearch, pChase, [this]() 
			{ 
				if (!IsValid(Agent) || !IsValid(ThiefAgent))
				{
					return false;
				}
				return FVector::Distance(Agent->GetActorLocation(), ThiefAgent->GetActorLocation()) < 600.0f;
			});
			
			// search -> patrol
			FSM->AddTransition(pSearch, pPatrol, [this, GuardBlackboard]() 
			{ 
				if (!IsValid(Agent))
				{
					return false;
				}
    
				double StartTime = GuardBlackboard->GetData<double>("SearchStartTime");
				return (GetWorld()->GetTimeSeconds() - StartTime) > 3.0;
			});
			
			// run the machine
			AIController->RunFiniteStateMachine();
		}
	}
}

// Called every frame
void ALevel_FSM::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	// move thief towards mouse click
	if (ThiefAgent)
	{
		FVector MousePos3D = FVector(MouseTarget.Position.X, MouseTarget.Position.Y, ThiefAgent->GetActorLocation().Z);
		FVector ToTarget = MousePos3D - ThiefAgent->GetActorLocation();
        
		// only move if we are far
		if (ToTarget.Length() > 50.0f)
		{
			FVector Direction = ToTarget.GetSafeNormal();
			float ThiefSpeed = 500.0f; // SPEED !!!
            
			FVector NewLocation = ThiefAgent->GetActorLocation() + (Direction * ThiefSpeed * DeltaTime);
            
			ThiefAgent->SetActorLocation(NewLocation, true);
			ThiefAgent->SetActorRotation(Direction.Rotation());
		}
	}
}

