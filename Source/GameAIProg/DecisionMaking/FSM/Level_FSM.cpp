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
	
	Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{0,0,90}, FRotator::ZeroRotator);
	Agent->SetDebugRenderingEnabled(false);
	
	// thief spawns away from the guard
	ThiefAgent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{500,500,90}, FRotator::ZeroRotator);
	
	// patrol route
	PatrolRoute.Add(FVector{0, 0, 90});
	PatrolRoute.Add(FVector{1000, 0, 90});
	PatrolRoute.Add(FVector{1000, 1000, 90});
	PatrolRoute.Add(FVector{0, 1000, 90});
	
	if (AGameAIController* AIController = Cast<AGameAIController>(Agent->GetController()))
	{
		if (UFSMComponent* FSM = Cast<UFSMComponent>(AIController->GetBrainComponent()))
		{
			auto GuardBlackboard = std::make_shared<GameAI::FSM::Blackboard>();
			
			GuardBlackboard->SetData("TargetAgent", ThiefAgent);
			GuardBlackboard->SetData("PatrolRoute", PatrolRoute);
			
			// Create the states
			auto Patrol = std::make_unique<GameAI::FSM::PatrolState>(Agent, GuardBlackboard.get());
			auto Chase = std::make_unique<GameAI::FSM::ChaseState>(Agent, GuardBlackboard.get());
			auto Search = std::make_unique<GameAI::FSM::SearchState>(Agent, GuardBlackboard.get());
			
			// Store raw pointers for transitions before moving
			GameAI::FSM::State* pPatrol = Patrol.get();
			GameAI::FSM::State* pChase = Chase.get();
			GameAI::FSM::State* pSearch = Search.get();
			
			// Add states to the FSM (this transfers ownership, which is why we saved the raw pointers above)
			FSM->AddState(std::move(Patrol));
			FSM->AddState(std::move(Chase));
			FSM->AddState(std::move(Search));

			// Add Transitions (using dummy lambdas for now - TODO replace with actual conditions)
			FSM->AddTransition(pPatrol, pChase, []() { return false; /* TODO: IsTargetVisible */ });
			FSM->AddTransition(pChase, pSearch, []() { return false; /* TODO: !IsTargetVisible */ });
			FSM->AddTransition(pSearch, pChase, []() { return false; /* TODO: IsTargetVisible */ });
			FSM->AddTransition(pSearch, pPatrol, []() { return false; /* TODO: IsSearchingTooLong */ });

			// Run the machine
			AIController->RunFiniteStateMachine();
		}
	}
}

// Called every frame
void ALevel_FSM::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

