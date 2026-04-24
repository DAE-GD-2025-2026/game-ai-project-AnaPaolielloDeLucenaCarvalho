// Fill out your copyright notice in the Description page of Project Settings.


#include "FSMComponent.h"
#include "FiniteStateMachine.h"


// Sets default values for this component's properties
UFSMComponent::UFSMComponent()
{	
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	FSMInstance = std::make_unique<GameAI::FSM::FSM>();
}

UFSMComponent::~UFSMComponent() = default;

void UFSMComponent::AddState(std::unique_ptr<GameAI::FSM::State>&& NewState)
{
	if (FSMInstance)
	{
		// Use std::move because we are transferring ownership of the unique_ptr
		FSMInstance->AddState(std::move(NewState)); 
	}
}

void UFSMComponent::AddTransition(GameAI::FSM::State* From, GameAI::FSM::State* To, std::function<bool()> EvalFunc) const
{
	if (FSMInstance)
	{
		FSMInstance->AddTransition(From, To, EvalFunc);
	}
}

// Called when the game starts
void UFSMComponent::BeginPlay()
{
	Super::BeginPlay();
}


// Called every frame
void UFSMComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    
	// Only update if the logic is allowed to run
	if (bIsRunning && FSMInstance)
	{
		FSMInstance->Update(DeltaTime); // Note: Call whatever your update function is named in GameAI::FSM::FSM
	}
}

void UFSMComponent::StartLogic()
{
	Super::StartLogic();

	bIsRunning = true;
	
	if (FSMInstance)
	{
		FSMInstance->Start(); 
	}
}

void UFSMComponent::StopLogic(const FString& Reason)
{
	bIsRunning = false;
}

bool UFSMComponent::IsRunning() const
{
	return bIsRunning;
}

