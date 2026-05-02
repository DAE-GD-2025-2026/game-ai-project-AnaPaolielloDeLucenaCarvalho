// Fill out your copyright notice in the Description page of Project Settings.

#include "GameAIController.h"

#include "BehaviorTree/BlackboardComponent.h"
#include "Perception/AIPerceptionComponent.h"
#include "Perception/AISenseConfig_Sight.h"
#include "Perception/AISense_Sight.h"

// Sets default values
AGameAIController::AGameAIController()
{
	PrimaryActorTick.bCanEverTick = true;

	AIPerceptionComp = CreateDefaultSubobject<UAIPerceptionComponent>(TEXT("AIPerceptionComp"));
	SightConfig = CreateDefaultSubobject<UAISenseConfig_Sight>(TEXT("SightConfig"));

	SightConfig->SightRadius = 600.0f;
	SightConfig->LoseSightRadius = 650.0f;
	SightConfig->PeripheralVisionAngleDegrees = 90.0f;
	SightConfig->DetectionByAffiliation.bDetectEnemies = true;
	SightConfig->DetectionByAffiliation.bDetectNeutrals = true;
	SightConfig->DetectionByAffiliation.bDetectFriendlies = true;

	AIPerceptionComp->ConfigureSense(*SightConfig);
	AIPerceptionComp->SetDominantSense(UAISense_Sight::StaticClass());
}

// Called when the game starts or when spawned
void AGameAIController::BeginPlay()
{
	Super::BeginPlay();

	AIPerceptionComp->OnTargetPerceptionUpdated.AddDynamic(this, &AGameAIController::OnTargetPerceptionUpdated);

	// start the Behavior Tree!
	if (BehaviorTreeAsset)
	{
		PatrolRoute.Empty();
		PatrolRoute.Add(FVector{-200, -200, 110});
		PatrolRoute.Add(FVector{200, -200, 110});
		PatrolRoute.Add(FVector{200, 200, 110});
		PatrolRoute.Add(FVector{-200, 200, 110});
		
		RunBehaviorTree(BehaviorTreeAsset);
	}
}

void AGameAIController::OnTargetPerceptionUpdated(AActor* Actor, FAIStimulus Stimulus)
{
	if (UBlackboardComponent* BBComp = GetBlackboardComponent())
	{
		if (Stimulus.WasSuccessfullySensed())
		{
			BBComp->SetValueAsObject(FName("TargetActor"), Actor);
			BBComp->ClearValue(FName("LastKnownLocation"));
		}
		else
		{
			BBComp->ClearValue(FName("TargetActor"));
			BBComp->SetValueAsVector(FName("LastKnownLocation"), Stimulus.ReceiverLocation);
		}
	}
}