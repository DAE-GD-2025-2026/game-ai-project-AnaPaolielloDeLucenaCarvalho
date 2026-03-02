// Fill out your copyright notice in the Description page of Project Settings.


#include "GameAISpectator.h"

#include "GameAISpectatorController.h"
#include "Camera/CameraComponent.h"


// Sets default values
AGameAISpectator::AGameAISpectator()
{
	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	bAddDefaultMovementBindings = false;
	bUseControllerRotationPitch = true;
	bUseControllerRotationYaw = true;

	Camera = CreateDefaultSubobject<UCameraComponent>("Camera");
	Camera->SetupAttachment(RootComponent);
}

// Called when the game starts or when spawned
void AGameAISpectator::BeginPlay()
{
	Super::BeginPlay();
}

void AGameAISpectator::PossessedBy(AController* NewController)
{
	Super::PossessedBy(NewController);

	GameAIController = Cast<AGameAISpectatorController>(GetController());
	
	GameAIController->AddInputMappingContexts(DefaultInputMappingContexts);
	GameAIController->AddInputMappingContexts(MovementInputMappingContexts);
}

void AGameAISpectator::UnPossessed()
{
	Super::UnPossessed();

	if (auto OldController = Cast<AGameAISpectatorController>(GetController()))
	{
		OldController->RemoveInputMappingContexts(MovementInputMappingContexts);
	}
}

// Called every frame
void AGameAISpectator::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void AGameAISpectator::SetCameraProjection(ECameraProjectionMode::Type ProjectionMode)
{
	Camera->SetProjectionMode(ProjectionMode);
}

