// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpectatorPawn.h"
#include "GameAISpectator.generated.h"

class AGameAISpectatorController;
class UInputMappingContext;

/*
 * Pawn class used to view the assignments
 */

UCLASS()
class GAMEAIPROG_API AGameAISpectator : public ASpectatorPawn
{
	GENERATED_BODY()

public:
	// Sets default values for this pawn's properties
	AGameAISpectator();

protected:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="GameAISpectator")	
	AGameAISpectatorController* GameAIController{};
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="GameAISpectator|Input")	
	TArray<UInputMappingContext*> DefaultInputMappingContexts{};
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="GameAISpectator|Input")	
	TArray<UInputMappingContext*> MovementInputMappingContexts{}; // Only bound when bIsLockedSpectatorMode == false

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="GameAISpectator|Input")
	float MaxTraceDistance{20000.0f};

	// Components
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="GameAISpectator")
	UCameraComponent* Camera{};
	
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	virtual void PossessedBy(AController* NewController) override;

	virtual void UnPossessed() override;
	
public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	void SetCameraProjection(ECameraProjectionMode::Type ProjectionMode);
};
