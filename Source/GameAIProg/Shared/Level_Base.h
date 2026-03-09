// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "EnhancedInputSubsystems.h"
#include "EnhancedInputComponent.h"
#include "Engine/LevelScriptActor.h"
#include "Templates/Tuple.h"

#include "ImGuiModule.h"
#include "InputMappingContextPriority.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringHelpers.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "GameAIProg/Shared/WorldTrimVolume.h"
#include "Shared/ImGuiHelpers.h"

#include "Level_Base.generated.h"

/*
 * Base class for all levels, handles some common variables
 */

UCLASS()
class GAMEAIPROG_API ALevel_Base : public ALevelScriptActor
{
	GENERATED_BODY()

public:
	// Default always active, input mappings (conditionals you must do yourself)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="LevelBase|Input")
	TArray<FInputMappingContextPriority> InputMappingContexts{};
	
	// Sets default values for this actor's properties
	ALevel_Base();

	// Called every frame
	virtual void Tick(float DeltaTime) override;
	
protected:
	// Common UI Vars
	float const MenuWidth{235};
	FVector2D ViewportSize{};
	ImVec2 WindowSize{};
	ImVec2 WindowPos{};

	// Mouse target render toggle
	bool VisualizeMouseTarget{true}; // TODO unused so far

	UPROPERTY(EditAnywhere)
	AWorldTrimVolume* TrimWorld{nullptr};

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	TSubclassOf<ASteeringAgent> SteeringAgentClass{};

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FSteeringParams MouseTarget{}; // UHT does not work with using statements, therefore this is called FSteeringParams
	
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	// Level input mapping
	UPROPERTY()
	UEnhancedInputLocalPlayerSubsystem* EnhancedInputSubsystem{nullptr};
	UPROPERTY()
	UEnhancedInputComponent* PlayerEnhancedInputComponent{nullptr};
	UPROPERTY()
	APlayerController* PlayerController{nullptr};
	
	bool CanBindLevelInput() const;
	void SetupEnhancedInputAttachment();
	virtual void BindLevelInputMappingContexts();
	virtual void BindLevelInputActions();
	
	// Helpers
	UPROPERTY(EditAnywhere,	BlueprintReadOnly)
	FVector LatestMouseWorldPos{};
	
	std::optional<FVector> GetMouseWorldPos() const;
	void UpdateLatestMouseWorldPos();
};
