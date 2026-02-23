// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <memory>

#include "CoreMinimal.h"
#include "CombinedSteeringBehaviors.h"
#include "GameAIProg/Shared/Level_Base.h"
#include "GameAIProg/Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "Level_CombinedSteering.generated.h"

UCLASS()
class GAMEAIPROG_API ALevel_CombinedSteering : public ALevel_Base
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	UPROPERTY(EditAnywhere, Category = "AI") // Set in the editor, the class of the agent to spawn
	TSubclassOf<ASteeringAgent> AgentClass;
	ALevel_CombinedSteering();

	// Called every frame
	virtual void Tick(float DeltaTime) override;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	virtual void BeginDestroy() override;

private:
	//Data members
	bool UseMouseTarget = false;
	bool CanDebugRender = false;

	UPROPERTY()
	ASteeringAgent* m_pMyAgent{nullptr};
	
	std::unique_ptr<BlendedSteering> m_pBlendedSteering;
	std::unique_ptr<Seek> m_pSeek;
	std::unique_ptr<Wander> m_pWander;
};
