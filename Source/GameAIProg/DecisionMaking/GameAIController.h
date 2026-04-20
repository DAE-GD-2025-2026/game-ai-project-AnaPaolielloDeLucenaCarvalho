// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "AIController.h"
#include "GameAIController.generated.h"

UCLASS()
class GAMEAIPROG_API AGameAIController : public AAIController
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="AI|FSM")
	TObjectPtr<UBlackboardData> FSMBlackboardAsset; 
	
	// Sets default values for this actor's properties
	AGameAIController();
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	
	void RunFiniteStateMachine();
	
protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	void InitFiniteStateMachine();
};
