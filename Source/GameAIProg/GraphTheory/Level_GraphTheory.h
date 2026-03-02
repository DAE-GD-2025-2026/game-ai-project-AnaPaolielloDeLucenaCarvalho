// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Movement/SteeringBehaviors/PathFollow/PathFollowSteeringBehavior.h"
#include "Shared/Level_Base.h"
#include "Shared/Graph/Graph.h"
#include "Shared/Graph/GraphEditorComponent.h"
#include "Shared/Graph/GraphRenderer.h"
#include "Level_GraphTheory.generated.h"

UCLASS()
class GAMEAIPROG_API ALevel_GraphTheory : public ALevel_Base
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "GraphEditor")
	TSubclassOf<UGraphEditorComponent> GraphEditorClass;
	
	// Sets default values for this actor's properties
	ALevel_GraphTheory();

	// Called every frame
	virtual void Tick(float DeltaTime) override;

protected:
	UPROPERTY()
	APlayerController* PlayerController{nullptr};
	
	virtual void BeginPlay() override;
	virtual void BeginDestroy() override;

private:
	UPROPERTY()
	ASteeringAgent* Agent{nullptr}; // ref
	PathFollow PathFollow{};
	GameAI::Graph Graph{false};
	GameAI::GraphRenderer Renderer{nullptr};
	GameAI::GraphNodeFactory<GameAI::Node> NodeFactory{};
	
	UPROPERTY()
	UGraphEditorComponent* PlayerGraphEditor{}; // ref
	
	void UpdateAgentPath( std::vector<GameAI::Node*> const & Trail);
};
