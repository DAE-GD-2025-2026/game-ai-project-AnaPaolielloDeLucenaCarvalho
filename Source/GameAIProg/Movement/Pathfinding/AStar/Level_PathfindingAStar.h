// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GraphTheory/Algorithms/Heuristics.h"
#include "Movement/SteeringBehaviors/PathFollow/PathFollowSteeringBehavior.h"
#include "Shared/Level_Base.h"
#include "Shared/Graph/GraphRenderer.h"
#include "Shared/Graph/TerrainGraph/TerrainGridGraph.h"
#include "Level_PathfindingAStar.generated.h"

UCLASS()
class GAMEAIPROG_API ALevel_PathfindingAStar : public ALevel_Base
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="PathfindingLevel|Input")
	UInputAction* SetStartNodeAction{};
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="PathfindingLevel|Input")
	UInputAction* SetEndNodeAction{};
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="PathfindingLevel|Input")
	UInputAction* SetNodeTerrainClearAction{};
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="PathfindingLevel|Input")
	UInputAction* SetNodeTerrainMudAction{};
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="PathfindingLevel|Input")
	UInputAction* SetNodeTerrainWaterAction{};

	// Sets default values for this actor's properties
	ALevel_PathfindingAStar();

	// Called every frame
	virtual void Tick(float DeltaTime) override;
protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	virtual void BeginDestroy() override;
	
	virtual void BindLevelInputActions() override;

private:
	UPROPERTY()
	ASteeringAgent* Agent{nullptr}; // ref
	PathFollow PathFollow{};
	
	GameAI::TerrainGridGraph* TerrainGraph{nullptr};
	GameAI::GraphRenderer* Renderer{nullptr};
	GameAI::TerrainNodeFactory* NodeFactory{nullptr};
	
	int PathStartNodeId{44};
	int PathEndNodeId{88};
	int SelectedHeuristic = 4;
	GameAI::HeuristicFunctions::Heuristic HeuristicFunction = GameAI::HeuristicFunctions::Chebyshev;
	std::vector<GameAI::Node*> FoundPath{};

	void CalculatePath();
	void UpdateAgentPath(std::vector<GameAI::Node*> const & Path);
	
	void UpdateImGui();
	
	// Input functions
	void SetStartNodeId();
	void SetEndNodeId();
	void SetNodeTerrain(GameAI::TerrainNode::Type TerrainType);
};
