// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GraphTheory/Level_GraphTheory.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Level_Navmesh.generated.h"

UCLASS()
class GAMEAIPROG_API ALevel_Navmesh : public ALevel_Base
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="NavmeshLevel|Input")
	UInputAction* SetTargetAction{};
	
	// Sets default values for this actor's properties
	ALevel_Navmesh();

	// Called every frame
	virtual void Tick(float DeltaTime) override;
	
	virtual void BindLevelInputActions() override;
	
protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

private:
	std::unique_ptr<GameAI::NavGraph> NavigationGraph;
	std::unique_ptr<GameAI::GraphRenderer> Renderer;
	
	UPROPERTY()
	ASteeringAgent* Agent{nullptr}; // ref
	PathFollow PathFollow{};
	std::vector<FVector2D> DebugDrawPath{};
	
	bool bDrawNavPolyVertices{false};
	bool bDrawNavPoly{true};
	bool bDrawNavGraph{true};
	bool bDrawPath{true};
	bool bDrawPortals{false};
	
	void UpdateImGui();
	
	TArray<TArray<FVector>> ExtractNavMeshTris() const;
	
	// Input functions
	void SetTarget();
};
