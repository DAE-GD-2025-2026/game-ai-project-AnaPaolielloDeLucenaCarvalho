#pragma once

#include "FlockingSteeringBehaviors.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "Movement/SteeringBehaviors/SteeringHelpers.h"
#include "Movement/SteeringBehaviors/CombinedSteering/CombinedSteeringBehaviors.h"
#include <memory>
#include "imgui.h"

class CellSpace; // Forward declaration
class Flock final
{
public:
	Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize = 10, 
	float WorldSize = 100.f, 
	ASteeringAgent* const pAgentToEvade = nullptr, 
	bool bTrimWorld = false);

	~Flock();

	void Tick(float DeltaTime);
	void RenderDebug();
	void ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize);
	
	float GetNeighborhoodRadius() const { return NeighborhoodRadius; }

	const TArray<ASteeringAgent*>& GetNeighbors() const;
	int GetNrOfNeighbors() const;

	// No space partitioning
	void RegisterNeighbors(ASteeringAgent* const Agent);

	FVector2D GetAverageNeighborPos() const;
	FVector2D GetAverageNeighborVelocity() const;

	void SetTarget_Seek(FSteeringParams const & Target);
	
	// Evade target
	ASteeringAgent* GetAgentToEvade() const { return pAgentToEvade; }

private:
	// For debug rendering purposes
	UWorld* pWorld{nullptr};
	
	int FlockSize{0};
	TArray<ASteeringAgent*> Agents{};
	float m_WorldSize{100.f};
	bool m_bTrimWorld{false};
	
	// Space partitioning
	std::unique_ptr<CellSpace> pPartitionedSpace{};
	int NrOfCellsX{ 10 };
	TArray<FVector2D> OldPositions{};
	bool bUseSpatialPartitioning{ true }; // UI Toggle

	// No space partitioning
	TArray<ASteeringAgent*> Neighbors{};
	
	float NeighborhoodRadius{200.f};
	int NrOfNeighbors{0};

	std::unique_ptr<Evade> m_pEvade{ nullptr }; 
	ASteeringAgent* pAgentToEvade{nullptr};
	
	//Steering Behaviors
	//std::unique_ptr<Separation> pSeparationBehavior{};
	//::unique_ptr<Cohesion> pCohesionBehavior{};
	//std::unique_ptr<VelocityMatch> pVelMatchBehavior{};
	//std::unique_ptr<Seek> pSeekBehavior{};
	//std::unique_ptr<Wander> pWanderBehavior{};
	
	std::unique_ptr<BlendedSteering> pBlendedSteering{};
	std::unique_ptr<PrioritySteering> pPrioritySteering{};

	// UI and rendering
	bool DebugRenderSteering{false};
	bool DebugRenderNeighborhood{true};
	bool DebugRenderPartitions{true};

	void RenderNeighborhood();
};
