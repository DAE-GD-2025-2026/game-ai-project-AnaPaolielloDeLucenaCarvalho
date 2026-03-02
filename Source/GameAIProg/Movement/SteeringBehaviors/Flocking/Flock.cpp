#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"
#include "../SpacePartitioning/SpacePartitioning.h"


Flock::Flock(UWorld* pWorld, TSubclassOf<ASteeringAgent> AgentClass, int FlockSize, float WorldSize, ASteeringAgent* const pAgentToEvade, bool bTrimWorld)
	: pWorld{pWorld}
	, FlockSize{ FlockSize }
	, m_WorldSize{WorldSize}
	, m_bTrimWorld{bTrimWorld}
	, pAgentToEvade{pAgentToEvade}
{
	Agents.SetNum(FlockSize);
	Neighbors.SetNum(FlockSize); 
	OldPositions.SetNum(FlockSize);
	
	// Create CellSpace
	pPartitionedSpace = std::make_unique<CellSpace>(pWorld, WorldSize * 2.0f, WorldSize * 2.0f, NrOfCellsX, NrOfCellsX, FlockSize);
	
	pBlendedSteering = std::make_unique<BlendedSteering>(std::vector<BlendedSteering::WeightedBehavior>{
		{ nullptr, 0.2f }, // Cohesion dummy
		{ nullptr, 0.5f }, // Separation dummy
		{ nullptr, 0.3f },  // Alignment dummy
		{ nullptr, 0.2f }, // Seek
		{ nullptr, 0.2f }  // Wander
	}); 
	
	pPrioritySteering = std::make_unique<PrioritySteering>(std::vector<ISteeringBehavior*>{}); 
	m_pEvade = std::make_unique<Evade>();

	if (pAgentToEvade) 
	{
		m_pEvade->SetTarget(FSteeringParams{ FVector2D(pAgentToEvade->GetActorLocation()) });
	}

	pPrioritySteering->AddBehaviour(m_pEvade.get());
	pPrioritySteering->AddBehaviour(pBlendedSteering.get());
	
	// Force the spawn even if they overlap 
	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

	for (int i = 0; i < FlockSize; ++i) 
	{
		FVector2D SpawnPos = FVector2D(FMath::RandRange(-WorldSize, WorldSize), FMath::RandRange(-WorldSize, WorldSize));
		
		ASteeringAgent* pAgent = pWorld->SpawnActor<ASteeringAgent>(AgentClass, FVector(SpawnPos.X, SpawnPos.Y, 150.f), FRotator::ZeroRotator, SpawnParams);
		
		if (pAgent)
		{
			pAgent->SetActorTickEnabled(false); 

			Agents[i] = pAgent;
			pAgent->SetSteeringBehavior(new FlockingSteeringBehaviors(pAgent, this));
			
			pPartitionedSpace->AddAgent(*pAgent);
			OldPositions[i] = FVector2D(pAgent->GetActorLocation());
		}
	}
}

Flock::~Flock()
{
	// Cleanup any additional data
	for (ASteeringAgent* pAgent : Agents) // Loop through every agent in the flock
	{
		if (pAgent && pAgent->IsValidLowLevel())
		{
			pAgent->Destroy();
		}
	}
	Agents.Empty();
}

void Flock::Tick(float DeltaTime)
{
	// For every agent:
	// Register the neighbors for this agent (-> fill the memory pool with the neighbors for the currently evaluated agent)
	// Update the agent (-> the steering behaviors use the neighbors in the memory pool)
	// Trim the agent to the world

	if (pAgentToEvade) 
	{
		m_pEvade->SetTarget(FSteeringParams{ FVector2D(pAgentToEvade->GetActorLocation()) });
	}
	
	for (int i = 0; i < Agents.Num(); ++i)
	{
		ASteeringAgent* pAgent = Agents[i];
		
		if(IsValid(pAgent))
		{
			RegisterNeighbors(pAgent);

			pAgent->Tick(DeltaTime);
            
			if (m_bTrimWorld)
			{
				FVector Location = pAgent->GetActorLocation();
				bool bChanged = false;

				if (Location.X > m_WorldSize) { Location.X = -m_WorldSize; bChanged = true; }
				else if (Location.X < -m_WorldSize) { Location.X = m_WorldSize; bChanged = true; }
				if (Location.Y > m_WorldSize) { Location.Y = -m_WorldSize; bChanged = true; }
				else if (Location.Y < -m_WorldSize) { Location.Y = m_WorldSize; bChanged = true; }
				
				if (bChanged)
				{
					pAgent->SetActorLocation(Location, false, nullptr, ETeleportType::TeleportPhysics);
				}
			}
			
			if (bUseSpatialPartitioning)
			{
				FVector2D currentPos(pAgent->GetActorLocation());
				pPartitionedSpace->UpdateAgentCell(*pAgent, OldPositions[i]);
				OldPositions[i] = currentPos;
			}
		}
	}
}

void Flock::RenderDebug()
{
	// Render all the agents in the flock
	for (ASteeringAgent* pAgent : Agents)
	{
		if (DebugRenderSteering && IsValid(pAgent))
		{
			FVector StartLoc = pAgent->GetActorLocation();
			FVector EndLoc = StartLoc + (pAgent->GetVelocity() * 0.5f);
           
			DrawDebugDirectionalArrow(pWorld, StartLoc, EndLoc, 20.f, FColor::Blue, false, -1.f, 0, 2.f);
		}
	}

	if (DebugRenderNeighborhood) RenderNeighborhood();
	if (bUseSpatialPartitioning && DebugRenderPartitions) pPartitionedSpace->RenderCells();
	
	// Evade in Red
	if (pAgentToEvade && IsValid(pAgentToEvade))
	{
		FVector predatorLoc = pAgentToEvade->GetActorLocation();
		DrawDebugCircle(pWorld, FVector(predatorLoc.X, predatorLoc.Y, 20.f), 300.f, 50, FColor::Red, false, -1.f, 0, 5.f, FVector(1, 0, 0), FVector(0, 1, 0), false);
	}
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	//UI
	{
		//Setup
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scroll wheel: zoom cam.");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();
		
		// Implement ImGUI checkboxes for debug rendering here
		ImGui::Checkbox("Steering", &DebugRenderSteering);
		ImGui::Checkbox("Neighborhood", &DebugRenderNeighborhood);
		ImGui::Checkbox("Use Spatial Partitioning", &bUseSpatialPartitioning);
		ImGui::Checkbox("Render Partitions", &DebugRenderPartitions);

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

		// Implement ImGUI sliders for steering behavior weights here
		ImGui::SliderFloat("Cohesion", &pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f);
		ImGui::SliderFloat("Separation", &pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight, 0.f, 1.f);
		ImGui::SliderFloat("Alignment", &pBlendedSteering->GetWeightedBehaviorsRef()[2].Weight, 0.f, 1.f);
		ImGui::SliderFloat("Seek", &pBlendedSteering->GetWeightedBehaviorsRef()[3].Weight, 0.f, 1.f);
		ImGui::SliderFloat("Wander", &pBlendedSteering->GetWeightedBehaviorsRef()[4].Weight, 0.f, 1.f);
		
		for (ASteeringAgent* pAgent : Agents) 
		{
			if (!pAgent) continue; 

			auto* pBehavior = static_cast<FlockingSteeringBehaviors*>(pAgent->GetSteeringBehavior());
			if (pBehavior && pBehavior->GetBlendedSteering())
			{
				auto& agentWeights = pBehavior->GetBlendedSteering()->GetWeightedBehaviorsRef();
				agentWeights[0].Weight = pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight; // Cohesion
				agentWeights[1].Weight = pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight; // Separation
				agentWeights[2].Weight = pBlendedSteering->GetWeightedBehaviorsRef()[2].Weight; // Alignment
				agentWeights[3].Weight = pBlendedSteering->GetWeightedBehaviorsRef()[3].Weight; // Seek
				agentWeights[4].Weight = pBlendedSteering->GetWeightedBehaviorsRef()[4].Weight; // Wander
			}
		}

		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
	// Debugger the neighbors for the first agent in the flock
	if (Agents.Num() == 0 || !IsValid(Agents[0])) 
	{
		return;
	}

	RegisterNeighbors(Agents[0]);
    
	DrawDebugCircle(pWorld, FVector(Agents[0]->GetActorLocation().X, Agents[0]->GetActorLocation().Y, 20.f), NeighborhoodRadius, 50, FColor::Purple, false, -1.f, 0, 5.f, FVector(1, 0, 0), FVector(0, 1, 0), false);

	int currentNrNeighbors = GetNrOfNeighbors();
	auto currentNeighbors = GetNeighbors();

	for (int i = 0; i < currentNrNeighbors; ++i) 
	{
		if (IsValid(currentNeighbors[i])) 
		{
			DrawDebugCircle(pWorld, FVector(currentNeighbors[i]->GetActorLocation().X, currentNeighbors[i]->GetActorLocation().Y, 20.f), 40.f, 50, FColor::Emerald, false, -1.f, 0, 5.f, FVector(1, 0, 0), FVector(0, 1, 0), false);
		}
	}
}

void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	if (bUseSpatialPartitioning)
	{
		pPartitionedSpace->RegisterNeighbors(*pAgent, NeighborhoodRadius);
	}
	else
	{
		NrOfNeighbors = 0;
		for (ASteeringAgent* pOtherAgent : Agents) // Loop through every agent in the flock, check if they are a neighbor
		{
			if (pOtherAgent != nullptr && pOtherAgent != pAgent)
			{
				float Distance = FVector2D::Distance(FVector2D(pAgent->GetActorLocation()), FVector2D(pOtherAgent->GetActorLocation()));
				if (Distance < NeighborhoodRadius)
				{
					Neighbors[NrOfNeighbors] = pOtherAgent;
					NrOfNeighbors++;
				}
			}
		}
	}
}

FVector2D Flock::GetAverageNeighborPos() const
{
	int currentNrNeighbors = GetNrOfNeighbors();
	if (currentNrNeighbors == 0) 
	{
		return FVector2D::ZeroVector;
	}

	FVector2D avgPosition = FVector2D::ZeroVector;
	auto currentNeighbors = GetNeighbors();
	
	// If using spatial partitioning, we can directly use the neighbors from the partitioned space, otherwise we use the neighbors from the memory pool
	if (bUseSpatialPartitioning)
	{
		currentNeighbors = pPartitionedSpace->GetNeighbors();
	}

	for (int i = 0; i < currentNrNeighbors; ++i) 
	{
		avgPosition += FVector2D(currentNeighbors[i]->GetActorLocation());
	}

	return avgPosition / static_cast<float>(currentNrNeighbors);
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	int currentNrNeighbors = GetNrOfNeighbors();
	if (currentNrNeighbors == 0) 
	{
		return FVector2D::ZeroVector;
	}

	FVector2D avgVelocity = FVector2D::ZeroVector;
	auto currentNeighbors = GetNeighbors();

	for (int i = 0; i < currentNrNeighbors; ++i) 
	{
		avgVelocity += FVector2D(currentNeighbors[i]->GetVelocity());
	}
	return avgVelocity / static_cast<float>(currentNrNeighbors);
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	// Implement
	for (ASteeringAgent* pAgent : Agents)
	{
		if (pAgent && pAgent->GetSteeringBehavior())
		{
			pAgent->GetSteeringBehavior()->SetTarget(Target);
		}
	}
}

int Flock::GetNrOfNeighbors() const
{
	if (bUseSpatialPartitioning) return pPartitionedSpace->GetNrOfNeighbors();
	return NrOfNeighbors;
}

const TArray<ASteeringAgent*>& Flock::GetNeighbors() const
{
	if (bUseSpatialPartitioning) return pPartitionedSpace->GetNeighbors();
	return Neighbors;
}