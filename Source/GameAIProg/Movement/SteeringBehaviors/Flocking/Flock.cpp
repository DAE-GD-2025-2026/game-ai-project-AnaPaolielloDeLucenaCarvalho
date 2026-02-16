#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"


Flock::Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize,
	float WorldSize,
	ASteeringAgent* const pAgentToEvade,
	bool bTrimWorld)
	: pWorld{pWorld}
	, FlockSize{ FlockSize }
	, pAgentToEvade{pAgentToEvade}
{
	Agents.SetNum(FlockSize);

	// Initialize the flock and the memory pool
	Neighbors.SetNum(FlockSize); // Memory pool for the neighbors (whole flock minus one)

	pBlendedSteering = std::make_unique<BlendedSteering>();
	pPrioritySteering = std::make_unique<PrioritySteering>();

	for (int i = 0; i < FlockSize; ++i) // Spawn agents at a random location and make them flocking
	{
		FVector2D SpawnPos = FVector2D(FMath::RandRange(-WorldSize, WorldSize), FMath::RandRange(-WorldSize, WorldSize));
		Agents[i] = pWorld->SpawnActor<ASteeringAgent>(AgentClass, FVector(SpawnPos.X, SpawnPos.Y, 0.f), FRotator::ZeroRotator);

		Agents[i]->SetSteeringBehavior(new FlockingSteeringBehaviors(Agents[i], this));
		Agents[i]->SetTrimWorld(bTrimWorld);
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
	// Update the agent (-> the steeringbehaviors use the neighbors in the memory pool)
	// Trim the agent to the world

	for (ASteeringAgent* pAgent : Agents) // Loop through every agent in the flock
	{
		RegisterNeighbors(pAgent);

		pAgent->Update(DeltaTime);
		pAgent->TrimToWorld();
	}
}

void Flock::RenderDebug()
{
	// Render all the agents in the flock
	for (ASteeringAgent* pAgent : Agents)
	{
		if (DebugRenderSteering)
		{
			pAgent->RenderDebug();
		}
	}

	if (DebugRenderNeighborhood)
	{
		RenderNeighborhood();
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
		ImGui::Text("Scrollwheel: zoom cam.");
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

		// TODO: Implement ImGUI checkboxes for debug rendering here
		ImGui::Checkbox("Steering", &DebugRenderSteering);
		ImGui::Checkbox("Neighborhood", &DebugRenderNeighborhood);

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

		// TOOD: Implement ImGUI sliders for steering behavior weights here
		ImGui::SliderFloat("Cohesion", &pBlendedSteering->GetWeightedBehaviors()[0].Weight, 0.f, 1.f);
		ImGui::SliderFloat("Separation", &pBlendedSteering->GetWeightedBehaviors()[1].Weight, 0.f, 1.f);
		ImGui::SliderFloat("Alignment", &pBlendedSteering->GetWeightedBehaviors()[2].Weight, 0.f, 1.f);

		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
	// Debugrender the neighbors for the first agent in the flock
	if (Agents.Num() == 0 || !Agents[0]) // Check if there are agents in the flock and if the first agent is valid
	{
		return;
	}

	RegisterNeighbors(Agents[0]);

	DrawDebugCircle(pWorld, FVector(Agents[0]->GetLocation().X, Agents[0]->GetLocation().Y, 0.f), NeighborhoodRadius, 24, FColor::White, false, -1.f, 0, 2.f, FVector(1, 0, 0), FVector(0, 1, 0), false);

	for (int i = 0; i < NrOfNeighbors; ++i) // Loop through neighbors in memory pool and draw a line from 1 agent to neighbors
	{
		DrawDebugLine(pWorld, FVector(Agents[0]->GetLocation().X, Agents[0]->GetLocation().Y, 0.f), FVector(Neighbors[i]->GetLocation().X, Neighbors[i]->GetLocation().Y, 0.f), FColor::Green, false, -1.f, 0, 1.f);
	}
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	// Implement
	NrOfNeighbors = 0;

	for (ASteeringAgent* pOtherAgent : Agents) // Loop through every agent in the flock, check if they are a neighbor
	{
		if (pOtherAgent != pAgent)
		{
			float Distance = FVector2D::Distance(pAgent->GetLocation(), pOtherAgent->GetLocation());
			if (Distance < NeighborhoodRadius)
			{
				Neighbors[NrOfNeighbors] = pOtherAgent;
				NrOfNeighbors++;
			}
		}
	}
}
#endif

FVector2D Flock::GetAverageNeighborPos() const
{
	if (NrOfNeighbors == 0) // Is there neighbors in memory pool
	{
		return FVector2D::ZeroVector;
	}

	FVector2D avgPosition = FVector2D::ZeroVector;

	// Implement
	for (int i = 0; i < NrOfNeighbors; ++i) // Loop through neighbors in memory pool and add up their position
	{
		avgPosition += Neighbors[i]->GetLocation();
	}
	return avgPosition / static_cast<float>(NrOfNeighbors);
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	if (NrOfNeighbors == 0) // Is there neighbors in memory pool
	{
		return FVector2D::ZeroVector;
	}

	FVector2D avgVelocity = FVector2D::ZeroVector;

	// Implement
	for (int i = 0; i < NrOfNeighbors; ++i) // Loop through neighbors in memory pool and add up their velocity
	{
		avgVelocity += Neighbors[i]->GetVelocity();
	}
	return avgVelocity / static_cast<float>(NrOfNeighbors);
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	// Implement
	for (ASteeringAgent* pAgent : Agents)
	{
		pAgent->SetSteeringBehaviorTarget(Target, ESteeringBehavior::Seek);
	}
}

