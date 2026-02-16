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

	// TODO: initialize the flock and the memory pool
	Neighbors.SetNum(FlockSize); // Memory pool for the neighbors (whole flock minus one)

	for (int i = 0; i < FlockSize; ++i) // Spawn agents at a random location and make them flocking
	{
		FVector2D SpawnPos = FVector2D(FMath::RandRange(-WorldSize, WorldSize), FMath::RandRange(-WorldSize, WorldSize));
		Agents[i] = pWorld->SpawnActor<ASteeringAgent>(AgentClass, SpawnPos, FRotator::ZeroRotator);
		Agents[i]->SetSteeringBehavior(new FlockingSteeringBehaviors(Agents[i], this));
		Agents[i]->SetTrimWorld(bTrimWorld);
	}
}

Flock::~Flock()
{
 // TODO: Cleanup any additional data
}

void Flock::Tick(float DeltaTime)
{
	// TODO: update the flock
	// TODO: for every agent:
	// TODO: register the neighbors for this agent (-> fill the memory pool with the neighbors for the currently evaluated agent)
	// TODO: update the agent (-> the steeringbehaviors use the neighbors in the memory pool)
	// TODO: trim the agent to the world
}

void Flock::RenderDebug()
{
 // TODO: Render all the agents in the flock
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

  // TODO: implement ImGUI checkboxes for debug rendering here

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

  // TODO: implement ImGUI sliders for steering behavior weights here
		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
	// TODO: Debugrender the neighbors for the first agent in the flock
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	// Implement
	NrOfNeighbors = 0;

	for (ASteeringAgent* pOtherAgent : Agents) // Loop through all agents in the flock
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
	FVector2D avgPosition = FVector2D::ZeroVector;

	// Implement
	for (int i = 0; i < NrOfNeighbors; ++i) // Loop through the neighbors in the memory pool and sum up their positions
	{
		avgPosition += Neighbors[i]->GetLocation();
	}

	if (NrOfNeighbors > 0)
	{
		avgPosition /= NrOfNeighbors; // Divide the summed up position by the number of neighbors to get the average
	}
	
	return avgPosition;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D avgVelocity = FVector2D::ZeroVector;

	// Implement
	for (int i = 0; i < NrOfNeighbors; ++i) // Loop through the neighbors in the memory pool and sum up their velocities
	{
		avgVelocity += Neighbors[i]->GetVelocity();
	}

	if (NrOfNeighbors > 0)
	{
		avgVelocity /= NrOfNeighbors; // Divide the summed up velocity by the number of neighbors to get the average
	}

	return avgVelocity;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
 // TODO: Implement
}

