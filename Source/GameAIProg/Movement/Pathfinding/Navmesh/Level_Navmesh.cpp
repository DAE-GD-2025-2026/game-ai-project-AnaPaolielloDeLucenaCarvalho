// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_Navmesh.h"

#include "NavigationSystem.h"
#include "AI/NavigationSystemBase.h"
#include "GraphTheory/Algorithms/AStar.h"
#include "GraphTheory/Algorithms/NavGraphPathfinding.h"
#include "NavMesh/RecastNavMesh.h"
#include "Runtime/Navmesh/Public/Detour/DetourNavMesh.h"
#include "Shared/GameAISpectator.h"

// Helper
FORCEINLINE FVector RecastToUnreal(const double* RecastVertex)
{
	return FVector(
		static_cast<float>(-RecastVertex[0]),   // X → -X
		static_cast<float>(-RecastVertex[2]),  // Z → -Y (handedness flip)
		static_cast<float>(RecastVertex[1])    // Y → Z
	);
}

// Sets default values
ALevel_Navmesh::ALevel_Navmesh()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_Navmesh::BeginPlay()
{
	Super::BeginPlay();
	TrimWorld->bShouldTrimWorld = false;
	
	if (AGameAISpectator* Player = Cast<AGameAISpectator>(PlayerController->GetPawnOrSpectator()); Player)
	{
		Player->SetCameraProjection(ECameraProjectionMode::Orthographic);
	}
	
	// Spawn the Agent
	Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, 
	FVector{2100.0,2100.0,90}, FRotator::ZeroRotator);
	Agent->SetDebugRenderingEnabled(false);
	Agent->SetSteeringBehavior(&PathFollow);
	
	auto NavPoly{std::make_unique<TriPolygon>()};
	for (TArray<FVector> const & Tri : ExtractNavMeshTris())
	{
		NavPoly->AddTriangle(Tri);
	}
	
	NavigationGraph = std::make_unique<GameAI::NavGraph>(std::move(NavPoly));
	Renderer = std::make_unique<GameAI::GraphRenderer>(GetWorld());
	Renderer->SetRenderOptions(GameAI::GraphRenderOptions{
		true, 
		false, 
		false, 
		true, 
		false
	});
}

// Called every frame
void ALevel_Navmesh::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	if (bDrawNavPoly)
	{
		NavigationGraph->GetNavPolygon()->DrawDebug(GetWorld(), FColor::Yellow);
	}
	
	if (bDrawNavPolyVertices)
	{
		for (const FVector& Vertex : NavigationGraph->GetNavPolygon()->GetVertices())
		{
			DrawDebugPoint(GetWorld(), Vertex, 10.0f, FColor::Cyan);
		}
	}
	
	if (bDrawNavGraph)
	{
		Renderer->RenderGraph(*NavigationGraph.get());
	}
	
	if (bDrawPath)
	{
		for (int PathIdx = 1; PathIdx < DebugDrawPath.size(); ++PathIdx)
		{
			DrawDebugLine(
				GetWorld(), 
				FVector{DebugDrawPath[PathIdx - 1], 5.0f}, 
				FVector{DebugDrawPath[PathIdx], 5.0f}, 
				FColor::Magenta, false, -1, 1, 10);
		}
	}
	
	// Todo: Draw the portals travelled through with SSFA
	// if (bDrawPortals)
	// {
	// 	
	// }
	
	UpdateImGui();
}

void ALevel_Navmesh::BindLevelInputActions()
{
	Super::BindLevelInputActions();
	
	PlayerEnhancedInputComponent->BindAction(SetTargetAction, ETriggerEvent::Triggered, 
		this, &ALevel_Navmesh::SetTarget);
}

void ALevel_Navmesh::UpdateImGui()
{
#pragma region UI
	//UI
	{
		//Setup
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: Set Target");
		ImGui::Unindent();

		/*Spacing*/ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing(); ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		/*Spacing*/ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing(); ImGui::Spacing();

		ImGui::Text("Navmesh Pathfinding");
		ImGui::Spacing();
		
		ImGui::Checkbox("NavPolyVertices", &bDrawNavPolyVertices);
		ImGui::Checkbox("NavPoly", &bDrawNavPoly);
		ImGui::Checkbox("NavGraph", &bDrawNavGraph);
		ImGui::Checkbox("Path", &bDrawPath);
		ImGui::Checkbox("Portals", &bDrawPortals);
		
		//End
		ImGui::End();
	}
#pragma endregion
}

TArray<TArray<FVector>> ALevel_Navmesh::ExtractNavMeshTris() const
{
	TArray<TArray<FVector>> Polys{};
	
	ANavigationData* NavData = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld())->GetDefaultNavDataInstance();
	if (dtNavMesh const * NavMesh = Cast<ARecastNavMesh>(NavData)->GetRecastMesh())
	{
		// Loop over all MeshTiles
		for (int TileIdx{0}; TileIdx < NavMesh->getMaxTiles(); ++TileIdx)
		{
			// check if tile is valid
			dtMeshTile const * Tile{NavMesh->getTile(TileIdx)};
			if (!Tile || !Tile->header || !Tile->polys) continue;
			
			for (int i = 0; i < Tile->header->detailMeshCount; ++i)
			{
				const dtPolyDetail* DetailMesh = &Tile->detailMeshes[i];
				const dtPoly* Poly = &Tile->polys[i];  // Corresponding base polygon
				
				for (int triIdx = 0; triIdx < DetailMesh->triCount; ++triIdx)
				{
					// Each detail triangle is stored as 4 bytes:
					// - 3 bytes: indices into either poly->verts or detailVerts
					// - 1 byte : flags (ignored)
					const unsigned char* TriData = &Tile->detailTris[(DetailMesh->triBase + triIdx) * 4];

					// For each of the three triangle corners
					TArray<FVector> TriVerts{};
					for (int corner = 0; corner < 3; ++corner)
					{
						unsigned char idx = TriData[corner];
						const double* Vert;

						if (idx < Poly->vertCount)
						{
							// Index into the base polygon's vertices
							Vert = &Tile->verts[Poly->verts[idx] * 3];
						}
						else
						{
							// Index into the detail vertices (height‑field detail)
							int detailVertIdx = DetailMesh->vertBase + (idx - Poly->vertCount);
							Vert = &Tile->detailVerts[detailVertIdx * 3];
						}

						// Convert to Unreal coordinates and add to output array
						TriVerts.Add(RecastToUnreal(Vert));
					}
					Polys.Add(TriVerts);
				}
			}
		}
	}
	
	return Polys;
}

void ALevel_Navmesh::SetTarget()
{
	GameAI::NavMeshPathfinding Pathfinder{};
	std::vector<FVector2D> Path =  Pathfinder.FindPath(Agent->GetPosition(), 
	FVector2D{LatestMouseWorldPos}, NavigationGraph.get());

	DebugDrawPath = Path;
	
	PathFollow.SetPath(Path);
	if (Path.size() > 0)
	{
		Agent->SetPosition(Path[0]);
	}
}
