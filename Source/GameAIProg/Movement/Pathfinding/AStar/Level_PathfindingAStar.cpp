// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_PathfindingAStar.h"

#include "GraphTheory/Algorithms/AStar.h"
#include "GraphTheory/Algorithms/BFS.h"
#include "GraphTheory/Algorithms/Heuristics.h"
#include "Shared/GameAISpectator.h"

using namespace GameAI;

// Sets default values
ALevel_PathfindingAStar::ALevel_PathfindingAStar()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_PathfindingAStar::BeginPlay()
{
	Super::BeginPlay();
	
	// Disable trimworld
	TrimWorld->bShouldTrimWorld = false;
	
	// Make the view orthogonal for less perspective issues
	if (PlayerController = Cast<APlayerController>(GetWorld()->GetFirstLocalPlayerFromController()->PlayerController); PlayerController)
	{
		if (AGameAISpectator* Player = Cast<AGameAISpectator>(PlayerController->GetPawnOrSpectator()); Player)
		{
			Player->SetCameraProjection(ECameraProjectionMode::Orthographic);
		}
	}
	
	// Spawn the Agent
	Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, 
	FVector{0,0,90}, FRotator::ZeroRotator);
	Agent->SetDebugRenderingEnabled(false);
	Agent->SetSteeringBehavior(&PathFollow);
	
	// Create graph & renderer
	Renderer = new GraphRenderer{GetWorld()};
	GraphRenderOptions RenderOptions{};
	RenderOptions.bDrawConnectionWeights = false;
	RenderOptions.bDrawConnections = false;
	RenderOptions.bDrawNodeIds = false;
	RenderOptions.bDrawNodes = false;
	Renderer->SetRenderOptions(RenderOptions);
	
	NodeFactory = new TerrainNodeFactory{};
	TerrainGraph = new TerrainGridGraph{NodeFactory, 10, 10, 200.0f, 1.0f, 
		FVector2D{-1000.0f, -1000.0f}, false};
	
	CalculatePath();
}

void ALevel_PathfindingAStar::BeginDestroy()
{
	Super::BeginDestroy();
	
	delete Renderer;
	delete TerrainGraph;
	delete NodeFactory;
}

void ALevel_PathfindingAStar::BindLevelInputActions()
{
	Super::BindLevelInputActions();
	
	// Path
	PlayerEnhancedInputComponent->BindAction(SetStartNodeAction, ETriggerEvent::Triggered, this, 
		&ALevel_PathfindingAStar::SetStartNodeId);
	PlayerEnhancedInputComponent->BindAction(SetEndNodeAction, ETriggerEvent::Triggered, this, 
		&ALevel_PathfindingAStar::SetEndNodeId);
	
	// Terrain
	PlayerEnhancedInputComponent->BindAction(SetNodeTerrainClearAction, ETriggerEvent::Triggered, this, 
		&ALevel_PathfindingAStar::SetNodeTerrain, TerrainNode::Type::Clear);
	PlayerEnhancedInputComponent->BindAction(SetNodeTerrainMudAction, ETriggerEvent::Triggered, this, 
		&ALevel_PathfindingAStar::SetNodeTerrain, TerrainNode::Type::Mud);
	PlayerEnhancedInputComponent->BindAction(SetNodeTerrainWaterAction, ETriggerEvent::Triggered, this, 
		&ALevel_PathfindingAStar::SetNodeTerrain, TerrainNode::Type::Water);
}

// Called every frame
void ALevel_PathfindingAStar::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	UpdateImGui();
	
	Renderer->RenderGraph(*TerrainGraph);
	TerrainGraph->DebugDrawCells(GetWorld());
	TerrainGraph->DrawTerrain(GetWorld());
	// TODO implement conditional debug draws
}

void ALevel_PathfindingAStar::CalculatePath()
{
	// Find a path
	//Check if valid start and end node exist
	if (PathStartNodeId != Graphs::InvalidNodeId
		&& PathEndNodeId != Graphs::InvalidNodeId
		&& PathStartNodeId != PathEndNodeId)
	{
		//Select (uncomment) BFS Pathfinding or A* Pathfinding
		BFS pathfinder = BFS(TerrainGraph);
		// AStar pathfinder = AStar(TerrainGraph, HeuristicFunction);
		TerrainNode* const startNode = TerrainGraph->GetNodeAs<TerrainNode>(PathStartNodeId);
		TerrainNode* const endNode = TerrainGraph->GetNodeAs<TerrainNode>(PathEndNodeId);

		FoundPath = pathfinder.FindPath(startNode, endNode);
		// std::cout << "New path calculated using " << typeid(pathfinder).name() << std::endl;
		UE_LOG(LogTemp, Log, TEXT("New path calculated using %hs"), typeid(pathfinder).name());
		UpdateAgentPath(FoundPath);
	}
	else
	{
		UE_LOG(LogTemp, Log, TEXT("No valid start & end node... Start: %d, End: %d"), PathStartNodeId, PathEndNodeId);
		// std::cout << "No valid start and end node..." << std::endl;
		FoundPath.clear();
	}
	
	// Update the highlighted nodes in the renderer
	std::vector<std::pair<int, FColor>> PathToHighlight{};
	PathToHighlight.push_back({PathStartNodeId, FColor::Green});
	if (!FoundPath.empty())
	{
		for (int Idx = 1; Idx < FoundPath.size() - 1; ++Idx)
		{
			PathToHighlight.push_back({FoundPath[Idx]->GetId(), FColor::Yellow});
		}
	}
	PathToHighlight.push_back({PathEndNodeId, FColor::Red});
	Renderer->SetHighlightedNodes(PathToHighlight);
}

void ALevel_PathfindingAStar::UpdateAgentPath(std::vector<Node*> const& Path)
{
	std::vector<FVector2D> pathPositions{};
	pathPositions.reserve(Path.size());
	for (Node* const pNode : Path)
	{
		pathPositions.emplace_back(pNode->GetPosition());
	}

	PathFollow.SetPath(pathPositions);
	if (pathPositions.size() > 0)
	{
		Agent->SetPosition(pathPositions[0]);
	}
}

void ALevel_PathfindingAStar::UpdateImGui()
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
		ImGui::Text("LMB: Set Path Start");
		ImGui::Text("RMB: Set Path End");
		ImGui::Text("1: Set terrain to Clear");
		ImGui::Text("2: Set terrain to Mud");
		ImGui::Text("3: Set terrain to Water");
		ImGui::Unindent();

		/*Spacing*/ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing(); ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		/*Spacing*/ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing(); ImGui::Spacing();

		ImGui::Text("A* Pathfinding");
		ImGui::Spacing();
		
		// TODO conditional debug draws
		// ImGui::Checkbox("Grid", &bDrawGrid);
		// ImGui::Checkbox("NodeNumbers", &bDrawNodeNumbers);
		// ImGui::Checkbox("Connections", &bDrawConnections);
		// ImGui::Checkbox("Connections Costs", &bDrawConnectionsCosts);
		if (ImGui::Combo("", &SelectedHeuristic, "Manhattan\0Euclidean\0SqEuclidean\0Octile\0Chebyshev", 4))
		{
			switch (SelectedHeuristic)
			{
			case 0:
				HeuristicFunction = HeuristicFunctions::Manhattan;
				break;
			case 1:
				HeuristicFunction = HeuristicFunctions::Euclidean;
				break;
			case 2:
				HeuristicFunction = HeuristicFunctions::SqEuclidean;
				break;
			case 3:
				HeuristicFunction = HeuristicFunctions::Octile;
				break;
			default:
			case 4:
				HeuristicFunction = HeuristicFunctions::Chebyshev;
				break;
			}
		}
		ImGui::Spacing();

		//End
		ImGui::End();
	}
#pragma endregion
}

void ALevel_PathfindingAStar::SetStartNodeId()
{
	int const NewStart = TerrainGraph->GetNodeIdAtPosition(FVector2D{LatestMouseWorldPos});
	if (NewStart >= 0 && NewStart != PathEndNodeId)
	{
		PathStartNodeId = NewStart;
		CalculatePath();
	}
}

void ALevel_PathfindingAStar::SetEndNodeId()
{
	int const NewEnd = TerrainGraph->GetNodeIdAtPosition(FVector2D{LatestMouseWorldPos});
	if (NewEnd >= 0 && NewEnd != PathStartNodeId)
	{
		PathEndNodeId = NewEnd;
		CalculatePath();
	}
}

void ALevel_PathfindingAStar::SetNodeTerrain(TerrainNode::Type TerrainType)
{
	TerrainGraph->PaintNodeAtPosition(FVector2D{LatestMouseWorldPos}, TerrainType);
	CalculatePath(); // since connections may change
}



