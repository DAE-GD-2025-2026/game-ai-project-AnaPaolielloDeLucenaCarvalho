// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_GraphTheory.h"

#include "Algorithms/EulerianPath.h"
#include "Shared/GameAISpectator.h"
#include <map>

using namespace GameAI;

// Sets default values
ALevel_GraphTheory::ALevel_GraphTheory()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_GraphTheory::BeginPlay()
{
	Super::BeginPlay();
	
	// Add the graph editor to our player
	Renderer = GameAI::GraphRenderer{GetWorld()};
	
	PlayerController = GetWorld()->GetFirstPlayerController();
	
	if (PlayerController && GraphEditorClass)
	{
		PlayerGraphEditor = NewObject<UGraphEditorComponent>(PlayerController->GetPawn(), GraphEditorClass);
		PlayerGraphEditor->RegisterComponent();
		PlayerGraphEditor->SetEditedGraph(&Graph);
		PlayerGraphEditor->SetNodeFactory(&NodeFactory);
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("Unable to get PlayerController or GraphEditorClass is null"));
		//return;
	}
	
	// Make the view orthogonal for less perspective issues
	if (AGameAISpectator* Player = Cast<AGameAISpectator>(PlayerController->GetPawnOrSpectator()); Player)
	{
		Player->SetCameraProjection(ECameraProjectionMode::Orthographic);
	}
	
	// Make the graph and a couple connected nodes here...
	int n0 = Graph.AddNode(std::make_unique<GameAI::Node>(FVector2D(-200, 0)));
	int n1 = Graph.AddNode(std::make_unique<GameAI::Node>(FVector2D(200, 0)));
	Graph.AddConnection(n0, n1);
	
	// Spawn the Agent
	Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, 
	FVector{0,0,90}, FRotator::ZeroRotator);
	
	// Safety check before assigning
	if (Agent) 
	{
		Agent->SetSteeringBehavior(&PathFollow);
	}
}

void ALevel_GraphTheory::BeginDestroy()
{
	Super::BeginDestroy();
}

void ALevel_GraphTheory::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	if (PlayerController && PlayerGraphEditor)
	{
		FVector MouseWorldPos, MouseWorldDir;
		if (PlayerController->DeprojectMousePositionToWorld(MouseWorldPos, MouseWorldDir))
		{
			FHitResult HitResult;
			if (GetWorld()->LineTraceSingleByChannel(HitResult, MouseWorldPos, MouseWorldPos + (MouseWorldDir * 20000.f), ECC_Visibility))
			{
				PlayerGraphEditor->SetLatestMousePos(HitResult.Location);
			}
		}
	}
	
#pragma region UI
	{
		//Setup
		bool windowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
		ImGui::SetWindowFocus();
		ImGui::PushItemWidth(70);
		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
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
		ImGui::Spacing();

		ImGui::Text("Graph Theory");
		ImGui::Spacing();
		ImGui::Spacing();

		//End
		ImGui::End();
	}
#pragma endregion UI
	
	Renderer.RenderGraph(Graph);
	
	int currentConnCount = Graph.GetConnections().size();
	int currentNodeCount = Graph.GetActiveNodes().size();
	
	static int lastNodeCount = -1;
	
	if (currentConnCount != lastConnectionCount || currentNodeCount != lastNodeCount)
	{
		lastConnectionCount = currentConnCount;
		lastNodeCount = currentNodeCount;

		GameAI::EulerianPath ePath(&Graph);
		GameAI::Eulerianity e;
		auto trail = ePath.FindPath(e);
		UpdateAgentPath(trail);

		UpdateGraphColoring();
	}
}

void ALevel_GraphTheory::UpdateAgentPath(std::vector<Node*> const& Trail)
{
	std::vector<FVector2D> path{};
	
	// Convert Node vector to positions vector
	for (auto* node : Trail)
	{
		if (node)
		{
			path.push_back(node->GetPosition());
		}
	}

	PathFollow.SetPath(path);
	if (Agent && path.size() > 0)
	{
		Agent->SetPosition(path[0]);
	}
}

void ALevel_GraphTheory::UpdateGraphColoring()
{
    std::vector<GameAI::Node*> nodes = Graph.GetActiveNodes();
    if (nodes.empty()) return;

    // colors to pick from
    std::vector<FColor> Palette =
    {
        FColor::Red, 
    	FColor::Green, 
    	FColor::Blue, 
        FColor::Yellow, 
    	FColor::Cyan, 
    	FColor::Magenta, 
    	FColor::Orange, 
    	FColor::Purple
    };

    std::map<int, int> NodeColors;

    // all nodes  uncolored
    for (auto* node : nodes)
    {
        NodeColors[node->GetId()] = -1;
    }

    NodeColors[nodes[0]->GetId()] = 0;

    std::vector<bool> AvailableColors(Palette.size(), true);

    for (int i = 1; i < nodes.size(); ++i)
    {
        int currentNodeId = nodes[i]->GetId();

        std::fill(AvailableColors.begin(), AvailableColors.end(), true);

        // check neighbors of current node
        auto connections = Graph.FindConnectionsFrom(currentNodeId);
        for (auto* conn : connections)
        {
            int neighborId = conn->GetToId();
            
            // neighbor has a color, that color = UNAVAILABLE
            if (NodeColors[neighborId] != -1)
            {
                AvailableColors[NodeColors[neighborId]] = false;
            }
        }
        
        for (auto& conn : Graph.GetConnections())
        {
            if (conn->GetToId() == currentNodeId)
            {
                int neighborId = conn->GetFromId();
                if (NodeColors[neighborId] != -1)
                {
                    AvailableColors[NodeColors[neighborId]] = false;
                }
            }
        }

        // find available color
        int chosenColor = 0;
        for (int c = 0; c < AvailableColors.size(); ++c)
        {
            if (AvailableColors[c])
            {
                chosenColor = c;
                break;
            }
        }

        NodeColors[currentNodeId] = chosenColor;
    }

    std::vector<std::pair<int, FColor>> Highlights;
    for (auto const& [nodeId, colorIndex] : NodeColors)
    {
        if (colorIndex >= 0 && colorIndex < Palette.size()) 
        {
            Highlights.push_back({nodeId, Palette[colorIndex]});
        }
    }

    Renderer.SetHighlightedNodes(Highlights);
    
    GameAI::GraphRenderOptions Options;
    Options.bDrawHighlightedNodes = true;
    Options.bDrawConnections = true;
    Renderer.SetRenderOptions(Options);
}




