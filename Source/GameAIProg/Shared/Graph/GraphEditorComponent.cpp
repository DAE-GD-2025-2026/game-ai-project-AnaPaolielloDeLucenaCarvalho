#include "GraphEditorComponent.h"

#include "Graph.h"
#include "GraphRenderer.h"


// Sets default values for this component's properties
UGraphEditorComponent::UGraphEditorComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;
}


bool UGraphEditorComponent::HasGraphUpdated()
{
	return bHasGraphUpdated;
}

// Called when the game starts
void UGraphEditorComponent::BeginPlay()
{
	Super::BeginPlay();
	
	// Get vars ready
	if (!GetEnhancedInput())
	{
		UE_LOG(LogTemp, Error, TEXT("Could not get EnhancedInputComponent/Subsystem!"));
		return;
	};

	// Bind default mapping
	EnhancedInputSubsystem->AddMappingContext(DefaultGraphEditingIMC, DefaultInputPriority);
	
	// Bind actions
	// Nodes
	EnhancedInputComponent->BindAction(CreateNodeAction, ETriggerEvent::Triggered, this, &UGraphEditorComponent::CreateNode);
	EnhancedInputComponent->BindAction(DestroyNodeAction, ETriggerEvent::Triggered, this, &UGraphEditorComponent::InvalidateNode);
	EnhancedInputComponent->BindAction(MoveNodeAction, ETriggerEvent::Triggered, this, &UGraphEditorComponent::MoveNodeBegin);
	EnhancedInputComponent->BindAction(MoveNodeAction, ETriggerEvent::Completed, this, &UGraphEditorComponent::MoveNodeEnd);
	
	// Connections
	EnhancedInputComponent->BindAction(CreateConnectionAction, ETriggerEvent::Triggered, this, &UGraphEditorComponent::CreateConnection);
}

// Called every frame
void UGraphEditorComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!EditedGraph || !NodeFactory)
	{
		// We can't do anything without a graph or node factory...
		return;
	}
	
	// Mark as not updated this frame, will be changed in actions
	bHasGraphUpdated = false;
	
	// Update the current node we're hovering;
	UpdateCurrentlyHoveredNode();
	
	// Check if NodeHover mapping should be applied or taken away
	UpdateConditionalInputMapping();
	
	// Update latest MousePos
	if (auto MousePosOptional = GetMouseWorldPos(); MousePosOptional.has_value())
	{
		LatestMousePos = *MousePosOptional;
	}
	
	// If we're moving a node, move it to the mousepos
	UpdateNodeMovement();

	// if we're adding a connection, debug draw it 
	if (bIsCreatingConnection)
	{
		FVector StartLine{EditedGraph->GetNode(LastTappedNodeId)->GetPosition(), GameAI::Graphs::DefaultGraphDrawHeight};
		DrawDebugLine(GetWorld(), StartLine, LatestMousePos, FColor::Green);
	}
}

void UGraphEditorComponent::UpdateConditionalInputMapping()
{
	if (IsHoveringOverNode() && 
		!EnhancedInputSubsystem->HasMappingContext(NodeHoverGraphEditingIMC))
	{
		EnhancedInputSubsystem->AddMappingContext(NodeHoverGraphEditingIMC, 3);
	}
	else if (!IsHoveringOverNode() && 
		EnhancedInputSubsystem->HasMappingContext(NodeHoverGraphEditingIMC))
	{
		EnhancedInputSubsystem->RemoveMappingContext(NodeHoverGraphEditingIMC);
	}
}

void UGraphEditorComponent::UpdateCurrentlyHoveredNode()
{
	for (auto& Node : EditedGraph->GetNodes())
	{
		if (Node->GetId() == GameAI::Graphs::InvalidNodeId) continue;
		
		float NodeRadius = GetNodeRadius(Node);

		FVector2D const MouseToNode{Node->GetPosition() - FVector2D{LatestMousePos}};
		if (MouseToNode.SquaredLength() < NodeRadius * NodeRadius)
		{
			if (CurrentlyHoveredNodeId == GameAI::Graphs::InvalidNodeId)
			{
				CurrentlyHoveredNodeId = Node->GetId();
			}
			return;
		}
	}
	
	// Otherwise invalidate
	CurrentlyHoveredNodeId = GameAI::Graphs::InvalidNodeId;
}

void UGraphEditorComponent::UpdateNodeMovement()
{
	if (!bIsMovingNode)
	{
		return;
	}
	
	EditedGraph->GetNode(CurrentlyHoveredNodeId)->SetPosition(FVector2D{LatestMousePos});
}

bool UGraphEditorComponent::GetEnhancedInput()
{
	auto* PlayerOwner = Cast<APawn>(GetOwner());
	if (auto* pPlayerController = Cast<APlayerController>(PlayerOwner->GetController()); 
		PlayerOwner && pPlayerController)
	{
		// We're also setting the controller but shhhhh
		PlayerController = pPlayerController;
		
		if (auto* LocalPlayer = pPlayerController->GetLocalPlayer(); LocalPlayer)
		{
			if (auto* Subsystem = LocalPlayer->GetSubsystem<UEnhancedInputLocalPlayerSubsystem>(); Subsystem)
			{
				EnhancedInputSubsystem = Subsystem;
			}
		}
		
		if (auto* Component = Cast<UEnhancedInputComponent>(pPlayerController->InputComponent); Component)
		{
			EnhancedInputComponent = Component;
		}
	}
	
	// Return success
	return EnhancedInputSubsystem && EnhancedInputComponent;
}

bool UGraphEditorComponent::IsHoveringOverNode()
{
	return CurrentlyHoveredNodeId != GameAI::Graphs::InvalidNodeId;
}

std::optional<FVector> UGraphEditorComponent::GetMouseWorldPos() const
{
	if (!PlayerController)
	{
		// we don't have a player controller to get the mouse pos from!
		return std::nullopt;
	}
	
	FVector MouseWorldPos{};
	FVector MouseWorldDirection{};
	PlayerController->DeprojectMousePositionToWorld(MouseWorldPos, MouseWorldDirection);
	
	// TODO FIXME move to level and just provide a set latest mousepos func?
	float MaxTraceDistance{20000.0f};

	if (FHitResult HitResult{}; 
		GetWorld()->LineTraceSingleByChannel(HitResult, MouseWorldPos,
			MouseWorldDirection * MaxTraceDistance,ECC_Visibility))
	{
		return HitResult.Location;
	}
	
	return std::nullopt;
}

void UGraphEditorComponent::CreateNode()
{
	if (!EditedGraph || CurrentlyHoveredNodeId != GameAI::Graphs::InvalidNodeId) return;
	
	EditedGraph->AddNode(NodeFactory->CreateNode(FVector2D{LatestMousePos.X, LatestMousePos.Y}));
	bHasGraphUpdated = true;
}

void UGraphEditorComponent::InvalidateNode()
{
	if (!EditedGraph || CurrentlyHoveredNodeId == GameAI::Graphs::InvalidNodeId) return;
	
	EditedGraph->RemoveNode(CurrentlyHoveredNodeId);
	bHasGraphUpdated = true;
}

void UGraphEditorComponent::MoveNodeBegin()
{
	if (!EditedGraph || CurrentlyHoveredNodeId == GameAI::Graphs::InvalidNodeId) return;
	bIsMovingNode = true;
	bHasGraphUpdated = true;
}

void UGraphEditorComponent::MoveNodeEnd()
{
	bIsMovingNode = false;
	bHasGraphUpdated = true;
}

void UGraphEditorComponent::CreateConnection()
{
	if (!EditedGraph || CurrentlyHoveredNodeId == GameAI::Graphs::InvalidNodeId) return;

	if (!bIsCreatingConnection)
	{
		// Init state
		// Store the at that time hovered node
		bIsCreatingConnection = true;
		LastTappedNodeId = CurrentlyHoveredNodeId;
	}
	else
	{
		// Attempt to create a connection between the new node id and the old
		if (CurrentlyHoveredNodeId != LastTappedNodeId)
		{
			EditedGraph->AddConnection(LastTappedNodeId, CurrentlyHoveredNodeId);
			bHasGraphUpdated = true;
		}
		
		// Reset state
		bIsCreatingConnection = false;
		LastTappedNodeId = GameAI::Graphs::InvalidNodeId;
	}
}




