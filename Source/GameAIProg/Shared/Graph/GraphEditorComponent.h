#pragma once

#include "CoreMinimal.h"
#include "InputMappingContext.h"
#include "Components/ActorComponent.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "Graph.h"
#include "GraphNodeFactory.h"
#include "GraphRenderer.h"
#include "GraphEditorComponent.generated.h"


UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent), Blueprintable, BlueprintType)
class GAMEAIPROG_API UGraphEditorComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	// Input Mappings Context 
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
	UInputMappingContext* DefaultGraphEditingIMC;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
	UInputMappingContext* NodeHoverGraphEditingIMC;
	
	// Input Actions
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
	UInputAction* CreateNodeAction;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
	UInputAction* DestroyNodeAction;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
	UInputAction* MoveNodeAction;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
	UInputAction* CreateConnectionAction;
	
	// Sets default values for this component's properties
	UGraphEditorComponent();

	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	void SetEditedGraph(GameAI::Graph* pEditedGraph) { EditedGraph = pEditedGraph; }
	void SetNodeFactory(GameAI::IGraphNodeFactory* pNodeFactory) { NodeFactory = pNodeFactory; }
	bool HasGraphUpdated();

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

private:
	// Enhanced Input
	UPROPERTY()
	UEnhancedInputLocalPlayerSubsystem* EnhancedInputSubsystem{};
	UPROPERTY()
	UEnhancedInputComponent* EnhancedInputComponent{};
	UPROPERTY()
	APlayerController* PlayerController{};
	
	int DefaultInputPriority{1};
	int NodeHoverInputPriority{2}; // should be higher than default
	
	// Graph
	GameAI::Graph* EditedGraph{};
	GameAI::IGraphNodeFactory* NodeFactory{};
	int CurrentlyHoveredNodeId{GameAI::Graphs::InvalidNodeId};
	int LastTappedNodeId{GameAI::Graphs::InvalidNodeId};
	bool bIsMovingNode{};
	bool bIsCreatingConnection{};
	bool bHasGraphUpdated{};

	// Mouse
	FVector LatestMousePos{};
	
	void UpdateConditionalInputMapping();
	void UpdateCurrentlyHoveredNode();
	void UpdateNodeMovement();
	
	template<typename NodeType>
	float GetNodeRadius(NodeType& Node)
	{
		float NodeRadius = GameAI::Graphs::DefaultNodeDrawRadius;
		if constexpr(requires {{Node.GetRadius()} -> std::convertible_to<float>;})
		{
			NodeRadius = Node.GetRadius();
		}
		return NodeRadius;
	}

	bool GetEnhancedInput();
	bool IsHoveringOverNode();
	std::optional<FVector> GetMouseWorldPos() const;
	
	// Actions funcs
	void CreateNode();
	void InvalidateNode();
	void MoveNodeBegin();
	void MoveNodeEnd();
	
	// Connections
	void CreateConnection();
};
