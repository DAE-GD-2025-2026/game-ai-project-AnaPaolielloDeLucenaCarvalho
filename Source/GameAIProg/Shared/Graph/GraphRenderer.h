#pragma once
#include "Graph.h"
#include "GraphConcepts.h"

namespace GameAI
{
	namespace Graphs
	{
		constexpr float DefaultGraphDrawHeight{1.0f};
		constexpr float DefaultNodeDrawRadius{30.0f};
		constexpr float DefaultConnectionDrawArrowSize{400.0f};
		constexpr int DefaultNodeDrawSegments{16};
		FColor const DefaultNodeDrawColor{FColor::Red};
		FColor const DefaultConnectionDrawColor{FColor::Blue};
	}
	
	struct GraphRenderOptions
	{
		// Nodes
		bool bDrawNodes{true};
		bool bDrawNodeIds{true};
		
		// Connections
		bool bDrawConnections{true};
		bool bDrawConnectionWeights{true};
	};
	
	class GraphRenderer final
	{
	public:
		explicit GraphRenderer(UWorld* world);
		~GraphRenderer() = default;
		
		void SetRenderOptions(GraphRenderOptions const & NewOptions);
		
		void RenderGraph(Graph const & Graph) const;

	private:
		UWorld* World;
		GraphRenderOptions Options{};
		
		template <typename NodeType>
		requires GameAI::is_drawable_node<NodeType>
		void DrawNode(NodeType const & Node, float Radius = Graphs::DefaultNodeDrawRadius,
			FColor Color = Graphs::DefaultNodeDrawColor, float DrawHeight = Graphs::DefaultGraphDrawHeight,
			int Segments = Graphs::DefaultNodeDrawSegments) const;
		
		template <typename ConnectionType>
		requires GameAI::is_connection_type<ConnectionType>
		void DrawConnection(Graph const & Graph, ConnectionType const & Connection, 
			FColor const & Color = Graphs::DefaultConnectionDrawColor) const;
	};
	
	template <typename NodeType>
	requires GameAI::is_drawable_node<NodeType>
	void GraphRenderer::DrawNode(NodeType const & Node, float Radius,
		FColor Color, float DrawHeight, int Segments) const
	{
		// If data exists, use it instead
		if constexpr (requires {{Node.GetRadius()} -> std::convertible_to<float>;})
		{
			Radius = Node.GetRadius();
		}
		if constexpr (requires {{ Node.GetColor() } -> std::convertible_to<FColor>;})
		{
			Color = Node.GetColor();
		}
		
		FTransform DrawTransform{FVector::UpVector.ToOrientationRotator(),
			FVector{Node.GetPosition(), DrawHeight}};
		DrawDebugCircle(World, DrawTransform.ToMatrixNoScale(), Radius, Segments, Color, 
			false, -1, 0, 3);

		if (Options.bDrawConnections)
		{
			FString NodeId{FString::Printf(TEXT("%d"), static_cast<int>(Node.GetId()))};
			DrawDebugString(World, DrawTransform.GetLocation(), NodeId,nullptr, FColor::White, 0);
		}
	}
	
	template <typename ConnectionType>
	requires GameAI::is_connection_type<ConnectionType>
	void GraphRenderer::DrawConnection(Graph const & Graph, ConnectionType const & Connection, FColor const & Color) const
	{
		// If data exists, use it instead
		if constexpr (requires {{ Connection.GetColor() } -> std::convertible_to<FColor>;})
		{
			Color = Connection.GetColor();
		}
		
		FVector Start{Graph.GetNode(Connection.GetFromId())->GetPosition(), Graphs::DefaultGraphDrawHeight};
		FVector End{Graph.GetNode(Connection.GetToId())->GetPosition(), Graphs::DefaultGraphDrawHeight};
	
		if (!Graph.GetIsDirectional())
		{
			DrawDebugLine(World, Start, End, Color, 
				false, -1, 0, 5);
		}
		else
		{
			DrawDebugDirectionalArrow(World, Start, End, Graphs::DefaultConnectionDrawArrowSize, Color,
				false, -1, 0, 5);
		}
		
		// Draw weight (if not 0)
		if (Options.bDrawConnectionWeights && !FMath::IsNearlyZero(Connection.GetWeight()))
		{
			FVector Middle = End + (Start - End) / 2;
			
			FString WeightString{FString::Printf(TEXT("%d"), static_cast<int>(Connection.GetWeight()))};
			DrawDebugString(World, Middle, WeightString,nullptr, FColor::White, 0);
		}
	}
}
