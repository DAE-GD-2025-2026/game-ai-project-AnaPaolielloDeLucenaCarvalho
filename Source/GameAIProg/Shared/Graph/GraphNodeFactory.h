#pragma once
#include "Graph.h"
#include "GraphConcepts.h"

namespace GameAI
{
	class IGraphNodeFactory
	{
	public:
		IGraphNodeFactory() = default;
		virtual ~IGraphNodeFactory() = default;

		virtual std::unique_ptr<Node> const CreateNode(FVector2D const & Position) const = 0;
		virtual std::unique_ptr<Node> const CloneNode(Node const & Other) const = 0;
	};
	
	template <typename NodeType>
	requires GameAI::is_drawable_node<NodeType> && std::derived_from<NodeType, Node>
	class GraphNodeFactory : public IGraphNodeFactory
	{
	public:
		
		virtual ~GraphNodeFactory() override = default;
		virtual std::unique_ptr<Node> const CreateNode(const FVector2D& Position) const override
		{
			// You would need to make your own implementation of this
			return std::unique_ptr<Node>(new NodeType(Position));
		}
		virtual std::unique_ptr<Node> const CloneNode(const Node& Other) const override
		{
			// You would need to make your own implementation of this
			return std::unique_ptr<Node>(new NodeType(Other));
		}
	};
	
	/*
	 * Template specialization example
		
		template <>
		class GraphNodeFactory<SpecialNode> : public IGraphNodeFactory
		{
			public:
				std::unique_ptr<Node> CreateNode(const FVector2D& Pos) const override {
					// Special logic for SpecialNode
					auto node = std::make_unique<SpecialNode>(Pos, *more state here*, 42);
				node->InitializeSomething();
				return node;
			}
			
			std::unique_ptr<Node> CloneNode(const Node& Other) const override {
				auto& other = static_cast<const SpecialNode&>(Other);
				return std::make_unique<SpecialNode>(other, *more state here*);
			}
		};
	 */
}
