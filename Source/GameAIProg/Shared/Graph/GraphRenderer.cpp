#include "GraphRenderer.h"

namespace GameAI
{
	GraphRenderer::GraphRenderer(UWorld* world)
	: World{world}
	{
	}

	void GraphRenderer::SetRenderOptions(GraphRenderOptions const& NewOptions)
	{
		Options = NewOptions;
	}

	void GraphRenderer::SetHighlightedNodes(std::vector<std::pair<int, FColor>> const& NodesToHighlight)
	{
		HighlightedNodes.clear();
		HighlightedNodes = NodesToHighlight;
	}

	void GraphRenderer::RenderGraph(Graph const &  Graph) const
	{
		// Draw connection
		if (Options.bDrawConnections)
		{
			for (auto& Connection : Graph.GetConnections())
			{
				DrawConnection(Graph, *Connection);
			}
		}
		
		for (auto & Node : Graph.GetNodes())
		{
			if (Node->GetId() != Graphs::InvalidNodeId) // We skip invalid nodes
			{
				if (auto FoundHighlight = std::ranges::find_if(HighlightedNodes, 
					[&](std::pair<int, FColor> const& Highlight){ return Node->GetId() == Highlight.first; });
					FoundHighlight != HighlightedNodes.end())
				{
					// If in highlights use the color override
					if (Options.bDrawHighlightedNodes)
					{
						DrawNodeSphere(*Node, Graphs::DefaultNodeDrawRadius, FoundHighlight->second);
					}
				}
					
				if (Options.bDrawNodes)
				{
					DrawNode(*Node);
				}
			}
		}
	}
}


