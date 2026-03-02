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

	void GraphRenderer::RenderGraph(Graph const &  Graph) const
	{
		if (Options.bDrawNodes)
		{
			for (auto & Node : Graph.GetNodes())
			{
				if (Node->GetId() != Graphs::InvalidNodeId)
				{
					// We skip invalid nodes
					DrawNode(*Node);
				}
			}
		}

		if (Options.bDrawNodes)
		{
			for (auto& Connection : Graph.GetConnections())
			{
				DrawConnection(Graph, *Connection);
			}
		}

	}
}


