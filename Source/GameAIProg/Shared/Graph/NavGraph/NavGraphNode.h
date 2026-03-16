#pragma once
#include "Shared/Graph/Graph.h"

namespace GameAI
{
	class NavGraphNode : public Node
	{
	public:
		NavGraphNode(FVector2D const& Position, int NavPolyEdgeIdx)
			: Node(Position),
			  NavPolyEdgeIdx(NavPolyEdgeIdx)
		{
		}
		
		int GetEdgeIdx() const { return NavPolyEdgeIdx; }

	private:
		int NavPolyEdgeIdx;
	};
}
