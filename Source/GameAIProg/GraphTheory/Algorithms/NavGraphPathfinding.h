#pragma once
#include <vector>

namespace GameAI
{
	class NavGraph;

	struct NavLine
	{
		FVector2D P1, P2;	
	};

	class NavMeshPathfinding
	{
	public:
		static std::vector<FVector2D> FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph,
			std::vector<FVector2D>& debugNodePositions, std::vector<NavLine>& debugPortals);
		static std::vector<FVector2D> FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph);
	};
}
