#pragma once
#include "../GridGraph/GridGraph.h"

namespace GameAI
{
	class TerrainGridGraph : public GridGraph
	{
	public:
		TerrainGridGraph(TerrainNodeFactory* Factory, int Rows, int Cols, float CellSize, float Cost = 1.0f, 
			FVector2D const & OriginPosition = {0,0}, bool IsDiagonallyConnected = true, 
			bool IsDirectional = false);
		
		void PaintNodeAtPosition(FVector2D const & Position, TerrainNode::Type TypeToPaint);
		void DrawTerrain(UWorld* World) const;

		static std::optional<FColor> GetTerrainColor(TerrainNode::Type TerrainType);
		static std::optional<float> GetTerrainCostMultiplier(TerrainNode::Type TerrainType);
		
		static std::unordered_map<TerrainNode::Type, FColor> const TerrainColors;
		static std::unordered_map<TerrainNode::Type, float> const TerrainCostMultipliers;
	};
}
