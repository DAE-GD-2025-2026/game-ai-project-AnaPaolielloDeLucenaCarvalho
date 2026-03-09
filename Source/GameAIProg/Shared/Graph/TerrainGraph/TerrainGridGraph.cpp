#include "TerrainGridGraph.h"

using namespace GameAI;

std::unordered_map<TerrainNode::Type, FColor> const TerrainGridGraph::TerrainColors
{
	{TerrainNode::Type::Mud, FColor{160,82,45}},
	{TerrainNode::Type::Water, FColor{0,0,255, 120}}
};
std::unordered_map<TerrainNode::Type, float> const TerrainGridGraph::TerrainCostMultipliers
{
	{TerrainNode::Type::Mud, 2.0f},
	{TerrainNode::Type::Water, 0},
	{TerrainNode::Type::Clear, 1.0f},
};

TerrainGridGraph::TerrainGridGraph(TerrainNodeFactory* Factory, int Rows, int Cols, float CellSize, float Cost,
	FVector2D const& OriginPosition, bool IsDiagonallyConnected, bool IsDirectional)
		: GridGraph(Factory, Rows, Cols, CellSize, Cost, OriginPosition, IsDiagonallyConnected, IsDirectional)
{
}

void TerrainGridGraph::PaintNodeAtPosition(FVector2D const& Position, TerrainNode::Type TypeToPaint)
{
	int NodeId = GetNodeIdAtPosition(Position);
	TerrainNode* AsTerrainNode = GetNodeAs<TerrainNode>(NodeId);
	
	TerrainNode::Type const OldType = AsTerrainNode->GetType();
	
	// Stop if we're just repainting the same type
	if (OldType == TypeToPaint) return;

	// Paint
	AsTerrainNode->SetType(TypeToPaint);
	
	if (OldType == TerrainNode::Type::Water)
	{
		// reconnect
		AddConnectionsToAdjacentCells(NodeId);
	}
	
	if (TypeToPaint == TerrainNode::Type::Water)
	{
		// remove connection to node
		RemoveConnectionsTo(NodeId);
		RemoveConnectionsFrom(NodeId);
		return;
	}
		
	// Apply the new terrain cost multiplier
	float TerrainCostMultiplier = *GetTerrainCostMultiplier(TypeToPaint);
	auto ConnectionsToNode = FindConnectionsWith(NodeId);
	for (Connection* ConnectionTo : ConnectionsToNode)
	{
		if (IsCardinalConnection(ConnectionTo->GetFromId(), ConnectionTo->GetToId()))
		{
			ConnectionTo->SetWeight(GetCardinalCost() * TerrainCostMultiplier);
		}
		else
		{
			ConnectionTo->SetWeight(GetDiagonalCost() * TerrainCostMultiplier);
		}
	}
}

void TerrainGridGraph::DrawTerrain(UWorld* World) const
{
	FVector CellExtents{CellSize/2, CellSize/2, 1.0f};
	FVector GirdOrigin3D{GridOrigin, 0.0f};
	for (int Row = 0; Row < NrRows; ++Row)
	{
		for (int Col = 0; Col < NrColumns; ++Col)
		{
			FVector const NodePos{CellSize * Col + CellExtents.X,CellSize * Row + CellExtents.Y,1.0f};
			
			switch (auto CurrentNodeType = GetNodeAs<TerrainNode>(GetNodeId(Col, Row))->GetType()) {
			case TerrainNode::Type::Mud:
				DrawDebugSolidBox(World, GirdOrigin3D + NodePos, CellExtents, *GetTerrainColor(CurrentNodeType));
				break;
			case TerrainNode::Type::Water:
				DrawDebugSolidBox(World, GirdOrigin3D + NodePos, CellExtents, *GetTerrainColor(CurrentNodeType));
				break;
			default: 
				// Do nothing! :)
				break;
			}
		}
	}
}

std::optional<FColor> TerrainGridGraph::GetTerrainColor(TerrainNode::Type TerrainType)
{
	if (auto const FindItr = TerrainColors.find(TerrainType); FindItr != TerrainColors.end())
	{
		return FindItr->second;
	}
	
	return std::nullopt;
}

std::optional<float> TerrainGridGraph::GetTerrainCostMultiplier(TerrainNode::Type TerrainType)
{
	if (auto const FindItr = TerrainCostMultipliers.find(TerrainType); FindItr != TerrainCostMultipliers.end())
	{
		return FindItr->second;
	}
	
	return std::nullopt;
}
