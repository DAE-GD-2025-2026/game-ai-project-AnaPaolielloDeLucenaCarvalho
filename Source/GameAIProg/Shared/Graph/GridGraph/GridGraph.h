#pragma once
#include "Shared/Graph/Graph.h"
#include <memory>
#include <unordered_map>

#include "Shared/Graph/GraphNodeFactory.h"


namespace GameAI
{
	class GridGraph : public Graph
	{
	public:
		enum class Direction
		{
			North,
			NorthEast,
			East,
			SouthEast,
			South,
			SouthWest,
			West,
			NorthWest,
			LAST // Keep as last value!
		};
		
		GridGraph(IGraphNodeFactory* Factory, int Rows, int Cols, float CellSize, float Cost = 1.0f, FVector2D const & OriginPosition = {0,0},
		          bool IsDiagonallyConnected = true, bool IsDirectional = false);
		virtual ~GridGraph() = default;
		
		int GetRows() const { return NrRows; }
		int GetColumns() const { return NrColumns; }
		float GetCellSize() const { return CellSize; }
		
		int GetNodeId(int Col, int Row) const { return Row * NrColumns + Col; }
		int GetNodeIdAtPosition(FVector2D const& Position) const;
		bool IsWithinBounds(int Col, int Row) const;
		FVector2D GetNodePosition(int Index) const;
		FIntVector2 GetColAndRow(int Index) const;
		
		std::unique_ptr<Node> const & GetNode(int Row, int Col) const;
		std::unique_ptr<Node>& GetNode(int Row, int Col);
		
		std::unique_ptr<Node> const & GetNodeAtPosition(FVector2D const & Position) const;
		std::unique_ptr<Node> & GetNodeAtPosition(FVector2D const & Position);
		
		void AddConnectionsToAdjacentCells(int NodeId);
		
		void DebugDrawCells(UWorld const * const World) const;
		
		float GetCardinalCost() const { return CostStraight; }
		float GetDiagonalCost() const { return CostDiagonal; }

		static bool IsCardinal(Direction Direction);
		bool IsCardinalConnection(int FromId, int ToId);
		
	protected:
		static std::unordered_map<Direction, FIntVector2> DirectionDeltas;
		FVector2D GridOrigin; // bottom left
		
		int NrRows;
		int NrColumns;
		float CellSize;
		
		float CostStraight;
		float CostDiagonal;
		
		bool bIsDiagonallyConnected;
	};
}
