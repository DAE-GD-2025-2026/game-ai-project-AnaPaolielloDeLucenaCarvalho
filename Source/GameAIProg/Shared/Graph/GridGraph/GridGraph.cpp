#include "GridGraph.h"

using namespace GameAI;

std::unordered_map<GridGraph::Direction, FIntVector2> GridGraph::DirectionDeltas
{
	{Direction::North, {1, 0}},
	{Direction::NorthEast, {1, 1}},
	{Direction::East, {0, 1}},
	{Direction::SouthEast, {-1, 1}},
	{Direction::South, {-1, 0}},
	{Direction::SouthWest, {-1, -1}},
	{Direction::West, {0, -1}},
	{Direction::NorthWest, {1, -1}},
};

GridGraph::GridGraph(IGraphNodeFactory* Factory, int Rows, int Cols, float CellSize, float Cost, FVector2D const& OriginPosition,
	bool IsDiagonallyConnected, bool IsDirectional)
	: Graph(IsDirectional)
	, GridOrigin{OriginPosition}
	, NrRows{Rows}
	, NrColumns{Cols}
	, CellSize{CellSize}
	, CostStraight{Cost}
	, CostDiagonal{FMath::Sqrt(Cost * Cost + Cost * Cost)}
	, bIsDiagonallyConnected{IsDiagonallyConnected}
{
	// Make empty grid
	Nodes.resize(Rows * Cols);
	
	// Create all nodes
	for (int Row = 0; Row < NrRows; ++Row)
	{
		for (int Col = 0; Col < NrColumns; ++Col)
		{
			const int Index = GetNodeId(Col, Row);
			auto NodePtr = Factory->CreateNode(GetNodePosition(Index));
			NodePtr->SetId(Index);
			Nodes[Index] = std::move(NodePtr);
		}
	}

	// Create connections in each valid direction on each node
	for (int Row = 0; Row < NrRows; ++Row)
	{
		for (int Col = 0; Col < NrColumns; ++Col)
		{
			AddConnectionsToAdjacentCells(GetNodeId(Col, Row));
		}
	}
}

int GridGraph::GetNodeIdAtPosition(FVector2D const& Position) const
{
	FVector2D const OriginToPosition = Position - GridOrigin;
	
	if (OriginToPosition.X < 0 || OriginToPosition.Y < 0)
	{
		return Graphs::InvalidNodeId;	
	}
	
	int const Col = OriginToPosition.X / CellSize;
	int const Row = OriginToPosition.Y / CellSize;

	if (!IsWithinBounds(Col, Row))
	{
		return Graphs::InvalidNodeId;
	}
	return GetNodeId(Col, Row);
}

bool GridGraph::IsWithinBounds(int Col, int Row) const
{
	return Row >= 0 && Row < NrRows && Col >= 0 && Col < NrColumns;
}

FVector2D GridGraph::GetNodePosition(int Index) const
{
	auto Position = GetColAndRow(Index);
	return GridOrigin + FVector2D{Position.X * CellSize + CellSize/2, Position.Y * CellSize + CellSize/2};
}

FIntVector2 GridGraph::GetColAndRow(int Index) const
{
	return { Index % NrColumns, Index / NrRows }; // Col, Row
}

std::unique_ptr<Node> const& GridGraph::GetNode(int Row, int Col) const
{
	return Graph::GetNode(GetNodeId(Col, Row));
}

std::unique_ptr<Node>& GridGraph::GetNode(int Row, int Col)
{
	return Graph::GetNode(GetNodeId(Col, Row));
}

std::unique_ptr<Node> const& GridGraph::GetNodeAtPosition(FVector2D const& Position) const
{
	return Graph::GetNode(GetNodeIdAtPosition(Position));
}

std::unique_ptr<Node>& GridGraph::GetNodeAtPosition(FVector2D const& Position)
{
	return Graph::GetNode(GetNodeIdAtPosition(Position));
}

void GridGraph::AddConnectionsToAdjacentCells(int NodeId)
{
	// Get our col & row
	FIntVector2 const NodeColRow = GetColAndRow(NodeId);
	
	// Decide if we skip non-cardinal directions
	int const DirectionIncrement = !bIsDiagonallyConnected ? 2 : 1;
	for (int DirectionAsInt{static_cast<int>(Direction::North)}; 
		DirectionAsInt < static_cast<int>(Direction::LAST); DirectionAsInt += DirectionIncrement)
	{
		FIntVector2 const PosDelta = DirectionDeltas[static_cast<Direction>(DirectionAsInt)];
		FIntVector2 const PosAtDelta = NodeColRow + PosDelta;

		if (IsWithinBounds(PosAtDelta.X, PosAtDelta.Y))
		{
			auto NewConnection{std::make_unique<Connection>(NodeId, GetNodeId(PosAtDelta.X, PosAtDelta.Y))};
			if (IsCardinal(static_cast<Direction>(DirectionAsInt)))
			{
				NewConnection->SetWeight(GetCardinalCost());
			}
			else
			{
				NewConnection->SetWeight(GetDiagonalCost());
			}
			
			// This will cause warnings with already existing connections, that is fine :)
			AddConnection(std::move(NewConnection));
		}
	}
}

void GridGraph::DebugDrawCells(UWorld const * const World) const
{
	FVector CellExtents{CellSize/2, CellSize/2, 1.0f};
	FVector GirdOrigin3D{GridOrigin, 0.0f};
	for (int Row = 0; Row < NrRows; ++Row)
	{
		for (int Col = 0; Col < NrColumns; ++Col)
		{
			FVector const NodePos{CellSize * Col + CellExtents.X,CellSize * Row + CellExtents.Y,1.0f};
			DrawDebugBox(World, GirdOrigin3D + NodePos, CellExtents, FColor::Red);
		}
	}
}

bool GridGraph::IsCardinal(Direction Direction)
{
	// works due to ordering of the enum
	return Direction != Direction::LAST && static_cast<int>(Direction) % 2 == 0;
}

bool GridGraph::IsCardinalConnection(int FromId, int ToId)
{
	FIntVector2 Delta{GetColAndRow(ToId) - GetColAndRow(FromId)};
	auto const FindItr = std::ranges::find_if(DirectionDeltas, 
		[&](auto const& KeyValue){return KeyValue.second == Delta;});
	if (FindItr != DirectionDeltas.end())
	{
		return IsCardinal(FindItr->first);
	}
	return false;
}
