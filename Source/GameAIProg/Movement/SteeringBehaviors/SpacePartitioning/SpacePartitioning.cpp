#include "SpacePartitioning.h"

// --- Cell ---
// ------------
Cell::Cell(float Left, float Bottom, float Width, float Height)
{
	BoundingBox.Min = { Left, Bottom };
	BoundingBox.Max = { BoundingBox.Min.X + Width, BoundingBox.Min.Y + Height };
}

std::vector<FVector2D> Cell::GetRectPoints() const
{
	const float left = BoundingBox.Min.X;
	const float bottom = BoundingBox.Min.Y;
	const float width = BoundingBox.Max.X - BoundingBox.Min.X;
	const float height = BoundingBox.Max.Y - BoundingBox.Min.Y;

	std::vector<FVector2D> rectPoints =
	{
		{ left , bottom  },
		{ left , bottom + height  },
		{ left + width , bottom + height },
		{ left + width , bottom  },
	};

	return rectPoints;
}

// --- Partitioned Space ---
// -------------------------
CellSpace::CellSpace(UWorld* pWorld, float Width, float Height, int Rows, int Cols, int MaxEntities)
	: pWorld{pWorld}
	, SpaceWidth{Width}
	, SpaceHeight{Height}
	, NrOfRows{Rows}
	, NrOfCols{Cols}
	, NrOfNeighbors{0}
{
	Neighbors.SetNum(MaxEntities);
	
	//calculate bounds of a cell
	CellWidth = Width / Cols;
	CellHeight = Height / Rows;

	CellOrigin = FVector2D(-SpaceWidth / 2.0f, -SpaceHeight / 2.0f);
	
	// Create the cells
	for (int r = 0; r < NrOfRows; ++r)
	{
		for (int c = 0; c < NrOfCols; ++c)
		{
			float left = CellOrigin.X + (c * CellWidth);
			float bottom = CellOrigin.Y + (r * CellHeight);
			Cells.push_back(Cell(left, bottom, CellWidth, CellHeight));
		}
	}
}

void CellSpace::AddAgent(ASteeringAgent& Agent)
{
	// Add the agent to the correct cell
	int index = PositionToIndex(FVector2D(Agent.GetActorLocation()));
	Cells[index].Agents.push_back(&Agent);
}

void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	// Check if the agent needs to be moved to another cell - calculate the index for the old and new position
	int oldIndex = PositionToIndex(OldPos);
	int newIndex = PositionToIndex(FVector2D(Agent.GetActorLocation()));
	
	// If index changed, remove from old and add to new
	if (oldIndex != newIndex)
	{
		Cells[oldIndex].Agents.remove(&Agent);
		Cells[newIndex].Agents.push_back(&Agent);
	}
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	NrOfNeighbors = 0;
	FVector2D agentPos(Agent.GetActorLocation());
	
	// Box representing the query neighborhood
	FRect queryBox;
	queryBox.Min = FVector2D(agentPos.X - QueryRadius, agentPos.Y - QueryRadius);
	queryBox.Max = FVector2D(agentPos.X + QueryRadius, agentPos.Y + QueryRadius);
	
	// Loop over all cells - check if overlap with box
	for (Cell& cell : Cells)
	{
		if (DoRectsOverlap(queryBox, cell.BoundingBox))
		{
			for (ASteeringAgent* pOtherAgent : cell.Agents)
			{
				// Exclude ourself + ensure valid pointer
				if (pOtherAgent != nullptr && pOtherAgent != &Agent)
				{
					// Precise distance check
					float distanceSq = FVector2D::DistSquared(agentPos, FVector2D(pOtherAgent->GetActorLocation()));
					if (distanceSq < (QueryRadius * QueryRadius))
					{
						Neighbors[NrOfNeighbors] = pOtherAgent;
						NrOfNeighbors++;
					}
				}
			}
		}
	}
}

void CellSpace::EmptyCells()
{
	for (Cell& c : Cells)
		c.Agents.clear();
}

void CellSpace::RenderCells() const
{
	// Render the cells with the number of agents inside of it
	for (const Cell& cell : Cells)
	{
		// Bounding box
		FVector min(cell.BoundingBox.Min.X, cell.BoundingBox.Min.Y, 0.0f);
		FVector max(cell.BoundingBox.Max.X, cell.BoundingBox.Max.Y, 0.0f);
		
		FVector center = min + (max - min) * 0.5f;
		FVector extent = (max - min) * 0.5f;

		DrawDebugBox(pWorld, center, extent, FColor::Yellow, false, 0.0f, 0, 5.0f);

		// Number of agents in the cell
		if (cell.Agents.size() > 0)
		{
			FString agentCount = FString::FromInt(cell.Agents.size());
			
			FVector TextPos = center;
			TextPos.Z = 150.0f; 

			DrawDebugString(pWorld, TextPos, agentCount, nullptr, FColor::White, 0.0f, true);
		}
	}
}

int CellSpace::PositionToIndex(FVector2D const & Pos) const
{	
	// Calculate column and row
	int c = static_cast<int>((Pos.X - CellOrigin.X) / CellWidth);
	int r = static_cast<int>((Pos.Y - CellOrigin.Y) / CellHeight);

	// Clamp the indices - don't go out of bounds
	c = FMath::Clamp(c, 0, NrOfCols - 1);
	r = FMath::Clamp(r, 0, NrOfRows - 1);

	// 2D grid to 1D array
	return r * NrOfCols + c;
}

bool CellSpace::DoRectsOverlap(FRect const & RectA, FRect const & RectB)
{
	// Check if the rectangles are separated on either axis
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X) return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y) return false;
    
	// If they are not separated, they must overlap
	return true;
}