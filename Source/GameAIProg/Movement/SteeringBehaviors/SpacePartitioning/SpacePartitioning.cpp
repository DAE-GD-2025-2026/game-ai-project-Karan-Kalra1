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

	// Create all cells
	Cells.reserve(NrOfRows * NrOfCols);

	float startX = -SpaceWidth * 0.5f;
	float startY = -SpaceHeight * 0.5f;

	for (int row = 0; row < NrOfRows; ++row)
	{
		for (int col = 0; col < NrOfCols; ++col)
		{
			float left = startX + col * CellWidth;
			float bottom = startY + row * CellHeight;

			Cells.emplace_back(left, bottom, CellWidth, CellHeight);
		}
	}
}

void CellSpace::AddAgent(ASteeringAgent& Agent)
{
	
		int index = PositionToIndex(Agent.GetPosition());
		Cells[index].Agents.push_back(&Agent);
	
}

void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	
		int oldIndex = PositionToIndex(OldPos);
		int newIndex = PositionToIndex(Agent.GetPosition());

		if (oldIndex != newIndex)
		{
			Cells[oldIndex].Agents.remove(&Agent);
			Cells[newIndex].Agents.push_back(&Agent);
		}
	
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	NrOfNeighbors = 0;

	FVector2D pos = Agent.GetPosition();
	FRect queryRect;
	queryRect.Min = pos - FVector2D(QueryRadius, QueryRadius);
	queryRect.Max = pos + FVector2D(QueryRadius, QueryRadius);

	for (Cell& cell : Cells)
	{
		if (!DoRectsOverlap(cell.BoundingBox, queryRect))
			continue;

		for (ASteeringAgent* other : cell.Agents)
		{
			if (&Agent == other)
				continue;

			float distSq =
				(Agent.GetPosition() - other->GetPosition()).SizeSquared();

			if (distSq < QueryRadius * QueryRadius)
			{
				Neighbors[NrOfNeighbors++] = other;
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
	for (const Cell& cell : Cells)
	{
		auto points = cell.GetRectPoints();

		for (int i = 0; i < 4; ++i)
		{
			DrawDebugLine(
				pWorld,
				FVector(points[i], 0),
				FVector(points[(i + 1) % 4], 0),
				FColor::Green,
				false,
				-1.f,
				0,
				1.f
			);
		}
	}
}

int CellSpace::PositionToIndex(FVector2D const & Pos) const
{
	
	int col = (Pos.X + SpaceWidth * 0.5f) / CellWidth;
	int row = (Pos.Y + SpaceHeight * 0.5f) / CellHeight;

	col = FMath::Clamp(col, 0, NrOfCols - 1);
	row = FMath::Clamp(row, 0, NrOfRows - 1);

	return row * NrOfCols + col;
	
}

bool CellSpace::DoRectsOverlap(FRect const & RectA, FRect const & RectB)
{
	// Check if the rectangles are separated on either axis
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X) return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y) return false;
    
	// If they are not separated, they must overlap
	return true;
}