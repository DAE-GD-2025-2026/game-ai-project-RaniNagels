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

	CellOrigin = FVector2D{-Width * 0.5f, -Height * 0.5f };

	// create the cells
	Cells.reserve(Rows * Cols);
	for (int row{}; row < Rows; ++row)
	{
		for (int col{}; col < Cols; ++col)
		{
			Cells.emplace_back(col * CellWidth - SpaceWidth*0.5f, row * CellHeight - SpaceHeight*0.5f, CellWidth, CellHeight);
		}
	}
}

void CellSpace::AddAgent(ASteeringAgent& Agent)
{
	// Add the agent to the correct cell
	const int index = PositionToIndex(Agent.GetPosition());
	if (index < 0 || index >= Cells.size())
		return; // agent might be invalid or out of bounds
	Cells[index].Agents.push_back(&Agent);
}

void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	// Check if the agent needs to be moved to another cell.
	// Use the calculated index for oldPos and currentPos for this

	const int oldIndex = PositionToIndex(OldPos);
	auto newPos = Agent.GetPosition();
	const int newIndex = PositionToIndex(newPos);

	if (oldIndex != newIndex) 
	{
		if (newIndex >= 0 && newIndex < Cells.size() &&
			oldIndex >= 0 && oldIndex < Cells.size())
		{
			for (auto a : Cells[oldIndex].Agents)
			{
				if (a == &Agent)
				{
					Cells[oldIndex].Agents.remove(a);
					break;
				}
			}
			//Cells[oldIndex].Agents.remove(&Agent);
			Cells[newIndex].Agents.push_back(&Agent);
			UE_LOG(LogTemp, Warning, TEXT("Agent has changed cell: %i -> %i"), oldIndex, newIndex);
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("oldIndex or newIndex is invalid, could not change cell!"));
			UE_LOG(LogTemp, Error, TEXT("oldIndex: %i, newIndex:%i"), oldIndex, newIndex);
		}
	}
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	// Register the neighbors for the provided agent
	// Only check the cells that are within the radius of the neighborhood

	NrOfNeighbors = 0;
	for (Cell& c : Cells)
	{
		FRect queryRect = {
			{ Agent.GetPosition().X - QueryRadius, Agent.GetPosition().Y - QueryRadius },
			{ Agent.GetPosition().X + QueryRadius, Agent.GetPosition().Y + QueryRadius } };

		if (DoRectsOverlap(c.BoundingBox, queryRect))
		{
			for (ASteeringAgent* agent : c.Agents)
			{
				if (agent != nullptr && agent != &Agent)
				{
					float distance = FVector2D::Distance(Agent.GetPosition(), agent->GetPosition());
					if (distance < QueryRadius)
					{
						Neighbors[NrOfNeighbors] = agent;
						++NrOfNeighbors;
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
	for (const Cell& c : Cells)
	{
		auto center = (c.BoundingBox.Min + c.BoundingBox.Max) / 2;
		DrawDebugBox(pWorld, FVector(center, 0), FVector(CellWidth * 0.5f, CellHeight * 0.5f, 0), FColor::Emerald);

		//FString text = FString::Printf(TEXT("%d"), c.Agents.size());
		//DrawDebugString(pWorld, FVector(c.BoundingBox.Max.X, c.BoundingBox.Min.Y, 0), FString(text));
	}
}

int CellSpace::PositionToIndex(FVector2D const & Pos) const
{
	FVector2D relativePos{ Pos - CellOrigin };
	int col = relativePos.X / CellWidth;
	int row = relativePos.Y / CellHeight;
	int index = row * NrOfCols + col;

	//if (index < 0 || index >= Cells.size())
	//	return -1; // Index is out of bounds
	return index;
}

bool CellSpace::DoRectsOverlap(FRect const & RectA, FRect const & RectB)
{
	// Check if the rectangles are separated on either axis
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X) return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y) return false;
    
	// If they are not separated, they must overlap
	return true;
}