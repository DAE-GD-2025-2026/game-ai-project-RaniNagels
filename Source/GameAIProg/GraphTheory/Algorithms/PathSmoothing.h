#pragma once
#include <vector>

#include "NavGraphPathfinding.h"
#include "VectorTypes.h"
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Shared/Graph/Graph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

namespace GameAI
{
	class SSFA final
{
public:
	//=== SSFA Functions ===
	//--- References ---
	//http://digestingduck.blogspot.be/2010/03/simple-stupid-funnel-algorithm.html
	//https://gamedev.stackexchange.com/questions/68302/how-does-the-simple-stupid-funnel-algorithm-work
	static std::vector<NavLine> FindPortals(std::vector<Node*> const & Path, TriPolygon const & NavPoly)
	{
		//Container
		std::vector<NavLine> Portals = {};

		// Dummy start point
		Portals.push_back(NavLine{ Path[0]->GetPosition(), Path[0]->GetPosition() });
		
		//For each node received, get it's corresponding line
		for (size_t i = 1; i < Path.size() - 1; ++i)
		{
			auto navNode = reinterpret_cast<NavGraphNode*>(Path[i]);
			if (navNode->GetEdgeIdx() == -1 || navNode->GetEdgeIdx() >= NavPoly.GetEdges().size()) continue;
			
			auto line = NavPoly.GetEdges()[navNode->GetEdgeIdx()];
			
			auto centerLine = (line.GetP1(NavPoly) + line.GetP2(NavPoly)) / 2.0f;
			FVector previousPosition = FVector{ i == 0 ? Path[0]->GetPosition() : Path[i - 1]->GetPosition(), 0.f };

			FVector lineVec = centerLine - previousPosition;
			FVector pathVec = line.GetP1(NavPoly) - previousPosition;
			
			//Redetermine it's "orientation" based on the required path (left-right vs right-left) - p1 should be right point
			FVector2D p1{}, p2{}; // right, left
			float result = UE::Geometry::Cross(lineVec, pathVec).Z;
			if (result < 0)
			{
				// Clock wise
				p1 = FVector2D{line.GetP1(NavPoly)};
				p2 = FVector2D{line.GetP2(NavPoly)};
			}
			else if (result > 0)
			{
				// Counter Clock wise
				p1 = FVector2D{line.GetP2(NavPoly)};
				p2 = FVector2D{line.GetP1(NavPoly)};
			}
			else continue;
			
			//Store portal
			Portals.push_back(NavLine{ p1, p2 });
		}

		//Add degenerate portal to force end evaluation
		Portals.push_back(NavLine{ Path[Path.size() - 1]->GetPosition(), Path[Path.size() - 1]->GetPosition() });

		return Portals;
	}

	static std::vector<FVector2D> OptimizePortals( std::vector<NavLine> const & Portals, TriPolygon const & NavPoly)
	{
		std::vector<FVector2D> Path{};

		int apexIdx{ 0 }, leftLegIdx{ 1 }, rightLegIdx{ 1 };

		FVector2D apexPoint{ Portals[apexIdx].P1 };

		//P1 == right point of portal, P2 == left point of portal
		FVector2D rightLeg{ Portals[rightLegIdx].P1 - apexPoint };
		FVector2D leftLeg{ Portals[leftLegIdx].P2 - apexPoint };

		// add apexPoint to the path.
		Path.push_back(apexPoint);
		
		for (size_t portalIdx{1}; portalIdx < Portals.size(); ++portalIdx)
		{
			const auto& portal = Portals[portalIdx];

			//--- RIGHT CHECK ---
			//1. See if moving funnel inwards - RIGHT
			FVector2D newRightLeg = portal.P1 - apexPoint;
			if (UE::Geometry::Cross(FVector{ rightLeg, 0 }, FVector{ newRightLeg, 0 }).Z >= 0)
			{
				//2. See if new line degenerates a line segment - RIGHT
				if (UE::Geometry::Cross(FVector{ newRightLeg, 0 }, FVector{ leftLeg, 0 }).Z > 0)
				{
					// the newRightLeg is not crossing over the left leg
					rightLeg = newRightLeg;
					rightLegIdx = portalIdx;
				}
				else
				{
					//Leftleg becomes new apex point
					apexPoint += leftLeg;

					apexIdx = leftLegIdx;
					portalIdx = leftLegIdx + 1;
					leftLegIdx = portalIdx;
					rightLegIdx = portalIdx;

					Path.push_back(apexPoint);

					//Calculate new legs (if not the end)
					if (portalIdx < Portals.size())
					{
						rightLeg = Portals[rightLegIdx].P1 - apexPoint;
						leftLeg = Portals[leftLegIdx].P2 - apexPoint;
						continue;
					}
				}
			}


			//--- LEFT CHECK ---
			//1. See if moving funnel inwards - LEFT
			FVector2D newLeftLeg = portal.P2 - apexPoint;
			if (UE::Geometry::Cross(FVector{ leftLeg, 0 }, FVector{ newLeftLeg, 0 }).Z <= 0)
			{
				//2. See if new line degenerates a line segment - LEFT
				if (UE::Geometry::Cross(FVector{ newLeftLeg, 0 }, FVector{ rightLeg, 0 }).Z < 0)
				{
					// the newLeftLeg is not crossing over the left leg
					leftLeg = newLeftLeg;
					leftLegIdx = portalIdx;
				}
				else
				{
					// the newLeftLeg is crossing over the left leg
					//Rightleg becomes new apex point
					apexPoint += rightLeg;

					apexIdx = rightLegIdx;
					portalIdx = rightLegIdx + 1;
					leftLegIdx = portalIdx;
					rightLegIdx = portalIdx;

					Path.push_back(apexPoint);

					//Calculate new legs (if not the end)
					if (portalIdx < Portals.size())
					{
						rightLeg = Portals[rightLegIdx].P1 - apexPoint;
						leftLeg = Portals[leftLegIdx].P2 - apexPoint;
						continue;
					}
				}
			}
		}

		// Add last path point
		Path.push_back(Portals.back().P1);

		return Path;
	}
private:
	SSFA() {};
	~SSFA() {};
};
}
