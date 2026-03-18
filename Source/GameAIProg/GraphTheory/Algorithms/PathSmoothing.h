#pragma once
#include <vector>

#include "NavGraphPathfinding.h"
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

	static std::vector<NavLine> FindPortals(std::vector<Node*> const& Path, TriPolygon const& NavPoly)
	{
		std::vector<NavLine> portals{};

		if (Path.size() < 2)
			return portals;

		// Add degenerate start portal
		portals.push_back(NavLine{ Path.front()->GetPosition(), Path.front()->GetPosition() });

		// For each node in the path that corresponds to a navmesh edge node,
		// retrieve its edge and orient it so:
		// P1 = right point, P2 = left point
		for (size_t i = 1; i + 1 < Path.size(); ++i)
		{
			auto* pNavNode = dynamic_cast<NavGraphNode*>(Path[i]);
			if (!pNavNode)
				continue;

			const int edgeIdx = pNavNode->GetEdgeIdx();
			if (edgeIdx < 0)
				continue;

			const auto& edge = NavPoly.GetEdges()[edgeIdx];

			FVector2D p1{ edge.GetP1(NavPoly).X, edge.GetP1(NavPoly).Y };
			FVector2D p2{ edge.GetP2(NavPoly).X, edge.GetP2(NavPoly).Y };

			// Determine orientation relative to path direction through this portal.
			// We want P1 = right, P2 = left.
			const FVector2D prev = Path[i - 1]->GetPosition();
			const FVector2D next = Path[i + 1]->GetPosition();
			const FVector2D dir = next - prev;

			const float cross1 = FVector2D::CrossProduct(dir, p1 - prev);
			const float cross2 = FVector2D::CrossProduct(dir, p2 - prev);

			// Point with smaller cross is more to the right
			if (cross1 > cross2)
			{
				std::swap(p1, p2);
			}

			portals.push_back(NavLine{ p1, p2 });
		}

		// Add degenerate end portal
		portals.push_back(NavLine{ Path.back()->GetPosition(), Path.back()->GetPosition() });

		return portals;
	}


	static std::vector<FVector2D> OptimizePortals(std::vector<NavLine> const& Portals, TriPolygon const& /*NavPoly*/)
	{
		std::vector<FVector2D> Path{};

		if (Portals.empty())
			return Path;

		FVector2D apexPoint = Portals[0].P1;
		int apexIndex = 0;

		FVector2D rightLeg{ 0.f, 0.f };
		FVector2D leftLeg{ 0.f, 0.f };
		int rightLegIndex = 1;
		int leftLegIndex = 1;

		Path.push_back(apexPoint);

		// Initialize legs from the first real portal if it exists
		if (Portals.size() > 1)
		{
			rightLeg = Portals[1].P1 - apexPoint;
			leftLeg = Portals[1].P2 - apexPoint;
		}

		for (int portalIdx = 2; portalIdx < static_cast<int>(Portals.size()); ++portalIdx)
		{
			const NavLine& portal = Portals[portalIdx];

			// --- RIGHT CHECK ---
			// Create a vector2 newRightLeg = from apexPoint to portal.P1
			FVector2D newRightLeg = portal.P1 - apexPoint;

			// Check if going inwards (CCW) or not
			// right check inward = CCW
			if (FVector2D::CrossProduct(rightLeg, newRightLeg) > 0.f)
			{
				// Check if we cross over the leftLeg
				if (FVector2D::CrossProduct(leftLeg, newRightLeg) >= 0.f)
				{
					// Not crossing over leftLeg -> update rightLeg
					rightLeg = newRightLeg;
					rightLegIndex = portalIdx;
				}
				else
				{
					// Crosses leftLeg:
					// Move apex by adding leftLeg
					apexPoint = apexPoint + leftLeg;
					apexIndex = leftLegIndex;

					// Push new apex point
					Path.push_back(apexPoint);

					// Set portalIdx to leftLegIndex + 1
					portalIdx = leftLegIndex + 1;

					// Reset leg indices to that new portalIdx
					leftLegIndex = portalIdx;
					rightLegIndex = portalIdx;

					// Recalculate legs if valid
					if (portalIdx < static_cast<int>(Portals.size()))
					{
						rightLeg = Portals[rightLegIndex].P1 - apexPoint;
						leftLeg = Portals[leftLegIndex].P2 - apexPoint;
					}

					continue;
				}
			}

			// --- LEFT CHECK ---
			// Create a vector2 newLeftLeg
			FVector2D newLeftLeg = portal.P2 - apexPoint;

			// Same logic as right, but inward is CW instead of CCW
			if (FVector2D::CrossProduct(leftLeg, newLeftLeg) < 0.f)
			{
				// Check if we cross over the rightLeg
				if (FVector2D::CrossProduct(rightLeg, newLeftLeg) <= 0.f)
				{
					// Not crossing over rightLeg -> update leftLeg
					leftLeg = newLeftLeg;
					leftLegIndex = portalIdx;
				}
				else
				{
					// Crosses rightLeg:
					// Move apex by adding rightLeg
					apexPoint = apexPoint + rightLeg;
					apexIndex = rightLegIndex;

					// Push new apex point
					Path.push_back(apexPoint);

					// Set portalIdx to rightLegIndex + 1
					portalIdx = rightLegIndex + 1;

					// Reset leg indices to that new portalIdx
					leftLegIndex = portalIdx;
					rightLegIndex = portalIdx;

					// Recalculate legs if valid
					if (portalIdx < static_cast<int>(Portals.size()))
					{
						rightLeg = Portals[rightLegIndex].P1 - apexPoint;
						leftLeg = Portals[leftLegIndex].P2 - apexPoint;
					}

					continue;
				}
			}
		}

		// Push the last point to the path
		Path.push_back(Portals.back().P1);

		return Path;
	}
private:
	SSFA() {};
	~SSFA() {};
};
}
