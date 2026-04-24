#pragma once
#include <vector>

#include "NavGraphPathfinding.h"
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Shared/Graph/Graph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"
#include "Shared/Utils/GeoUtilities.h"

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
		std::vector<NavLine> Portals = {};
		if (Path.size() < 2) return Portals;

		// 1. Add degenerate portal
		FVector2D startPos = Path.front()->GetPosition();
		Portals.push_back({startPos, startPos});

		// 2. Extract edge portals
		for (size_t i = 1; i < Path.size() - 1; ++i)
		{
			auto* pNavNode = static_cast<NavGraphNode*>(Path[i]);
			int edgeIdx = pNavNode->GetEdgeIdx();
			if (edgeIdx == -1) continue;

			const auto& edge = NavPoly.GetEdges()[edgeIdx];
			FVector2D v1 = FVector2D(edge.GetP1(NavPoly));
			FVector2D v2 = FVector2D(edge.GetP2(NavPoly));

			// 3. Ensure P1 is Right and P2 is Left
			FVector2D currentPos = Path[i]->GetPosition();
			FVector2D nextPos = Path[i+1]->GetPosition();
			FVector2D forward = nextPos - currentPos;

			if (FVector2D::CrossProduct(forward, v1 - currentPos) > 0.f)
			{
				Portals.push_back({v1, v2});
			}
			else
			{
				Portals.push_back({v2, v1});
			}
		}

			// 4. Add degenerate portal at END
			FVector2D endPos = Path.back()->GetPosition();
			Portals.push_back({endPos, endPos});

			return Portals;
		}

	static std::vector<FVector2D> OptimizePortals(std::vector<NavLine> const & Portals, TriPolygon const & NavPoly)
    {
        std::vector<FVector2D> Path{};
        if (Portals.empty()) return Path;

        FVector2D apexPos = Portals[0].P1;
        Path.push_back(apexPos);
        if (Portals.size() == 1) return Path;

        FVector2D rightLeg = Portals[1].P1 - apexPos;
        FVector2D leftLeg = Portals[1].P2 - apexPos;
        int rightLegIndex = 1;
        int leftLegIndex = 1;

        int amtPortals = static_cast<int>(Portals.size());

        for (int i = 2; i < amtPortals; ++i)
        {
            const auto& portal = Portals[i];

            // RIGHT CHECK
            FVector2D newRightLeg = portal.P1 - apexPos;            
            if (FVector2D::CrossProduct(rightLeg, newRightLeg) <= 0.f)
            {
                if (FVector2D::CrossProduct(leftLeg, newRightLeg) < 0.f)
                {
                    apexPos = Portals[leftLegIndex].P2;
                    Path.push_back(apexPos);

                    int nextIdx = leftLegIndex + 1;
                    if (nextIdx < amtPortals)
                    {
                        rightLeg = Portals[nextIdx].P1 - apexPos;
                        leftLeg = Portals[nextIdx].P2 - apexPos;
                        rightLegIndex = nextIdx;
                        leftLegIndex = nextIdx;
                    	i = nextIdx - 1;
                        continue;
                    }
                    else
                    {
                    	break;
                    }
                }
                else
                {
                    rightLeg = newRightLeg;
                    rightLegIndex = i;
                }
            }

            // LEFT CHECK
            FVector2D newLeftLeg = portal.P2 - apexPos;            
            if (FVector2D::CrossProduct(leftLeg, newLeftLeg) >= 0.f)
            {
                if (FVector2D::CrossProduct(rightLeg, newLeftLeg) > 0.f)
                {
                    apexPos = Portals[rightLegIndex].P1;
                    Path.push_back(apexPos);

                    int nextIdx = rightLegIndex + 1;
                    if (nextIdx < amtPortals)
                    {
                        rightLeg = Portals[nextIdx].P1 - apexPos;
                        leftLeg = Portals[nextIdx].P2 - apexPos;
                        rightLegIndex = nextIdx;
                        leftLegIndex = nextIdx;
                    	i = nextIdx - 1;
                        continue;
                    }
                    else
                    {
                    	break;
                    }
                }
                else
                {
                    leftLeg = newLeftLeg;
                    leftLegIndex = i;
                }
            }
        }

        Path.push_back(Portals.back().P1);
        return Path;
    }
private:
	SSFA() {};
	~SSFA() {};
};
}
