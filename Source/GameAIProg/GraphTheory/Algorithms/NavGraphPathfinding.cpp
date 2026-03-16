#include "NavGraphPathfinding.h"

#include "AStar.h"
#include "PathSmoothing.h"
#include "VectorTypes.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

using namespace GameAI;

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos,
	NavGraph* const pNavGraph, std::vector<FVector2D>& debugNodePositions, std::vector<NavLine>& debugPortals) 
{	
	if (!pNavGraph)
	{
		return {};
	}
	
	const auto* pNavPoly = pNavGraph->GetNavPolygon();
	
	// A. Get the startTriangle and endTriangle
	const auto* pStartTri = pNavPoly->GetTriangleAtPosition(startPos, true);
	const auto* pEndTri = pNavPoly->GetTriangleAtPosition(endPos, true);
	
	// Check if they exist
	if (!pStartTri || !pEndTri)
	{
		return {};
	}

	// Check if they are the same
	if (pStartTri == pEndTri)
	{
		return { startPos, endPos };
	}
	
	// B. Clone the graph
	auto pClonedGraph = pNavGraph->Clone();
	
	// C. Create the start NavGraphNode and add to the cloned graph
    int startNodeId = pClonedGraph->AddNode(std::make_unique<NavGraphNode>(startPos, -1));

    for (const auto& edge : pStartTri->GetEdges())
    {
        int edgeIdx = pNavPoly->FindEdgeIndex(edge).value_or(-1);
        int nodeId = pNavGraph->GetNodeIdFromEdgeIndex(edgeIdx);

        if (nodeId != -1)
        {
        	float dist = FVector2D::Distance(startPos, pClonedGraph->GetNode(nodeId)->GetPosition());
            pClonedGraph->AddConnection(startNodeId, nodeId);
            pClonedGraph->FindConnection(startNodeId, nodeId)->SetWeight(dist);
        }
    }

    // D. Create the end NavGraphNode and add to the cloned graph
    int endNodeId = pClonedGraph->AddNode(std::make_unique<NavGraphNode>(endPos, -1));

    for (const auto& edge : pEndTri->GetEdges())
    {
        int edgeIdx = pNavPoly->FindEdgeIndex(edge).value_or(-1);
        int nodeId = pNavGraph->GetNodeIdFromEdgeIndex(edgeIdx);

        if (nodeId != -1)
        {
        	float dist = FVector2D::Distance(endPos, pClonedGraph->GetNode(nodeId)->GetPosition());
            pClonedGraph->AddConnection(nodeId, endNodeId);
            pClonedGraph->FindConnection(nodeId, endNodeId)->SetWeight(dist);
        }
    }

    // E. Run A*
	AStar aStar(pClonedGraph.get(), HeuristicFunctions::Euclidean);
	auto pathNodes = aStar.FindPath( pClonedGraph->GetNode(startNodeId).get(), pClonedGraph->GetNode(endNodeId).get() );

    std::vector<FVector2D> finalPath{};
    for (auto* pNode : pathNodes)
    {
        FVector2D pos = pNode->GetPosition();
        finalPath.push_back(pos);
        debugNodePositions.push_back(pos);
    }
	
	// Extra: optimiser
	if (!pathNodes.empty())
	{
		debugPortals = SSFA::FindPortals(pathNodes, *pNavPoly);
		finalPath = SSFA::OptimizePortals(debugPortals, *pNavPoly);
    
		debugNodePositions = finalPath; 
	}

    return finalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};

	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals);
}