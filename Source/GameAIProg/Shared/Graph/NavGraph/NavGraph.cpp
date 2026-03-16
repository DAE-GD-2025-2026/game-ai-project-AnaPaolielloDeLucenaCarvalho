#include "NavGraph.h"

#include "NavGraphNode.h"

GameAI::NavGraph::NavGraph(std::unique_ptr<TriPolygon> && NavPoly)
	: Graph{false}
	, pNavPoly{std::move(NavPoly)}
{
	CreateNavigationGraph();
}

GameAI::NavGraph::NavGraph(const NavGraph& Other)
	: Graph(false)
{
	Nodes.reserve(Other.Nodes.size());
	for (std::unique_ptr<Node> const & OtherNode : Other.Nodes)
	{
		Nodes.push_back(std::make_unique<NavGraphNode>(*dynamic_cast<NavGraphNode*>(OtherNode.get())));
	}
        
	Connections.reserve(Other.Connections.size());
	for (std::unique_ptr<Connection> const & OtherConnection : Other.Connections)
	{
		Connections.push_back(std::make_unique<Connection>(*OtherConnection.get()));
	}
}

std::unique_ptr<GameAI::NavGraph> GameAI::NavGraph::Clone() const
{
	return std::make_unique<NavGraph>(*this);
}

int GameAI::NavGraph::GetNodeIdFromEdgeIndex(int EdgeIdx) const
{
	if (EdgeIdx >= 0)
	{
		for (auto const & pNode : Nodes)
		{
			if (reinterpret_cast<NavGraphNode*>(pNode.get())->GetEdgeIdx() == EdgeIdx)
			{
				return pNode->GetId();
			}
		}
	}
	
	return Graphs::InvalidNodeId;
}

void GameAI::NavGraph::CreateNavigationGraph()
{
	if (!pNavPoly)
	{
		return;
	}
	
	// A. Loop over all the lines (edges) of the Polygon
	const auto& allEdges = pNavPoly->GetEdges();
	const auto& allTriangles = pNavPoly->GetTriangles();

	for (int i = 0; i < (int)allEdges.size(); ++i)
	{
		int sharedCount = 0;
		for (const auto& tri : allTriangles)
		{
			if (tri.HasEdge(allEdges[i]))
			{
				sharedCount++;
			}
		}
    
		if (sharedCount > 1)
		{
			FVector p1 = allEdges[i].GetP1(*pNavPoly);
			FVector p2 = allEdges[i].GetP2(*pNavPoly);
			FVector2D middle = { (p1.X + p2.X) * 0.5f, (p1.Y + p2.Y) * 0.5f };

			AddNode(std::make_unique<NavGraphNode>(middle, i));
		}
	}

    // B. For each Triangle in the Navigation Mesh, find the Nodes and connect them
	const auto& triangles = pNavPoly->GetTriangles();
	for (const auto& tri : triangles)
	{
		std::vector<int> nodeIds;
		auto edges = tri.GetEdges();

		for (const auto& edge : edges)
		{
			int edgeIdx = pNavPoly->FindEdgeIndex(edge).value_or(-1);
			int nodeId = GetNodeIdFromEdgeIndex(edgeIdx);

			if (nodeId != -1) 
			{
				nodeIds.push_back(nodeId);
			}
		}

		// 2 nodes - 1 connection
		if (nodeIds.size() == 2)
		{
			AddConnection(nodeIds[0], nodeIds[1]);
			AddConnection(nodeIds[1], nodeIds[0]);
		}
		// 3 nodes - 3 connections
		else if (nodeIds.size() == 3)
		{
			AddConnection(nodeIds[0], nodeIds[1]);
			AddConnection(nodeIds[1], nodeIds[0]);
    
			AddConnection(nodeIds[1], nodeIds[2]);
			AddConnection(nodeIds[2], nodeIds[1]);
    
			AddConnection(nodeIds[2], nodeIds[0]);
			AddConnection(nodeIds[0], nodeIds[2]);
		}
	}

    // C. Set the connection costs to the distance
	SetConnectionCostsToDistances();
}
