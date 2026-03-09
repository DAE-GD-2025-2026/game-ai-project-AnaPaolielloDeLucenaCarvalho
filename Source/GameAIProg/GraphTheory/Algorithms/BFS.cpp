#include "BFS.h"

#include <map>
#include <queue>
#include <algorithm>

#include "Shared/Graph/Graph.h"

using namespace GameAI;

BFS::BFS(Graph* const pGraph)
	: pGraph(pGraph)
{
}

// Breath First Search Algorithm searches for a path from the startNode to the destinationNode
std::vector<Node*> BFS::FindPath(Node* const pStartNode, Node* const pDestinationNode) const
{
	std::vector<Node*> path;
	if (!pStartNode || !pDestinationNode || pStartNode == pDestinationNode) 
	{
		return path;
	}

	std::queue<Node*> frontier;
	std::map<Node*, Node*> cameFrom;

	frontier.push(pStartNode);
	cameFrom[pStartNode] = nullptr; // start node has no parent

	bool bFound = false;

	// BFS Loop
	while (!frontier.empty())
	{
		Node* current = frontier.front();
		frontier.pop();

		// exit if we reached the goal
		if (current == pDestinationNode)
		{
			bFound = true;
			break;
		}

		// neighbors
		auto connections = pGraph->FindConnectionsFrom(current->GetId());
		for (auto* conn : connections)
		{
			Node* nextNode = pGraph->GetNode(conn->GetToId()).get();

			if (cameFrom.find(nextNode) == cameFrom.end())
			{
				frontier.push(nextNode);
				cameFrom[nextNode] = current; // where we came from
			}
		}
	}

	// Backtracking
	if (bFound)
	{
		Node* current = pDestinationNode;
		while (current != nullptr)
		{
			path.push_back(current);
			current = cameFrom[current];
		}
        
		std::reverse(path.begin(), path.end());
	}

	return path;
}
