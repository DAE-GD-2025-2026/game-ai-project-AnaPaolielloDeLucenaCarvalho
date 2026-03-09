#include "AStar.h"

#include <algorithm>

using namespace GameAI;

AStar::AStar(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction)
	: pGraph(pGraph)
	, HeuristicFunction(hFunction)
{
}

std::vector<Node*> AStar::FindPath(Node* const pStartNode, Node* const pDestinationNode)
{
	std::vector<Node*> path{};
	
	if (!pStartNode || !pDestinationNode || pStartNode == pDestinationNode) 
	{
		return path;
	}

	std::vector<NodeRecord> openList;
	std::vector<NodeRecord> closedList;
	NodeRecord currentNodeRecord;

	NodeRecord startRecord;
	startRecord.pNode = pStartNode;
	startRecord.pConnection = nullptr;
	startRecord.costSoFar = 0.f;
	startRecord.estimatedTotalCost = GetHeuristicCost(pStartNode, pDestinationNode);
	
	openList.push_back(startRecord);

	// While Loop (part 1)
	while (!openList.empty())
	{		
		// A. Get record from the open list with lowest F-score
		auto bestRecordIt = std::min_element(openList.begin(), openList.end());
		currentNodeRecord = *bestRecordIt;		
		openList.erase(bestRecordIt);

		// B. Check if that record refers to the end node
		if (currentNodeRecord.pNode == pDestinationNode)
		{
			closedList.push_back(currentNodeRecord);
			break;
		}

		// C. Else, we get all the connections of the NodeRecord’s node
		auto connections = pGraph->FindConnectionsFrom(currentNodeRecord.pNode->GetId());

		// While Loop (part 2)
		for (auto* connection : connections)
		{
			// 1. Get the Node pNextnode this connection is pointing to
			Node* pNextNode = pGraph->GetNode(connection->GetToId()).get();

			// 2. Calculate the total G-Cost so far
			float gCost = currentNodeRecord.costSoFar + connection->GetWeight();

			// D. Check if the connection leads to a node already on the closedlist
			auto closedIt = std::find_if(closedList.begin(), closedList.end(), [pNextNode](const NodeRecord& record) { return record.pNode == pNextNode; });

			if (closedIt != closedList.end())
			{
				if (closedIt->costSoFar <= gCost)
				{
					continue;
				}
				else
				{
					closedList.erase(closedIt);
				}
			}

			// E. Check if the connection leads to a node already on the openlist
			auto openIt = std::find_if(openList.begin(), openList.end(), [pNextNode](const NodeRecord& record) { return record.pNode == pNextNode; });

			if (openIt != openList.end())
			{
				if (openIt->costSoFar <= gCost)
				{
					continue;
				}
				else
				{
					openList.erase(openIt);
				}
			}

			// F. At this point any expensive connection should be removed. Create a new NodeRecord
			NodeRecord newRecord;
			newRecord.pNode = pNextNode;
			newRecord.pConnection = connection;
			newRecord.costSoFar = gCost;
			newRecord.estimatedTotalCost = gCost + GetHeuristicCost(pNextNode, pDestinationNode);
			
			openList.push_back(newRecord);
		}

		// G. remove the currentNodeRecord from the openList and add it to the closedList
		openList.erase(std::remove(openList.begin(), openList.end(), currentNodeRecord), openList.end());
		closedList.push_back(currentNodeRecord);
	}

	// 3. Backtracking
	if (currentNodeRecord.pNode != pDestinationNode)
	{
		return path;
	}

	while (currentNodeRecord.pNode != pStartNode)
	{
		// Add the node of the currentRecord to the path
		path.push_back(currentNodeRecord.pNode);

		// Look in the closedList for a record where pNode == the currentRecords' connections' startNode
		int fromNodeId = currentNodeRecord.pConnection->GetFromId();
		auto prevIt = std::find_if(closedList.begin(), closedList.end(),
			[fromNodeId](const NodeRecord& record) { return record.pNode->GetId() == fromNodeId; });

		// Set the new currentRecord to the found record
		if (prevIt != closedList.end())
		{
			currentNodeRecord = *prevIt;
		}
		else
		{
			break;
		}
	}
	
	// Add the startNode to the path
	path.push_back(pStartNode);
	
	// Reverse and return the path
	std::reverse(path.begin(), path.end());

	return path;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
	FVector2D toDestination = pGraph->GetNode(pEndNode->GetId())->GetPosition() - pGraph->GetNode(pStartNode->GetId())->GetPosition();
	return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}