#pragma once
#include <stack>
#include "Shared/Graph/Graph.h"

namespace GameAI
{
	enum class Eulerianity
	{
		notEulerian,
		semiEulerian,
		eulerian,
	};

	class EulerianPath final
	{
	public:
		EulerianPath(Graph* const pGraph);

		Eulerianity IsEulerian() const;
		std::vector<Node*> FindPath(Eulerianity& eulerianity) const;

	private:
		void VisitAllNodesDFS(const std::vector<Node*>& pNodes, std::vector<bool>& visited, int startIndex) const;
		bool IsConnected() const;

		Graph* m_pGraph;
	};

	inline EulerianPath::EulerianPath(Graph* const pGraph)
		: m_pGraph(pGraph)
	{
	}

	inline Eulerianity EulerianPath::IsEulerian() const
	{
		// If the graph is not connected, there can be no Eulerian Trail
		if (!IsConnected())
		{
			return Eulerianity::notEulerian;
		}
	
		std::vector<Node*> Nodes = m_pGraph->GetActiveNodes();
		int oddCount = 0;
	
		// Count nodes with odd degree
		for (auto* node : Nodes)
		{
			auto connections = m_pGraph->FindConnectionsFrom(node->GetId());
			if (connections.size() % 2 != 0)
				{
				oddCount++;
			}
		}
	
		// A connected graph with more than 2 nodes with an odd degree (an odd amount of connections) is not Eulerian
		if (oddCount > 2)
		{
			return Eulerianity::notEulerian;
		}
	
		// A connected graph with exactly 2 nodes with an odd degree is Semi-Eulerian (unless there are only 2 nodes)
		if (oddCount == 2)
		{
			return Eulerianity::semiEulerian;
		}
		
		// An Euler trail can be made, but only starting and ending in these 2 nodes
		// A connected graph with no odd nodes is Eulerian
		return Eulerianity::eulerian;
	}

	inline std::vector<Node*> EulerianPath::FindPath(Eulerianity& eulerianity) const
	{				
		Graph graphCopy = m_pGraph->Clone();
		std::vector<Node*> Path = {};
		std::vector<Node*> Nodes = graphCopy.GetActiveNodes();
	
		// Check if there can be an Euler path
		eulerianity = IsEulerian();
		
		// If this graph is not eulerian, return the empty path
		if (eulerianity == Eulerianity::notEulerian || Nodes.size() == 0)
		{
			return Path;
		}

		// If Semi-Eulerian, start at an odd node
		int currentNodeId = Nodes[0]->GetId();
		if (eulerianity == Eulerianity::semiEulerian)
		{
			for (auto* node : Nodes)
			{
				if (graphCopy.FindConnectionsFrom(node->GetId()).size() % 2 != 0)
				{
					currentNodeId = node->GetId();
					break;
				}
			}
		}
	
		std::stack<int> nodeStack;

		// Start algorithm loop
		while (graphCopy.FindConnectionsFrom(currentNodeId).size() > 0 || !nodeStack.empty())
		{
			auto connections = graphCopy.FindConnectionsFrom(currentNodeId);
		
			if (connections.size() > 0)
			{
				nodeStack.push(currentNodeId);
				int neighborId = connections[0]->GetToId();
				graphCopy.RemoveConnection(currentNodeId, neighborId); // removes edge
				currentNodeId = neighborId;
			}
			else
			{
				Path.push_back(m_pGraph->GetNode(currentNodeId).get());
				currentNodeId = nodeStack.top();
				nodeStack.pop();
			}
		}
	
		Path.push_back(m_pGraph->GetNode(currentNodeId).get());
		std::reverse(Path.begin(), Path.end());
	
		return Path;
	}

	inline void EulerianPath::VisitAllNodesDFS(const std::vector<Node*>& Nodes, std::vector<bool>& visited, int startIndex ) const
	{
		// Mark the visited node
		visited[startIndex] = true;
		int currentNodeId = Nodes[startIndex]->GetId();

		// Ask the graph for the connections from that node
		auto connections = m_pGraph->FindConnectionsFrom(currentNodeId);

		// Recursively visit any valid connected nodes that were not visited before
		for (auto* conn : connections)
		{
			int neighborId = conn->GetToId();
		
			// Tip: use an index-based for-loop to find the correct index
			int neighborIndex = -1;
			for (int i = 0; i < Nodes.size(); ++i)
			{
				if (Nodes[i]->GetId() == neighborId)
				{
					neighborIndex = i;
					break;
				}
			}

			if (neighborIndex != -1 && !visited[neighborIndex])
			{
				VisitAllNodesDFS(Nodes, visited, neighborIndex);
			}
		}
	}

	inline bool EulerianPath::IsConnected() const
	{
		std::vector<Node*> Nodes = m_pGraph->GetActiveNodes();
		if (Nodes.empty())
		{
			return true;
		}

		std::vector<bool> visited(Nodes.size(), false);
		int startIndex = 0;

		VisitAllNodesDFS(Nodes, visited, startIndex);
		
		for (bool v : visited)
		{
			if (!v) return false;
		}
		return true;
	}
}