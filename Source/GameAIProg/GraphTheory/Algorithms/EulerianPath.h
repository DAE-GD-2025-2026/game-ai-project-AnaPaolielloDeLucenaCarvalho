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
		// TODO If the graph is not connected, there can be no Eulerian Trail
		if (!IsConnected())
		{
			return Eulerianity::notEulerian;
		}
		
		// TODO Count nodes with odd degree
		//if (/* TODO if there are more than 2 nodes with an odd degree */)
		//{
		//	return Eulerianity::notEulerian;
		//}
		
		// TODO A connected graph with more than 2 nodes with an odd degree (an odd amount of connections) is not Eulerian


		// TODO A connected graph with exactly 2 nodes with an odd degree is Semi-Eulerian (unless there are only 2 nodes)
		// TODO An Euler trail can be made, but only starting and ending in these 2 nodes

		// TODO A connected graph with no odd nodes is Eulerian
		
		return Eulerianity::notEulerian;
	}

	inline std::vector<Node*> EulerianPath::FindPath(Eulerianity& eulerianity) const
	{
		// Get a copy of the graph because this algorithm involves removing edges
		Graph graphCopy = m_pGraph->Clone();
		std::vector<Node*> Path = {};
		std::vector<Node*> Nodes = graphCopy.GetActiveNodes();
		int currentNodeId{ Graphs::InvalidNodeId };
		
		// TODO Check if there can be an Euler path
		// TODO If this graph is not eulerian, return the empty path
		
		// TODO Start algorithm loop
		std::stack<int> nodeStack;

		std::reverse(Path.begin(), Path.end());
		return Path;
	}

	inline void EulerianPath::VisitAllNodesDFS(const std::vector<Node*>& Nodes, std::vector<bool>& visited, int startIndex ) const
	{
		// TODO Mark the visited node

		// TODO Ask the graph for the connections from that node
		// TODO recursively visit any valid connected nodes that were not visited before
		// TODO Tip: use an index-based for-loop to find the correct index
	}

	inline bool EulerianPath::IsConnected() const
	{
		std::vector<Node*> Nodes = m_pGraph->GetActiveNodes();
		if (Nodes.size() == 0)
			return false;

		// TODO choose a starting node
		
		// TODO start a depth-first-search traversal from the node that has at least one connection
		
		// TODO if a node was never visited, this graph is not connected
	}
}