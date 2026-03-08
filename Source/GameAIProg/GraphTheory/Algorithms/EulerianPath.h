#pragma once
#include <stack>
#include <algorithm>
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
			return Eulerianity::notEulerian;

		// Count nodes with odd degree
		const std::vector<Node*> nodes = m_pGraph->GetActiveNodes();
		int oddDegreeCount = 0;

		for (Node* node : nodes)
		{
			const int degree = static_cast<int>(m_pGraph->FindConnectionsFrom(node->GetId()).size());
			if (degree % 2 != 0)
			{
				++oddDegreeCount;
			}
		}

		// More than 2 odd-degree nodes => not Eulerian
		if (oddDegreeCount > 2)
			return Eulerianity::notEulerian;

		// Exactly 2 odd-degree nodes => semi-Eulerian
		if (oddDegreeCount == 2)
			return Eulerianity::semiEulerian;

		// No odd-degree nodes => Eulerian
		return Eulerianity::eulerian;
	}

	inline std::vector<Node*> EulerianPath::FindPath(Eulerianity& eulerianity) const
	{
		// Get a copy of the graph because this algorithm involves removing edges
		Graph graphCopy = m_pGraph->Clone();
		std::vector<Node*> path = {};
		std::vector<Node*> nodes = graphCopy.GetActiveNodes();
		int currentNodeId{ Graphs::InvalidNodeId };

		// Check if there can be an Euler path
		eulerianity = IsEulerian();

		// If this graph is not eulerian, return the empty path
		if (eulerianity == Eulerianity::notEulerian || nodes.empty())
			return path;

		// Choose a starting node
		// If semi-Eulerian: start at one of the odd-degree nodes
		if (eulerianity == Eulerianity::semiEulerian)
		{
			for (Node* node : nodes)
			{
				if (graphCopy.FindConnectionsFrom(node->GetId()).size() % 2 != 0)
				{
					currentNodeId = node->GetId();
					break;
				}
			}
		}
		else
		{
			// If all nodes have even degree: choose any node
			currentNodeId = nodes[0]->GetId();
		}

		// Start algorithm loop
		std::stack<int> nodeStack;

		while (!graphCopy.FindConnectionsFrom(currentNodeId).empty() || !nodeStack.empty())
		{
			// If the current node has neighbors
			if (!graphCopy.FindConnectionsFrom(currentNodeId).empty())
			{
				// Add the node to the stack
				nodeStack.push(currentNodeId);

				// Take any of its neighbors
				Connection* nextConnection = graphCopy.FindConnectionsFrom(currentNodeId)[0];

				// Set that neighbor as the current node
				int nextNodeId = nextConnection->GetToId();

				// Remove the edge from the copy of the graph
				graphCopy.RemoveConnection(nextConnection);

				currentNodeId = nextNodeId;
			}
			else
			{
				// Dead end: add current node to path
				path.push_back(m_pGraph->GetNode(currentNodeId).get());

				// Backtrack
				currentNodeId = nodeStack.top();
				nodeStack.pop();
			}
		}

		// Also add the last current node to the path
		path.push_back(m_pGraph->GetNode(currentNodeId).get());

		// Path is in reversed order
		std::reverse(path.begin(), path.end());
		return path;
	}

	inline void EulerianPath::VisitAllNodesDFS(const std::vector<Node*>& nodes, std::vector<bool>& visited, int startIndex) const
	{
		// Mark the visited node
		visited[startIndex] = true;

		// Ask the graph for the connections from that node
		const int nodeId = nodes[startIndex]->GetId();
		const auto connections = m_pGraph->FindConnectionsFrom(nodeId);

		// Recursively visit any valid connected nodes that were not visited before
		for (Connection* connection : connections)
		{
			const int otherNodeId = connection->GetToId();

			// loop to find the correct index
			for (int i = 0; i < static_cast<int>(nodes.size()); ++i)
			{
				if (nodes[i]->GetId() == otherNodeId)
				{
					if (!visited[i])
					{
						VisitAllNodesDFS(nodes, visited, i);
					}
					break;
				}
			}
		}
	}

	inline bool EulerianPath::IsConnected() const
	{
		std::vector<Node*> nodes = m_pGraph->GetActiveNodes();
		if (nodes.size() == 0)
			return false;

		std::vector<bool> visited(nodes.size(), false);

		// Choose a starting node
		int startIndex = 0;

		// Start a depth-first-search traversal from that node
		VisitAllNodesDFS(nodes, visited, startIndex);

		// If a node was never visited, this graph is not connected
		for (bool wasVisited : visited)
		{
			if (!wasVisited)
				return false;
		}

		return true;
	}
}