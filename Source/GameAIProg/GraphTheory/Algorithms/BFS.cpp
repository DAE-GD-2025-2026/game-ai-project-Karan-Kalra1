#include "BFS.h"

#include <map>
#include <queue>

#include "Shared/Graph/Graph.h"

using namespace GameAI;

BFS::BFS(Graph* const pGraph)
	: pGraph(pGraph)
{
}

// TODO Breath First Search Algorithm searches for a path from the startNode to the destinationNode
std::vector<Node*> BFS::FindPath(Node* const pStartNode, Node* const pDestinationNode) const
{
	std::vector<Node*> path{};

	if (pGraph == nullptr || pStartNode == nullptr || pDestinationNode == nullptr)
	{
		return path;
	}

	std::queue<Node*> openQueue{};
	std::map<int, bool> visited{};
	std::map<int, int> parentOf{};

	openQueue.push(pStartNode);
	visited[pStartNode->GetId()] = true;
	parentOf[pStartNode->GetId()] = -1;

	bool pathFound = false;

	while (!openQueue.empty())
	{
		Node* const pCurrentNode = openQueue.front();
		openQueue.pop();

		if (pCurrentNode == pDestinationNode)
		{
			pathFound = true;
			break;
		}

		const std::vector<Connection*> connections = pGraph->FindConnectionsFrom(pCurrentNode->GetId());

		for (Connection* const pConnection : connections)
		{
			Node* const pNextNode = pGraph->GetNode(pConnection->GetToId()).get();
			const int nextId = pNextNode->GetId();

			if (!visited[nextId])
			{
				visited[nextId] = true;
				parentOf[nextId] = pCurrentNode->GetId();
				openQueue.push(pNextNode);
			}
		}
	}

	if (!pathFound)
	{
		return path;
	}

	int currentId = pDestinationNode->GetId();

	while (currentId != -1)
	{
		path.push_back(pGraph->GetNode(currentId).get());
		currentId = parentOf[currentId];
	}

	std::reverse(path.begin(), path.end());
	return path;
}
