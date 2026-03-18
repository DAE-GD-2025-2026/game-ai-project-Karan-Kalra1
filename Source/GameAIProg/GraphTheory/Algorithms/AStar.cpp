#include "AStar.h"

#include <algorithm>

using namespace GameAI;

AStar::AStar(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction)
	: pGraph(pGraph)
	, HeuristicFunction(hFunction)
{
}

std::vector<Node*> AStar::FindPath(Node* const pStartNode, Node* const pGoalNode)
{
	std::vector<Node*> path{};

	if (pGraph == nullptr || pStartNode == nullptr || pGoalNode == nullptr)
	{
		return path;
	}

	std::vector<NodeRecord> openList{};
	std::vector<NodeRecord> closedList{};

	NodeRecord startRecord{};
	startRecord.pNode = pStartNode;
	startRecord.pConnection = nullptr;
	startRecord.costSoFar = 0.0f;
	startRecord.estimatedTotalCost = GetHeuristicCost(pStartNode, pGoalNode);

	openList.push_back(startRecord);

	NodeRecord currentRecord{};

	while (!openList.empty())
	{
		// get record with lowest f-cost
		auto currentIt = std::min_element(openList.begin(), openList.end());
		currentRecord = *currentIt;

		// stop if goal reached
		if (currentRecord.pNode == pGoalNode)
		{
			break;
		}

		// erase current before modifying openList anywhere else
		openList.erase(currentIt);

		const std::vector<Connection*> connections =
			pGraph->FindConnectionsFrom(currentRecord.pNode->GetId());

		for (Connection* const pConnection : connections)
		{
			if (pConnection == nullptr)
			{
				continue;
			}

			Node* const pNextNode = pGraph->GetNode(pConnection->GetToId()).get();
			if (pNextNode == nullptr)
			{
				continue;
			}

			const float newCostSoFar = currentRecord.costSoFar + pConnection->GetWeight();
			float heuristicCost = 0.0f;

			// check closed list
			auto closedIt = std::find_if(
				closedList.begin(),
				closedList.end(),
				[pNextNode](const NodeRecord& record)
				{
					return record.pNode == pNextNode;
				});

			if (closedIt != closedList.end())
			{
				if (closedIt->costSoFar <= newCostSoFar)
				{
					continue;
				}

				heuristicCost = closedIt->estimatedTotalCost - closedIt->costSoFar;
				closedList.erase(closedIt);
			}
			else
			{
				// check open list
				auto openIt = std::find_if(
					openList.begin(),
					openList.end(),
					[pNextNode](const NodeRecord& record)
					{
						return record.pNode == pNextNode;
					});

				if (openIt != openList.end())
				{
					if (openIt->costSoFar <= newCostSoFar)
					{
						continue;
					}

					heuristicCost = openIt->estimatedTotalCost - openIt->costSoFar;
					openList.erase(openIt);
				}
				else
				{
					heuristicCost = GetHeuristicCost(pNextNode, pGoalNode);
				}
			}

			NodeRecord nextRecord{};
			nextRecord.pNode = pNextNode;
			nextRecord.pConnection = pConnection;
			nextRecord.costSoFar = newCostSoFar;
			nextRecord.estimatedTotalCost = newCostSoFar + heuristicCost;

			openList.push_back(nextRecord);
		}

		closedList.push_back(currentRecord);
	}

	if (currentRecord.pNode != pGoalNode)
	{
		return path;
	}

	//BackTracking

	while (currentRecord.pNode != pStartNode)
	{
		path.push_back(currentRecord.pNode);

		if (currentRecord.pConnection == nullptr)
		{
			path.clear();
			return path;
		}

		const int fromNodeId = currentRecord.pConnection->GetFromId();

		auto parentIt = std::find_if(
			closedList.begin(),
			closedList.end(),
			[fromNodeId](const NodeRecord& record)
			{
				return record.pNode != nullptr && record.pNode->GetId() == fromNodeId;
			});

		if (parentIt == closedList.end())
		{
			path.clear();
			return path;
		}

		currentRecord = *parentIt;
	}

	path.push_back(pStartNode);
	std::reverse(path.begin(), path.end());

	return path;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
	FVector2D toDestination =
		pGraph->GetNode(pEndNode->GetId())->GetPosition() -
		pGraph->GetNode(pStartNode->GetId())->GetPosition();

	return HeuristicFunction(FMath::Abs(toDestination.X), FMath::Abs(toDestination.Y));
}