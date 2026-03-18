#include "NavGraphPathfinding.h"

#include "AStar.h"
#include "PathSmoothing.h"
#include "VectorTypes.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

using namespace GameAI;

std::vector<NavLine> NavMeshPathfinding::m_portals{};


std::vector<FVector2D> NavMeshPathfinding::FindPath(
	const FVector2D& startPos,
	const FVector2D& endPos,
	NavGraph* const pNavGraph,
	std::vector<FVector2D>& debugNodePositions,
	std::vector<NavLine>& debugPortals)
{
	std::vector<FVector2D> finalPath{};

	if (pNavGraph == nullptr || pNavGraph->GetNavPolygon() == nullptr)
		return finalPath;

	const TriPolygon* pNavPoly = pNavGraph->GetNavPolygon();

	// Get the start and endTriangle
	const auto* pStartTriangle = pNavPoly->GetTriangleAtPosition(startPos, true);
	const auto* pEndTriangle = pNavPoly->GetTriangleAtPosition(endPos, true);

	if (pStartTriangle == nullptr || pEndTriangle == nullptr)
		return finalPath;

	// Same triangle => direct path
	if (*pStartTriangle == *pEndTriangle)
	{
		finalPath.push_back(startPos);
		finalPath.push_back(endPos);
		return finalPath;
	}

	// Copy the graph
	std::unique_ptr<NavGraph> pGraph = pNavGraph->Clone();
	if (!pGraph)
		return finalPath;

	// Create Extra node for the Start Node (Agent's position)
	const int startNodeId = pGraph->AddNode(
		std::make_unique<NavGraphNode>(startPos, -1)
	);

	{
		const auto startEdges = pStartTriangle->GetEdges();
		for (const auto& edge : startEdges)
		{
			const auto edgeIdx = pNavPoly->FindEdgeIndex(edge);
			if (!edgeIdx.has_value())
				continue;

			const int otherNodeId = pGraph->GetNodeIdFromEdgeIndex(edgeIdx.value());
			if (otherNodeId == Graphs::InvalidNodeId)
				continue;

			pGraph->AddConnection(startNodeId, otherNodeId);

			// Set actual cost
			if (Connection* pConn = pGraph->FindConnection(startNodeId, otherNodeId))
			{
				const FVector2D delta =
					pGraph->GetNode(startNodeId)->GetPosition() -
					pGraph->GetNode(otherNodeId)->GetPosition();
				pConn->SetWeight(delta.Length());
			}
			if (Connection* pConn = pGraph->FindConnection(otherNodeId, startNodeId))
			{
				const FVector2D delta =
					pGraph->GetNode(otherNodeId)->GetPosition() -
					pGraph->GetNode(startNodeId)->GetPosition();
				pConn->SetWeight(delta.Length());
			}
		}
	}

	// Create extra node for the endNode
	const int endNodeId = pGraph->AddNode(
		std::make_unique<NavGraphNode>(endPos, -1)
	);

	{
		const auto endEdges = pEndTriangle->GetEdges();
		for (const auto& edge : endEdges)
		{
			const auto edgeIdx = pNavPoly->FindEdgeIndex(edge);
			if (!edgeIdx.has_value())
				continue;

			const int otherNodeId = pGraph->GetNodeIdFromEdgeIndex(edgeIdx.value());
			if (otherNodeId == Graphs::InvalidNodeId)
				continue;

			pGraph->AddConnection(endNodeId, otherNodeId);

			// Set actual cost
			if (Connection* pConn = pGraph->FindConnection(endNodeId, otherNodeId))
			{
				const FVector2D delta =
					pGraph->GetNode(endNodeId)->GetPosition() -
					pGraph->GetNode(otherNodeId)->GetPosition();
				pConn->SetWeight(delta.Length());
			}
			if (Connection* pConn = pGraph->FindConnection(otherNodeId, endNodeId))
			{
				const FVector2D delta =
					pGraph->GetNode(otherNodeId)->GetPosition() -
					pGraph->GetNode(endNodeId)->GetPosition();
				pConn->SetWeight(delta.Length());
			}
		}
	}

	// Run A star on new graph
	AStar aStar{ pGraph.get(), HeuristicFunctions::Chebyshev };
	std::vector<Node*> nodes = aStar.FindPath(
		pGraph->GetNode(startNodeId).get(),
		pGraph->GetNode(endNodeId).get()
	);

	// Debug Visualisation
	for (Node* pNode : nodes)
	{
		if (pNode)
		{
			debugNodePositions.push_back(pNode->GetPosition());
			finalPath.push_back(pNode->GetPosition());
		}
	}

	// Extra: Run optimiser on new graph
	 debugPortals = SSFA::FindPortals(nodes, *pNavGraph->GetNavPolygon());
	 finalPath = SSFA::OptimizePortals(debugPortals, *pNavGraph->GetNavPolygon());

	SetPortals(debugPortals);

	return finalPath;
}
std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};

	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals);
}