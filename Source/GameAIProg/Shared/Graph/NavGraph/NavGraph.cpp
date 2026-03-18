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
	// 1. Go over all the edges of the navigation mesh and create nodes
	const auto& edges = pNavPoly->GetEdges();

	for (int edgeIdx = 0; edgeIdx < static_cast<int>(edges.size()); ++edgeIdx)
	{
		// A navgraph node only exists on edges shared by 2 triangles
		int connectedTriangleCount = 0;

		for (const auto& triangle : pNavPoly->GetTriangles())
		{
			const auto triEdges = triangle.GetEdges();
			for (const auto& triEdge : triEdges)
			{
				if (triEdge == edges[edgeIdx])
				{
					++connectedTriangleCount;
					break;
				}
			}
		}

		if (connectedTriangleCount >= 2)
		{
			const FVector p1 = edges[edgeIdx].GetP1(*pNavPoly);
			const FVector p2 = edges[edgeIdx].GetP2(*pNavPoly);
			const FVector middle = (p1 + p2) * 0.5f;

			AddNode(std::make_unique<NavGraphNode>(
				FVector2D{ middle.X, middle.Y },
				edgeIdx
			));
		}
	}

	// 2. Create connections now that every node is created
	for (const auto& triangle : pNavPoly->GetTriangles())
	{
		std::vector<int> nodeIds{};

		const auto triEdges = triangle.GetEdges();
		for (const auto& triEdge : triEdges)
		{
			const auto edgeIdx = pNavPoly->FindEdgeIndex(triEdge);
			if (!edgeIdx.has_value())
				continue;

			const int nodeId = GetNodeIdFromEdgeIndex(edgeIdx.value());
			if (nodeId != Graphs::InvalidNodeId)
			{
				nodeIds.push_back(nodeId);
			}
		}

		// 2 valid nodes -> 1 connection
		// 3 valid nodes -> 3 connections
		for (int i = 0; i < static_cast<int>(nodeIds.size()); ++i)
		{
			for (int j = i + 1; j < static_cast<int>(nodeIds.size()); ++j)
			{
				AddConnection(nodeIds[i], nodeIds[j]);
			}
		}
	}

	// 3. Set the connections cost to the actual distance
	SetConnectionCostsToDistances();
}
