#include "NavGraph.h"

#include <cmath>

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

std::vector<std::pair<TriPolygon::Edge, int>> GameAI::NavGraph::GetEdgesWithIdx() const
{
	int edgeIdx{};
	std::vector<std::pair<TriPolygon::Edge, int>> edgeWithIdx;
	for (auto line : pNavPoly->GetEdges())
	{
		edgeWithIdx.push_back(std::pair<TriPolygon::Edge, int>(line, edgeIdx));
		edgeIdx++;
	}
	
	return edgeWithIdx;
}

void GameAI::NavGraph::CreateNavigationGraph()
{
	//1. Go over all the edges of the navigation mesh and create nodes
			// Create node here
	int edgeIdx{};
	for (auto line : pNavPoly->GetEdges())
	{
		std::vector<TriPolygon::Triangle> connectedTriangles;
		for (auto& triangle : pNavPoly->GetTriangles())
		{
			if (triangle.HasEdge(line))
				connectedTriangles.push_back(triangle);
		}
		
		if (connectedTriangles.size() >= 2)
		{
			auto X = line.GetP1(*pNavPoly.get()).X + line.GetP2(*pNavPoly.get()).X;
			auto Y = line.GetP1(*pNavPoly.get()).Y + line.GetP2(*pNavPoly.get()).Y;
			auto position = FVector2D{X/2, Y/2};

			this->AddNode(std::make_unique<NavGraphNode>(position, edgeIdx));
		}
		
		edgeIdx++;
	}

	//2. Create connections now that every node is created	
		//2 valid nodes -> 1 connection
		//3 valid nodes -> 3 connections
	std::vector<std::pair<TriPolygon::Edge, int>> edgeWithIdx = GetEdgesWithIdx();
	for (auto triangle : pNavPoly->GetTriangles())
	{
		std::vector<int> triangleNodes;
		for (auto triangleEdge : triangle.GetEdges())
		{
			int idx{};
			for (auto eid : edgeWithIdx)
			{
				if (eid.first == triangleEdge)
				{
					idx = eid.second;
					break;
				}
			}
			int nodeIdx = this->GetNodeIdFromEdgeIndex(idx);
			if (nodeIdx >= 0)
				triangleNodes.push_back(nodeIdx);
		}
		
		if (triangleNodes.size() == 2)
		{
			this->AddConnection(std::make_unique<Connection>(triangleNodes[0], triangleNodes[1]));
		}
		else if (triangleNodes.size() == 3)
		{
			this->AddConnection(std::make_unique<Connection>(triangleNodes[0], triangleNodes[1]));
			this->AddConnection(std::make_unique<Connection>(triangleNodes[1], triangleNodes[2]));
			this->AddConnection(std::make_unique<Connection>(triangleNodes[2], triangleNodes[0]));
		}
	}
		
	//3. Set the connections cost to the actual distance
	for (auto& connection : this->GetConnections())
	{
		// calculate distance
		auto& node1 = GetNode(connection->GetFromId())->GetPosition();
		auto& node2 = GetNode(connection->GetToId())->GetPosition();
		
		float distance = FVector2D::Distance(node1, node2);
		connection->SetWeight(distance);
	}
}
