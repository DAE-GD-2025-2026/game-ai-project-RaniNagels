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
		if (!IsConnected()) return Eulerianity::notEulerian;

		// TODO Count nodes with odd degree (not yet directional)
		int count{};
		for (auto* node : m_pGraph->GetActiveNodes())
		{
			int amountOfConnections{};
			amountOfConnections += m_pGraph->FindConnectionsFrom(node->GetId()).size();
			//amountOfConnections += m_pGraph->FindConnectionsTo(node->GetId()).size();
			if (amountOfConnections % 2 == 1) ++count;
		}

		// TODO A connected graph with more than 2 nodes with an odd degree (an odd amount of connections) is not Eulerian
		if (count > 2) return Eulerianity::notEulerian;

		// TODO A connected graph with exactly 2 nodes with an odd degree is Semi-Eulerian (unless there are only 2 nodes)
		// TODO An Euler trail can be made, but only starting and ending in these 2 nodes
		if (count == 2 && m_pGraph->GetActiveNodes().size() != 2) return Eulerianity::semiEulerian;

		// TODO A connected graph with no odd nodes is Eulerian
		return Eulerianity::eulerian;
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
		switch (eulerianity)
		{
		case Eulerianity::eulerian:
			currentNodeId = Nodes.front()->GetId();
			break;
		case Eulerianity::semiEulerian:
			for (auto node : Nodes)
			{
				int amount{};
				amount += graphCopy.FindConnectionsFrom(node->GetId()).size();
				//amount += graphCopy.FindConnectionsTo(node->GetId()).size();
				if (amount % 2 == 1)
				{
					currentNodeId = node->GetId();
					break;
				}
			}
			break;
		case Eulerianity::notEulerian:
			return Path;
			break;
		}

		// TODO Start algorithm loop
		std::stack<int> nodeStack{};
		while (graphCopy.FindConnectionsFrom(currentNodeId).size() != 0
			|| !nodeStack.empty())
		{
			if (graphCopy.FindConnectionsFrom(currentNodeId).size() != 0)
			{
				nodeStack.push(currentNodeId);
				
				auto* connection = graphCopy.FindConnectionsFrom(currentNodeId).front();
				currentNodeId = connection->GetToId();
				graphCopy.RemoveConnection(connection);
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
		if (startIndex >= Nodes.size()) return;
		if (Nodes.size() != visited.size()) return;

		// TODO Mark the visited node
		visited[startIndex] = true;

		// TODO Ask the graph for the connections from that node
		auto connections = m_pGraph->FindConnectionsFrom(Nodes[startIndex]->GetId());

		// TODO recursively visit any valid connected nodes that were not visited before
		// TODO Tip: use an index-based for-loop to find the correct index
		for (int i{}; i < connections.size(); ++i)
		{
			int nextIndex = connections[i]->GetToId();
			if (visited[nextIndex] == false)
				VisitAllNodesDFS(Nodes, visited, connections[i]->GetToId());
		}
	}

	inline bool EulerianPath::IsConnected() const
	{
		std::vector<Node*> nodes = m_pGraph->GetActiveNodes();
		if (nodes.size() == 0)
			return false;

		// TODO choose a starting node
		int startIndex{0};
		
		// TODO start a depth-first-search traversal from the node that has at least one connection
		std::vector<bool>visited(nodes.size());
		VisitAllNodesDFS(nodes, visited, startIndex);
		
		// TODO if a node was never visited, this graph is not connected
		for (bool v : visited)
		{
			if (v == false) return false;
		}

		return true;
	}
}