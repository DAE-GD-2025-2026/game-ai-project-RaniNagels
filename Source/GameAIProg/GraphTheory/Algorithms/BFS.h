#pragma once
#include <vector>
#include <unordered_map>

namespace GameAI
{
	class Graph;
	class Node;

	class BFS
	{
	public:
		BFS(Graph* const pGraph);

		std::vector<Node*> FindPath(Node* const pStartNode, Node* const pDestinationNode) const;
		
	private:
		Graph* pGraph;
		
		std::vector<Node*> Reconstruct_Path(std::unordered_map<Node*, Node*> nodeMap, Node* const pStartNode, Node* const pDestinationNode) const;
	};
}
