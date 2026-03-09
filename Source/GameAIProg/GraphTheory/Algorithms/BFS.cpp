#include "BFS.h"

#include <set>
#include <queue>
#include <unordered_map>

#include "Shared/Graph/Graph.h"

using namespace GameAI;

BFS::BFS(Graph* const pGraph)
	: pGraph(pGraph)
{
}

// TODO Breath First Search Algorithm searches for a path from the startNode to the destinationNode
std::vector<Node*> BFS::FindPath(Node* const pStartNode, Node* const pDestinationNode) const
{
	std::queue<Node*> queue;
	queue.push(pStartNode);
	std::set<Node*> visited;
	std::unordered_map<Node*, Node*> nodeMap;
	visited.insert(pStartNode);
	
	while(!queue.empty())
	{
		// front() will return the oldest element
		Node* pNode = queue.front();
		queue.pop();
		
		if (pNode == pDestinationNode)
		{
			return Reconstruct_Path(nodeMap, pStartNode, pDestinationNode);
		}
		else
		{
			for (auto connection : pGraph->FindConnectionsFrom(pNode->GetId()))
			{
				auto toNode = pGraph->GetNodeAs<Node>(connection->GetToId());
				if (visited.find(toNode) == visited.end())
				{
					visited.insert(toNode);
					nodeMap.insert({toNode, pNode});
					queue.push(toNode);
				}
			}
		}
	}
	std::vector<Node*> path{};
	return path;
}

std::vector<Node*> BFS::Reconstruct_Path(std::unordered_map<Node*, Node*> nodeMap, Node* const pStartNode, Node* const pDestinationNode) const
{
	Node* currentNode = pDestinationNode;
	std::vector<Node*> path;
	
	while (currentNode != pStartNode)
	{
		path.push_back(currentNode);
		currentNode = nodeMap.at(currentNode);
	}
	
	path.push_back(pStartNode);
	std::reverse(path.begin(), path.end());
	return path;
}
