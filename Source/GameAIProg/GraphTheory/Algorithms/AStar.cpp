#include "AStar.h"

using namespace GameAI;

AStar::AStar(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction)
	: pGraph(pGraph)
	, HeuristicFunction(hFunction)
{
}

std::vector<Node*>AStar::FindPath(Node* const pStartNode, Node* const pGoalNode)
{
	std::vector<Node*> path{};
	std::vector<NodeRecord> openList{}; // nodes yet to be checked
	std::vector<NodeRecord> closedList{}; // nodes already checked
	NodeRecord currentNodeRecord{}; // the information relating to the current node
	
	// 1. kickstart the loop
	NodeRecord startNodeRecord{};
	startNodeRecord.pNode = pStartNode;
	startNodeRecord.pConnection = nullptr;
	startNodeRecord.costSoFar = 0;
	startNodeRecord.estimatedTotalCost = GetHeuristicCost(pStartNode, pGoalNode);
	openList.emplace_back(startNodeRecord);
	currentNodeRecord = openList.front();

	// Keep track of the closest node to the goal in case we cannot find a path to the goal
	NodeRecord closestNodeRecord = startNodeRecord;
	float closestNodeHeuristicCost = GetHeuristicCost(pStartNode, pGoalNode);
	
	// 2. the while loop
	while (!openList.empty())
	{
		// A. Get record from the open list with lowest f-score
		currentNodeRecord = *std::min_element(openList.begin(), openList.end());

		float currentHeuristicCost = GetHeuristicCost(currentNodeRecord.pNode, pGoalNode);
		if (currentHeuristicCost < closestNodeHeuristicCost)
		{
			closestNodeRecord = currentNodeRecord;
			closestNodeHeuristicCost = currentHeuristicCost;
		}
		
		// B. Check if that record refers to the end node
		if (currentNodeRecord.pNode == pGoalNode)
			break;
		
		// C. Else, we get all the connections of the NodeRecord's node
		for (auto connection : pGraph->FindConnectionsFrom(currentNodeRecord.pNode->GetId()))
		{
			// C1. Get the Node pNextnode this connection is pointing to
			auto pNextNodeIndex = connection->GetToId();
			auto pNextNode = pGraph->GetNodeAs<Node>(pNextNodeIndex);
			
			// C2. Calculate the total G-Cost so far (prev. G-Cost + connection->GetCost())
			float GCostSoFar = currentNodeRecord.costSoFar + connection->GetWeight();
			
			// D. Check if the connection leads to a node already on the closedlist
			if (!RemoveIfWorse(closedList, pNextNode, GCostSoFar))
				continue;

			// E. Check if the connection leads to a node already on the openlist
			if (!RemoveIfWorse(openList, pNextNode, GCostSoFar))
				continue;
			
			// F. At this point, any expensive connection should be removed (if it existed).
			// We Create a new NodeRecord and add it to the openlist
			NodeRecord newNodeRecord{};
			newNodeRecord.pNode = pNextNode;
			newNodeRecord.pConnection = connection;
			newNodeRecord.costSoFar = GCostSoFar;
			newNodeRecord.estimatedTotalCost = GCostSoFar + GetHeuristicCost(pNextNode, pGoalNode);
			openList.emplace_back(newNodeRecord);
		}
		// G. Remove the CurrentNodeRecord from the openList and add it to the closedList
		std::erase(openList, currentNodeRecord);
		closedList.emplace_back(currentNodeRecord);
	}

	bool pathFound = currentNodeRecord.pNode == pGoalNode;
	if (!pathFound)
	{
		auto it = std::find(closedList.begin(), closedList.end(), closestNodeRecord);

		if (it == closedList.end())
			closedList.emplace_back(closestNodeRecord);

		currentNodeRecord = closestNodeRecord;
	}
	
	// 3. Reconstruct path from last connection to start node
	while (currentNodeRecord.pConnection != nullptr)
	{
		path.emplace_back(currentNodeRecord.pNode);
		auto previousNode = pGraph->GetNodeAs<Node>(currentNodeRecord.pConnection->GetFromId());
		for (const auto& record : closedList)
		{
			if (record.pNode == previousNode)
			{
				currentNodeRecord = record;
				// break must be in the if, else => infinity loop!
				break;
			}
		}
	}
	path.emplace_back(pStartNode);
	std::reverse(path.begin(), path.end());
	
	return path;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
	FVector2D toDestination = pGraph->GetNode(pEndNode->GetId())->GetPosition() - pGraph->GetNode(pStartNode->GetId())->GetPosition();
	return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}

bool AStar::RemoveIfWorse(std::vector<NodeRecord>& list, Node* node, float newCost)
{
	for (auto it = list.begin(); it != list.end(); ++it)
	{
		if (it->pNode == node)
		{
			if (it->costSoFar <= newCost)
				return false;

			list.erase(it);
			return true;
		}
	}
	return true;
}
