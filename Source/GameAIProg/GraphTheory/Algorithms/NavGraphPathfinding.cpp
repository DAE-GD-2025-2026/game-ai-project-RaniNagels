#include "NavGraphPathfinding.h"

#include "AStar.h"
#include "PathSmoothing.h"
#include "VectorTypes.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

using namespace GameAI;

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos,
	NavGraph* const pNavGraph, std::vector<FVector2D>& debugNodePositions, std::vector<NavLine>& debugPortals) 
{
	//Create the path to return
	std::vector<FVector2D> finalPath{};

	//Get the start and endTriangle
	auto* startTriangle = pNavGraph->GetNavPolygon()->GetTriangleAtPosition(startPos, true);
	auto* endTriangle = pNavGraph->GetNavPolygon()->GetTriangleAtPosition(endPos, true);
	if (startTriangle == nullptr || endTriangle == nullptr) return finalPath;
	if (startTriangle == endTriangle)
	{
		finalPath.push_back(startPos);
		finalPath.push_back(endPos);
		return finalPath;
	}

	//We have valid start/end triangles and they are not the same
	//=> Start looking for a path
	//Copy the graph
	auto clone = pNavGraph->Clone();
	
	//Create Extra node for the Start Node (Agent's position
	int startIdx = clone->AddNode(std::make_unique<NavGraphNode>(startPos, -1));
	
	int idx{};
	std::vector<std::pair<TriPolygon::Edge, int>> edgeWithIdx = pNavGraph->GetEdgesWithIdx();
	
	for (auto edge : startTriangle->GetEdges())
	{
		for (auto edgeIdx : edgeWithIdx)
		{
			if (edgeIdx.first == edge)
			{
				auto nodeId = clone->GetNodeIdFromEdgeIndex(edgeIdx.second);
				if (nodeId == -1) continue;
				
				std::unique_ptr<Connection> connection = std::make_unique<Connection>(startIdx, nodeId);
				connection->SetWeight(FVector2D::Distance(startPos, clone->GetNode(nodeId)->GetPosition()));
				
				clone->AddConnection(std::move(connection));
				break;
			}
		}
	}
	
	//Create extra node for the endNode
	auto endIdx = clone->AddNode(std::make_unique<NavGraphNode>(endPos, -1));

	for (auto edge : endTriangle->GetEdges())
	{
		for (auto edgeIdx : edgeWithIdx)
		{
			if (edgeIdx.first == edge)
			{
				auto nodeId = clone->GetNodeIdFromEdgeIndex(edgeIdx.second);
				if (nodeId == -1) continue;
				
				std::unique_ptr<Connection> connection = std::make_unique<Connection>(endIdx, nodeId);
				connection->SetWeight(FVector2D::Distance(startPos, clone->GetNode(nodeId)->GetPosition()));
				
				clone->AddConnection(std::move(connection));
				break;
			}
		}
	}
	
	//Run A star on new graph
	AStar astar{clone.get(), HeuristicFunctions::Euclidean};
	auto path = astar.FindPath(clone->GetNode(startIdx).get(), clone->GetNode(endIdx).get());
	
	for (auto p : path)
		finalPath.push_back(p->GetPosition());
	
	//Debug Visualisation

	// Extra: Run optimiser on new graph (First check if everything works without SSFA!)
	debugPortals = SSFA::FindPortals(path, *pNavGraph->GetNavPolygon());
	finalPath = SSFA::OptimizePortals(debugPortals, *pNavGraph->GetNavPolygon());
	
	return finalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};

	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals);
}