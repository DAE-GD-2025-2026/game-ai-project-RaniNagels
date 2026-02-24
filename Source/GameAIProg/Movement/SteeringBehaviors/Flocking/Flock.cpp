#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "../SpacePartitioning/SpacePartitioning.h"
#include "Shared/ImGuiHelpers.h"
#include <string>

Flock::Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize,
	float WorldSize,
	ASteeringAgent* const pAgentToEvade,
	bool bTrimWorld)
	: pWorld{pWorld}
	, FlockSize{ FlockSize }
	, WorldSize{WorldSize}
	, TrimWorld{ bTrimWorld }
	, pAgentToEvade{pAgentToEvade}
{
	pCohesionBehavior = std::make_unique<Cohesion>(this);
	pSeparationBehavior = std::make_unique<Separation>(this);
	pVelMatchBehavior = std::make_unique<VelocityMatch>(this);
	pSeekBehavior = std::make_unique<Seek>();
	pWanderBehavior = std::make_unique<Wander>();
	pEvadeBehavior = std::make_unique<Evade>();

	std::vector<BlendedSteering::WeightedBehavior> weightedBehaviors{};
	weightedBehaviors.push_back({ pCohesionBehavior.get(), 0.2f });
	weightedBehaviors.push_back({ pSeparationBehavior.get(), 0.2f });
	weightedBehaviors.push_back({ pVelMatchBehavior.get(), 0.2f });
	weightedBehaviors.push_back({ pSeekBehavior.get(), 0.2f });
	weightedBehaviors.push_back({ pWanderBehavior.get(), 0.2f });

	pBlendedSteering = std::make_unique<BlendedSteering>(weightedBehaviors);

	std::vector<ISteeringBehavior*> steeringBehaviors{};
	steeringBehaviors.push_back(pEvadeBehavior.get());
	steeringBehaviors.push_back(pBlendedSteering.get());
	pPrioritySteering = std::make_unique<PrioritySteering>(steeringBehaviors);

	Neighbors.SetNum(FlockSize);

	pPartitionedSpace = std::make_unique<CellSpace>(pWorld, WorldSize, WorldSize, NrOfCellsX, NrOfCellsX, FlockSize);
	pPartitionedSpace->EmptyCells();

	OldPositions.SetNum(FlockSize);

	// initialize the flock and the memory pool
	Agents.SetNum(FlockSize);
	for (int i{}; i < FlockSize; ++i)
	{
		ASteeringAgent* newAgent = nullptr;
		while (newAgent == nullptr)
		{
			// spawn the agents in random locations, Z should always be 90
			float RandX = FMath::FRandRange(-WorldSize * 0.5f, WorldSize * 0.5f);
			float RandY = FMath::FRandRange(-WorldSize * 0.5f, WorldSize * 0.5f);

			FVector SpanwLocation = FVector(RandX, RandY, 90);
			newAgent = pWorld->SpawnActor<ASteeringAgent>(AgentClass, SpanwLocation, FRotator::ZeroRotator);
		}
		Agents[i] = newAgent;
		OldPositions[i] = Agents[i]->GetPosition();

		if (Agents[i] == nullptr)
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to spawn agent %d"), i);
			continue;
		}
		Agents[i]->SetSteeringBehavior(pPrioritySteering.get());
		Agents[i]->SetDebugRenderingEnabled(false);

		if (UsingSpacialPartitioning)
			pPartitionedSpace->AddAgent(*Agents[i]);
	}
}

Flock::~Flock()
{
 // Cleanup any additional data
	pPartitionedSpace->EmptyCells();
}

void Flock::Tick(float DeltaTime)
{
  // update the flock
  // for every agent:
  // register the neighbors for this agent (-> fill the memory pool with the neighbors for the currently evaluated agent)
  // update the agent (-> the steeringbehaviors use the neighbors in the memory pool)
  // trim the agent to the world

	FTargetData EvadeTargetData{};
	EvadeTargetData.Position = pAgentToEvade->GetPosition();
	EvadeTargetData.Orientation = pAgentToEvade->GetRotation();
	EvadeTargetData.LinearVelocity = pAgentToEvade->GetLinearVelocity();
	EvadeTargetData.AngularVelocity = pAgentToEvade->GetAngularVelocity();
	pEvadeBehavior->SetTarget(EvadeTargetData);

	for (int i {}; i < Agents.Num(); ++i)
	{
		if (Agents[i] == nullptr) continue;

		if (UsingSpacialPartitioning) pPartitionedSpace->UpdateAgentCell(*Agents[i], OldPositions[i]);
		RegisterNeighbors(Agents[i]);

		Agents[i]->Tick(DeltaTime);
		
		TrimToWorld(Agents[i], i);
		if (UsingSpacialPartitioning) OldPositions[i] = Agents[i]->GetPosition();
	}

	TrimToWorld(pAgentToEvade);
}

void Flock::RenderDebug()
{
	// Render trim world
	DrawDebugBox(pWorld, FVector::ZeroVector, FVector(WorldSize * 0.5f, WorldSize * 0.5f, 0), FColor::Red);

	if (DebugRenderPartitions && UsingSpacialPartitioning) pPartitionedSpace->RenderCells();

	// Render the debug first agent in the flock! and only the debug as well as its neighborhood
	for (size_t i{}; i < GetNrOfNeighbors(); ++i)

	{
		if (Agents[i] != nullptr)
		{
			RenderNeighborhood(Agents[i]);
			Agents[i]->SetDebugRenderingEnabled(DebugRenderSteering);
			return;
		}
	}
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	//UI
	{
		//Setup
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scrollwheel: zoom cam.");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();

		// TODO: implement ImGUI checkboxes for debug rendering here
		ImGui::Checkbox("DebugRender First Agent", &DebugRenderSteering);
		ImGui::Checkbox("DebugRender Neighborhood", &DebugRenderNeighborhood);

		ImGui::Spacing();
		if (ImGui::Checkbox("Use SpacialPartitioning", &UsingSpacialPartitioning))
		{
			if (UsingSpacialPartitioning)
				StartUsingSpacialPartitioning();
			else 
				StopUsingSpacialPartitioning();
		}
		if (UsingSpacialPartitioning)
			ImGui::Checkbox("DebugRender Partitions", &DebugRenderPartitions);

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Cohesion",
			pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f,
			[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight = InVal; }, "%.2f");

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Separation",
			pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight, 0.f, 1.f,
			[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight = InVal; }, "%.2f");

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Velocity Match",
			pBlendedSteering->GetWeightedBehaviorsRef()[2].Weight, 0.f, 1.f,
			[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[2].Weight = InVal; }, "%.2f");

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Seek",
			pBlendedSteering->GetWeightedBehaviorsRef()[3].Weight, 0.f, 1.f,
			[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[3].Weight = InVal; }, "%.2f");

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander",
			pBlendedSteering->GetWeightedBehaviorsRef()[4].Weight, 0.f, 1.f,
			[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[4].Weight = InVal; }, "%.2f");

		float coh = pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight;
		float sep = pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight;
		float vel = pBlendedSteering->GetWeightedBehaviorsRef()[2].Weight;
		float see = pBlendedSteering->GetWeightedBehaviorsRef()[3].Weight;
		float wan = pBlendedSteering->GetWeightedBehaviorsRef()[4].Weight;
		float total = coh + sep + vel + see + wan;
		float rw1 = coh / total;
		float rw2 = sep / total;
		float rw3 = vel / total;
		float rw4 = see / total;
		float rw5 = wan / total;

		ImGui::Spacing();
		ImGui::Text("Actual Weights:");
		ImGui::Text(("Cohesion:   " + std::to_string(rw1)).c_str());
		ImGui::Text(("Separation: " + std::to_string(rw2)).c_str());
		ImGui::Text(("Alignment:  " + std::to_string(rw3)).c_str());
		ImGui::Text(("Seek:       " + std::to_string(rw4)).c_str());
		ImGui::Text(("Wander:     " + std::to_string(rw5)).c_str());

		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood(ASteeringAgent* const pAgent)
{
	// Debugrender the neighbors for the first agent in the flock
	// hightlight the agents in the neighborhood and draw circle
	if (DebugRenderNeighborhood)
	{
		DrawDebugCircle(pWorld, FVector(pAgent->GetPosition(), 0), NeighborhoodRadius, 32, FColor::Red, false, -1.f, 0, 10.f, FVector(1.f, 0.f, 0.f), FVector(0.f, 1.f, 0.f), false);
		RegisterNeighbors(pAgent);
		for (int i{}; i < GetNrOfNeighbors(); ++i)
		{
			DrawDebugSphere(pWorld, FVector{ GetNeighbors()[i]->GetPosition(), 90 }, 20, 16, FColor::Emerald);
		}
	}
}

void Flock::TrimToWorld(ASteeringAgent* const pAgent, int index)
{
	float halfSize = WorldSize / 2;
	float safetyOffset = 10; // to prevent leaving the grid even slightly and messing up the spacial partitioning
	halfSize -= safetyOffset;
	bool moved = false;
	if (pAgent->GetPosition().X > halfSize)
	{
		pAgent->SetActorLocation(FVector(-halfSize, pAgent->GetPosition().Y, 90));
		moved = true;
	}
	else if (pAgent->GetPosition().X < -halfSize)
	{
		pAgent->SetActorLocation(FVector(halfSize, pAgent->GetPosition().Y, 90));
		moved = true;
	}

	if (pAgent->GetPosition().Y > halfSize)
	{
		pAgent->SetActorLocation(FVector(pAgent->GetPosition().X, -halfSize, 90));
		moved = true;
	}
	else if (pAgent->GetPosition().Y < -halfSize)
	{
		pAgent->SetActorLocation(FVector(pAgent->GetPosition().X, halfSize, 90));
		moved = true;
	}

	if (moved && index != -1 && UsingSpacialPartitioning)
		pPartitionedSpace->UpdateAgentCell(*pAgent, OldPositions[index]);
}

void Flock::StartUsingSpacialPartitioning()
{
	for (ASteeringAgent* agent : Agents)
	{
		pPartitionedSpace->AddAgent(*agent);
		RegisterNeighbors(agent);
	}
}

void Flock::StopUsingSpacialPartitioning()
{
	pPartitionedSpace->EmptyCells();
	for (ASteeringAgent* agent : Agents)
	{
		RegisterNeighbors(agent);
	}
}

void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	if (UsingSpacialPartitioning)
		pPartitionedSpace->RegisterNeighbors(*pAgent, NeighborhoodRadius);
	else
	{
		NrOfNeighbors = 0;
		for (auto* agent : Agents)
		{
			if (pAgent != nullptr && pAgent != agent && agent != nullptr)
			{
				float distance = FVector2D::Distance(pAgent->GetPosition(), agent->GetPosition());
				if (distance <= NeighborhoodRadius)
				{
					Neighbors[NrOfNeighbors] = agent;
					++NrOfNeighbors;
				}
			}
		}
	}
}

int Flock::GetNrOfNeighbors() const
{
	if (UsingSpacialPartitioning)
		return pPartitionedSpace->GetNrOfNeighbors();
	else
		return NrOfNeighbors;
}

const TArray<ASteeringAgent*>& Flock::GetNeighbors() const
{
	if (UsingSpacialPartitioning)
		return pPartitionedSpace->GetNeighbors();
	else
		return Neighbors;
}

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D avgPosition = FVector2D::ZeroVector;
	if (GetNrOfNeighbors() > 0)
	{
		for (int i{}; i < GetNrOfNeighbors(); ++i)
		{
			avgPosition += GetNeighbors()[i]->GetPosition();
		}
		avgPosition /= GetNrOfNeighbors();
	}

	return avgPosition;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D avgVelocity = FVector2D::ZeroVector;

	if (GetNrOfNeighbors() > 0)
	{
		for (int i{}; i < GetNrOfNeighbors(); ++i)
		{
			avgVelocity += GetNeighbors()[i]->GetLinearVelocity();
		}
		avgVelocity /= GetNrOfNeighbors();
	}

	return avgVelocity;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	pSeekBehavior->SetTarget(Target);
}

