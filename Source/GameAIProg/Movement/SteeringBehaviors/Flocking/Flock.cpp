#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
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

	// initialize the flock and the memory pool
	Agents.SetNum(FlockSize);
	for (int i{}; i < FlockSize; ++i)
	{
		// spawn the agents in random locations, Z should always be 90
		float RandX = FMath::FRandRange(-WorldSize * 0.5f, WorldSize * 0.5f);
		float RandY = FMath::FRandRange(-WorldSize * 0.5f, WorldSize * 0.5f);

		FVector SpanwLocation = FVector(RandX, RandY, 90);
		Agents[i] = pWorld->SpawnActor<ASteeringAgent>(AgentClass, SpanwLocation, FRotator::ZeroRotator);

		if (Agents[i] == nullptr)
		{
			UE_LOG(LogTemp, Warning, TEXT("Failed to spawn agent %d"), i);
			continue;
		}
		Agents[i]->SetSteeringBehavior(pPrioritySteering.get());
		Agents[i]->SetDebugRenderingEnabled(false);
	}

	Neighbors.SetNum(FlockSize - 1);
}

Flock::~Flock()
{
 // Cleanup any additional data
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

	for (ASteeringAgent* agent : Agents)
	{
		if (agent == nullptr) continue;
		RegisterNeighbors(agent);
		agent->Tick(DeltaTime);
		TrimToWorld(agent);
	}
	TrimToWorld(pAgentToEvade);
}

void Flock::RenderDebug()
{
	// Render trim world
	float halfSize = WorldSize / 2;
	FVector p1{ halfSize, -halfSize, 0 };
	FVector p2{ halfSize, halfSize, 0 };
	FVector p3{ -halfSize, halfSize, 0 };
	FVector p4{ -halfSize, -halfSize, 0 };
	DrawDebugLine(pWorld, p1, p2, FColor::Red);
	DrawDebugLine(pWorld, p2, p3, FColor::Red);
	DrawDebugLine(pWorld, p3, p4, FColor::Red);
	DrawDebugLine(pWorld, p4, p1, FColor::Red);

	// Render the debug first agent in the flock! and only the debug as well as its neighborhood
	for (size_t i{}; i < NrOfNeighbors; ++i)
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
		DrawDebugCircle(pWorld, FVector(pAgent->GetPosition(), 90), NeighborhoodRadius, 32, FColor::Red, false, -1.f, 0, 10.f, FVector(1.f, 0.f, 0.f), FVector(0.f, 1.f, 0.f), false);
		RegisterNeighbors(pAgent);
		for (int i{}; i < NrOfNeighbors; ++i)
		{
			DrawDebugSphere(pWorld, FVector{ Neighbors[i]->GetPosition(), 90 }, 20, 16, FColor::Emerald);
		}
	}
}

void Flock::TrimToWorld(ASteeringAgent* const pAgent)
{
	float halfSize = WorldSize / 2;
	if (pAgent->GetPosition().X > halfSize)
	{
		pAgent->SetActorLocation(FVector(-halfSize, pAgent->GetPosition().Y, 90));
	}
	else if (pAgent->GetPosition().X < -halfSize)
	{
		pAgent->SetActorLocation(FVector(halfSize, pAgent->GetPosition().Y, 90));
	}

	if (pAgent->GetPosition().Y > halfSize)
	{
		pAgent->SetActorLocation(FVector(pAgent->GetPosition().X, -halfSize, 90));
	}
	else if (pAgent->GetPosition().Y < -halfSize)
	{
		pAgent->SetActorLocation(FVector(pAgent->GetPosition().X, halfSize, 90));
	}
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
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
#endif

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D avgPosition = FVector2D::ZeroVector;

	if (NrOfNeighbors > 0)
	{
		for (int i{}; i < NrOfNeighbors; ++i)
		{
			avgPosition += Neighbors[i]->GetPosition();
		}
		avgPosition /= NrOfNeighbors;
	}
	
	return avgPosition;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D avgVelocity = FVector2D::ZeroVector;

	if (NrOfNeighbors > 0)
	{
		for (int i{}; i < NrOfNeighbors; ++i)
		{
			avgVelocity += Neighbors[i]->GetLinearVelocity();
		}
		avgVelocity /= NrOfNeighbors;
	}

	return avgVelocity;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	pSeekBehavior->SetTarget(Target);
}

