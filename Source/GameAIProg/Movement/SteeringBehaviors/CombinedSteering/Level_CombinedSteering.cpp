#include "Level_CombinedSteering.h"

#include "imgui.h"
#include <string>


// Sets default values
ALevel_CombinedSteering::ALevel_CombinedSteering()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_CombinedSteering::BeginPlay()
{
	Super::BeginPlay();

	// SteeringBehaviors
	m_pWander = std::make_unique<Wander>();
	m_pSeek = std::make_unique<Seek>();
	m_pEvade = std::make_unique<Evade>();

	// BlendedSteering
	std::vector<BlendedSteering::WeightedBehavior> weightedBehaviors{};
	weightedBehaviors.push_back({ m_pSeek.get(), 0.5f });
	weightedBehaviors.push_back({ m_pWander.get(), 0.5f });
	m_pBlendedSteering = std::make_unique<BlendedSteering>(weightedBehaviors);

	m_pBlendedSteeringAgent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{ 0,0,90 }, FRotator::ZeroRotator);
	m_pBlendedSteeringAgent->SetSteeringBehavior(m_pBlendedSteering.get());
	m_pBlendedSteeringAgent->SetDebugRenderingEnabled(false);

	// PrioritySteering
	std::vector<ISteeringBehavior*> steeringBehaviors{};
	steeringBehaviors.push_back(m_pEvade.get());
	steeringBehaviors.push_back(m_pWander.get());
	m_pPrioritySteering = std::make_unique<PrioritySteering>(steeringBehaviors);

	m_pPrioritySteeringAgent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{ 0,0,90 }, FRotator::ZeroRotator);
	m_pPrioritySteeringAgent->SetSteeringBehavior(m_pPrioritySteering.get());
	m_pPrioritySteeringAgent->SetDebugRenderingEnabled(false);
}

void ALevel_CombinedSteering::BeginDestroy()
{
	Super::BeginDestroy();

}

// Called every frame
void ALevel_CombinedSteering::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
#pragma region UI
	//UI
	{
		//Setup
		bool windowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Game AI", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
	
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
		ImGui::Spacing();
	
		ImGui::Text("Flocking");
		ImGui::Spacing();
		ImGui::Spacing();
	
		if (ImGui::Checkbox("Debug Rendering", &CanDebugRender))
		{
			m_pBlendedSteeringAgent->SetDebugRenderingEnabled(CanDebugRender);
			m_pPrioritySteeringAgent->SetDebugRenderingEnabled(CanDebugRender);
		}
		ImGui::Checkbox("Trim World", &TrimWorld->bShouldTrimWorld);
		if (TrimWorld->bShouldTrimWorld)
		{
			ImGuiHelpers::ImGuiSliderFloatWithSetter("Trim Size",
				TrimWorld->GetTrimWorldSize(), 1000.f, 3000.f,
				[this](float InVal) { TrimWorld->SetTrimWorldSize(InVal); });
		}
		
		ImGui::Spacing();
		ImGui::Spacing();
		ImGui::Spacing();
	
		ImGui::Text("Behavior Weights");
		ImGui::Spacing();


		ImGuiHelpers::ImGuiSliderFloatWithSetter("Seek",
			m_pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f,
			[this](float InVal) { m_pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight = InVal; }, "%.2f");
		
		ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander",
			m_pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight, 0.f, 1.f,
			[this](float InVal) { m_pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight = InVal; }, "%.2f");

		float w1 = m_pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight;
		float w2 = m_pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight;
		float total = w1 + w2;
		float rw1 = w1 / total;
		float rw2 = w2 / total;
		std::string seekOut		= "Seek:   ";
		std::string wanderOut	= "Wander: ";
		seekOut += std::to_string(rw1);
		wanderOut += std::to_string(rw2);;

		ImGui::Spacing();
		ImGui::Text("Actual Weights:");
		ImGui::Text(seekOut.c_str());
		ImGui::Text(wanderOut.c_str());
	
		//End
		ImGui::End();
	}
#pragma endregion
	
	// Combined Steering Update
	m_pSeek->SetTarget(MouseTarget);

	FTargetData EvadeTargetData{};
	EvadeTargetData.Position = m_pBlendedSteeringAgent->GetPosition();
	EvadeTargetData.Orientation = m_pBlendedSteeringAgent->GetRotation();
	EvadeTargetData.LinearVelocity = m_pBlendedSteeringAgent->GetLinearVelocity();
	EvadeTargetData.AngularVelocity = m_pBlendedSteeringAgent->GetAngularVelocity();

	m_pEvade->SetTarget(EvadeTargetData);
}
