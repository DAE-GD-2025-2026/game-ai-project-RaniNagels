// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_FSM.h"
#include "States/State.h"
#include "Perception/AIPerceptionComponent.h"
#include "FSMComponent.h"
#include "DecisionMaking/GameAIController.h"


// Sets default values
ALevel_FSM::ALevel_FSM()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_FSM::BeginPlay()
{
	Super::BeginPlay();

	TrimWorld->SetTrimWorldSize(2000.f);

	m_Seek = new Seek();
	m_Seek->SetTarget(MouseTarget);
	
	m_Thief = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, 
		FVector{0,0,90}, FRotator::ZeroRotator);
	m_Thief->SetDebugRenderingEnabled(false);
	m_Thief->SetSteeringBehavior(m_Seek);

	m_PatrolPoints.push_back(FVector2D{ 780, -730 });
	m_PatrolPoints.push_back(FVector2D{ 780, 1060 });
	m_PatrolPoints.push_back(FVector2D{ -590, 1060 });
	m_PatrolPoints.push_back(FVector2D{ -590, -730 });

	m_Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass,
		FVector{ m_PatrolPoints[m_CurrentPatrolPoint], 90 }, FRotator::ZeroRotator);
	m_Agent->SetDebugRenderingEnabled(false);
	
	if (AGameAIController* AIController = Cast<AGameAIController>(m_Thief->GetController()))
	{
		m_Blackboard = AIController->GetBlackboardComponent();
		auto perception = AIController->GetAIPerceptionComponent();
		if (!perception)
			UE_LOG(LogTemp, Warning, TEXT("Perception component NOT found!"));
		m_Blackboard->SetValueAsObject("Agent", m_Agent);
		m_Blackboard->SetValueAsVector("PatrolPoint", FVector{ m_PatrolPoints[m_CurrentPatrolPoint], 0 });

		// TODO: make keys in the editor!!!!!!!!!

		UpdateBlackboardValues(0.f);

		if (UFSMComponent* FSM = Cast<UFSMComponent>(AIController->GetBrainComponent()))
		{
			FSM->AddBlackboard(m_Blackboard);
			auto* Patrol = FSM->AddState(std::make_unique<GameAI::FSM::PatrolState>(), true);
			auto* Search = FSM->AddState(std::make_unique<GameAI::FSM::SearchState>(), false);
			FSM->AddTransition(Patrol, std::make_unique<GameAI::FSM::CounterTransition>(Search, 25.f));
			AIController->RunFiniteStateMachine();
		}
	}
	
}

void ALevel_FSM::UpdateBlackboardValues(float DeltaTime)
{
	UObject* rawAgent = m_Blackboard->GetValueAsObject("Agent");
	ASteeringAgent* agent = Cast<ASteeringAgent>(rawAgent);
	FVector target = m_Blackboard->GetValueAsVector("PatrolPoint");
	float distance = FVector2D::Distance(agent->GetPosition(), FVector2D{ target.X, target.Y });
	if ( distance < 80.f)
	{
		++m_CurrentPatrolPoint;
		m_CurrentPatrolPoint %= m_PatrolPoints.size();
		m_Blackboard->SetValueAsVector("PatrolPoint", FVector{ m_PatrolPoints[m_CurrentPatrolPoint], 0 });
	}

	m_Blackboard->SetValueAsVector("AgentTarget", FVector{ MouseTarget.Position.X, MouseTarget.Position.Y, 0 });
	float oldCounter = m_Blackboard->GetValueAsFloat("Counter");
	m_Blackboard->SetValueAsFloat("Counter", oldCounter + DeltaTime);
}

// Called every frame
void ALevel_FSM::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// update target
	m_Seek->SetTarget(MouseTarget);
	
	UpdateBlackboardValues(DeltaTime);
}

