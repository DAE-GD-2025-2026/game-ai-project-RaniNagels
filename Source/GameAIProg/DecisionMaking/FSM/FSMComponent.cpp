// Fill out your copyright notice in the Description page of Project Settings.


#include "FSMComponent.h"


// Sets default values for this component's properties
UFSMComponent::UFSMComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// TODO Setup FSM
	FSMInstance = std::make_unique<GameAI::FSM::FSM>();
}


GameAI::FSM::State* UFSMComponent::AddState(std::unique_ptr<GameAI::FSM::State>&& NewState, bool currentState)
{
	return FSMInstance->AddState(std::move(NewState), currentState);
}

void UFSMComponent::AddTransition(GameAI::FSM::State* From, std::unique_ptr<GameAI::FSM::Transition>&& transition)
{
	FSMInstance->AddTransition(From, std::move(transition));
}

// Called when the game starts
void UFSMComponent::BeginPlay()
{
	Super::BeginPlay();
}


// Called every frame
void UFSMComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	FSMInstance->Update(DeltaTime);
}

void UFSMComponent::AddBlackboard(UBlackboardComponent* blackboard)
{
	if (blackboard != nullptr)
		FSMInstance->SetBlackboard(blackboard);
}

void UFSMComponent::StartLogic()
{
	Super::StartLogic();

	FSMInstance->Start();
}

void UFSMComponent::StopLogic(const FString& Reason)
{
	Super::StopLogic(Reason);
	// TODO
}

bool UFSMComponent::IsRunning() const
{
	return bIsRunning;
}

