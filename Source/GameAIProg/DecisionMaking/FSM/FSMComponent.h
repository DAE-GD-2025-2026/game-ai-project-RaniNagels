// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <functional>
#include <memory>
#include "States/FinateStateMachine.h"
#include "CoreMinimal.h"
#include "BrainComponent.h"
#include "FSMComponent.generated.h"

//namespace GameAI::FSM
//{
//	class State;
//	class Transition;
//	class FSM; // contains FSM logic
//}

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class GAMEAIPROG_API UFSMComponent : public UBrainComponent
{
	GENERATED_BODY()

public:
	// Sets default values for this component's properties
	UFSMComponent();

	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
	                           FActorComponentTickFunction* ThisTickFunction) override;

	void AddBlackboard(UBlackboardComponent* blackboard);
	
	virtual void StartLogic() override;
	virtual void StopLogic(const FString& Reason) override;
	
	virtual bool IsRunning() const override; 
	
	GameAI::FSM::State* AddState(std::unique_ptr<GameAI::FSM::State>&& NewState, bool currentState = false);
	void AddTransition(GameAI::FSM::State* From, std::unique_ptr<GameAI::FSM::Transition>&& transition);
		
protected:
	// Called when the game starts
	virtual void BeginPlay() override;

private:
	std::unique_ptr<GameAI::FSM::FSM> FSMInstance;
	bool bIsRunning{false};
};
