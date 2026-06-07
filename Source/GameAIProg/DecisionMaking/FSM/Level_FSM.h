// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Shared/Level_Base.h"
#include "Runtime/AIModule/Classes/BehaviorTree/BlackboardComponent.h"
#include "Level_FSM.generated.h"

UCLASS()
class GAMEAIPROG_API ALevel_FSM : public ALevel_Base
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	ALevel_FSM();

	// Called every frame
	virtual void Tick(float DeltaTime) override;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

private:
	UPROPERTY()
	ASteeringAgent* m_Thief{ nullptr }; // ref
	Seek* m_Seek{ nullptr };
	std::vector<FVector2D> m_PatrolPoints{};
	int m_CurrentPatrolPoint{ 0 };

	ASteeringAgent* m_Agent{ nullptr };

	UBlackboardComponent* m_Blackboard{ nullptr };

	void UpdateBlackboardValues(float DeltaTime);
};
