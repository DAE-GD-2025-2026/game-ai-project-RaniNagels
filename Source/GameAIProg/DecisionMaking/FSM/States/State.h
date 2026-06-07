#pragma once
#include "Runtime/AIModule/Classes/BehaviorTree/BlackboardComponent.h"
#include "../../../Movement/SteeringBehaviors/SteeringAgent.h"
#include <memory>

namespace GameAI
{
	namespace FSM
	{
		class State
		{
		public:
			explicit State() = default;
			virtual ~State() = default;
			
			virtual void Start(UBlackboardComponent* blackboard) {};
			virtual void Update(UBlackboardComponent* blackboard, float deltaTime) {};

		};

		class PatrolState final : public State
		{
		public:
			PatrolState() : State()
			{
				m_pSeek = std::make_unique<Seek>();
			}
			~PatrolState() = default;

			virtual void Start(UBlackboardComponent* blackboard) override
			{
				UObject* rawAgent = blackboard->GetValueAsObject("Agent");
				ASteeringAgent* agent = Cast<ASteeringAgent>(rawAgent);
				if (m_pSeek && agent)
					agent->SetSteeringBehavior(m_pSeek.get());
			}
			virtual void Update(UBlackboardComponent* blackboard, float deltaTime) override
			{
				FVector target = blackboard->GetValueAsVector("PatrolPoint");
				FTargetData data{};
				data.Position = FVector2D{ target.X, target.Y };
				m_pSeek->SetTarget(data);
			}

		private:
			std::unique_ptr<Seek> m_pSeek;
		};

		class SearchState final : public State
		{
		public:
			SearchState() : State()
			{
				m_pSeek = std::make_unique<Seek>();
				m_pWander = std::make_unique<Wander>();
			}
			~SearchState() = default;

			virtual void Start(UBlackboardComponent* blackboard) override
			{
				UObject* rawAgent = blackboard->GetValueAsObject("Agent");
				ASteeringAgent* agent = Cast<ASteeringAgent>(rawAgent);
				if (m_pWander && agent)
					agent->SetSteeringBehavior(m_pWander.get());
			}
			virtual void Update(UBlackboardComponent* blackboard, float override)
			{
				
			}

		private:
			std::unique_ptr<Seek> m_pSeek;
			std::unique_ptr<Wander> m_pWander;
		};
	}
}