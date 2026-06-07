#pragma once
#include <functional>
#include "State.h"

namespace GameAI
{
	namespace FSM
	{
		class Transition
		{
		public:
			Transition(State* to)
				: m_To(to)
			{
			}
			virtual ~Transition() = default;

			State* EvaluateCondition(UBlackboardComponent* blackboard)
			{
				if (Condition(blackboard))
					return m_To;
				else
					return nullptr;
			}

		protected:
			virtual bool Condition(UBlackboardComponent* blackboard) = 0;

		private:
			State* m_To;
			UBlackboardComponent* m_Blackboard;
		};

		class CounterTransition : public Transition
		{
		public:
			CounterTransition(State* to, int maxCount)
				: Transition(to)
				, m_MaxCount(maxCount)
			{
			}

			virtual ~CounterTransition() = default;

		private:
			virtual bool Condition(UBlackboardComponent* blackboard) override
			{
				float count = blackboard->GetValueAsFloat("Counter");
				if (count >= m_MaxCount)
				{
					return true;
				}
				return false;
			}

			float m_MaxCount;
		};
	}
}