#pragma once
#include "State.h"
#include "Transition.h"
#include <vector>
#include <memory>
#include <functional>
#include <unordered_map>

namespace GameAI
{
	namespace FSM
	{
		class FSM final
		{
		public:
			explicit FSM();
			~FSM() = default;

			void SetBlackboard(UBlackboardComponent* blackboard);

			void Update(float deltaTime);
			State* AddState(std::unique_ptr<State> state, bool currentState = false);
			void AddTransition(State* from, std::unique_ptr<Transition>&& transition);

			void Start();

		private:
			void ChangeState(State* newState);

			UBlackboardComponent* Blackboard = nullptr;
			State* CurrentState = nullptr;
			std::vector<std::unique_ptr<State>> m_pStates;

			//                 from
			std::unordered_map<State*, std::vector<std::unique_ptr<Transition>>> m_Transitions;
		};
	}
}