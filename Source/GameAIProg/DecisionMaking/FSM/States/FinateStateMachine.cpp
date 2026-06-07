#include "FinateStateMachine.h"
//#include "Runtime/AIModule/Classes/BehaviorTree/BlackboardComponent.h"

GameAI::FSM::FSM::FSM()
{
}

void GameAI::FSM::FSM::SetBlackboard(UBlackboardComponent* blackboard)
{
	if (blackboard)
		Blackboard = blackboard;
}

void GameAI::FSM::FSM::Update(float deltaTime)
{
	if (CurrentState)
		CurrentState->Update(Blackboard, deltaTime);

	for (auto& transition : m_Transitions[CurrentState])
	{
		State* newState = transition->EvaluateCondition(Blackboard);
		if (newState)
		{
			ChangeState(newState);
			break;
		}
	}
}

GameAI::FSM::State* GameAI::FSM::FSM::AddState(std::unique_ptr<State> state, bool currentState)
{
	m_pStates.push_back(std::move(state));

	if (currentState)
		CurrentState = m_pStates.back().get(); // set but do not start!

	return m_pStates.back().get();
}

void GameAI::FSM::FSM::AddTransition(State* from, std::unique_ptr<Transition>&& transition)
{
	m_Transitions[from].push_back(std::move(transition));
}

void GameAI::FSM::FSM::Start()
{
	if (CurrentState)
		CurrentState->Start(Blackboard); // start the current state
	else
		ChangeState(m_pStates.front().get()); // start the first state if current state is not set
}

void GameAI::FSM::FSM::ChangeState(State* newState)
{
	CurrentState = newState;
	CurrentState->Start(Blackboard);
}
