#include "FiniteStateMachine.h"

namespace GameAI::FSM
{
	void FSM::AddState(std::unique_ptr<State>&& NewState)
	{
		States.push_back(std::move(NewState));
	}

	void FSM::AddTransition(State* From, State* To, std::function<bool()> EvalFunc)
	{
		Transitions.push_back({From, To, EvalFunc});
	}

	void FSM::Start()
	{
		if (!States.empty() && CurrentState == nullptr)
		{
			CurrentState = States[0].get();
			CurrentState->OnEnter();
		}
	}

	void FSM::Update(float DeltaTime)
	{
		if (CurrentState == nullptr) return;

		for (const auto& Transition : Transitions)
		{
			if (Transition.FromState == CurrentState || Transition.FromState == nullptr)
			{
				if (Transition.Condition())
				{
					CurrentState->OnExit();
					CurrentState = Transition.ToState;
					CurrentState->OnEnter();
					break;
				}
			}
		}

		if (CurrentState)
		{
			CurrentState->Update(DeltaTime);
		}
	}
}