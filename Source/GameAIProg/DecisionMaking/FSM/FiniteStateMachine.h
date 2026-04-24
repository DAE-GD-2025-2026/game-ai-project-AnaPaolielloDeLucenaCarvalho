#pragma once
#include <vector>
#include <memory>
#include <functional>

namespace GameAI::FSM
{
	// Forward declaration - Blackboard
	class Blackboard; 

	class State
	{
	public:
		virtual ~State() = default;
		virtual void OnEnter() {}
		virtual void Update(float DeltaTime) = 0;
		virtual void OnExit() {}
	};

	class Transition
	{
	public:
		State* FromState;
		State* ToState;
		std::function<bool()> Condition;
	};

	class FSM
	{
	public:
		FSM() = default;
		~FSM() = default;

		void AddState(std::unique_ptr<State>&& NewState);
		void AddTransition(State* From, State* To, std::function<bool()> EvalFunc);
        
		void Start(); 
		void Update(float DeltaTime);

	private:
		std::vector<std::unique_ptr<State>> States;
		std::vector<Transition> Transitions;
		State* CurrentState = nullptr;
	};
}
