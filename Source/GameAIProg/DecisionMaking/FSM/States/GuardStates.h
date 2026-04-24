#pragma once
#include "../FiniteStateMachine.h"
#include "Blackboard.h"
#include "../../Movement/SteeringBehaviors/SteeringAgent.h"

namespace GameAI::FSM
{
	class PatrolState : public State
	{
	public:
		PatrolState(ASteeringAgent* InAgent, Blackboard* InBoard) 
			: Agent(InAgent)
			, Board(InBoard)
			, CurrentTargetIndex(0)
		{
		}

		void OnEnter() override { }
		void Update(float DeltaTime) override 
		{ 
			// get route from Blackboard
			TArray<FVector> Route = Board->GetData<TArray<FVector>>("PatrolRoute");
			
			if (Route.Num() == 0 || !Agent) return;

			// find target location
			FVector TargetLocation = Route[CurrentTargetIndex];
			FVector AgentLocation = Agent->GetActorLocation();

			// move the agent to target
			FVector Direction = (TargetLocation - AgentLocation).GetSafeNormal();
			Agent->SetActorLocation(AgentLocation + (Direction * 300.0f * DeltaTime)); 

			if (FVector::Distance(AgentLocation, TargetLocation) < 100.0f)
			{
				CurrentTargetIndex = (CurrentTargetIndex + 1) % Route.Num();
			}
		}
		void OnExit() override { }
		
	private:
		ASteeringAgent* Agent;
		Blackboard* Board;
		int CurrentTargetIndex;
	};

	class ChaseState : public State
	{
	public:
		ChaseState(ASteeringAgent* InAgent, Blackboard* InBoard) 
			: Agent(InAgent), Board(InBoard)
		{
		}
		
		void OnEnter() override { }
		void Update(float DeltaTime) override { }
		void OnExit() override { }
		
	private:
		ASteeringAgent* Agent;
		Blackboard* Board;
	};

	class SearchState : public State
	{
	public:
		SearchState(ASteeringAgent* InAgent, Blackboard* InBoard) 
			: Agent(InAgent), Board(InBoard)
		{
		}
		
		void OnEnter() override { }
		void Update(float DeltaTime) override { }
		void OnExit() override { }
		
	private:
		ASteeringAgent* Agent;
		Blackboard* Board;
	};
}