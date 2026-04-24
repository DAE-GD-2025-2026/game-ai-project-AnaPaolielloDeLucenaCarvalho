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

			// move agent to target
			FVector Direction = (TargetLocation - AgentLocation).GetSafeNormal();
			FVector NewLocation = AgentLocation + (Direction * 300.0f * DeltaTime);

			Agent->SetActorLocation(NewLocation, true);

			if (FVector::Distance(AgentLocation, TargetLocation) < 200.0f)
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
		void Update(float DeltaTime) override 
		{ 
			ASteeringAgent* Thief = Board->GetData<ASteeringAgent*>("TargetAgent");
			if (!Agent || !Thief) return;

			// move to thief
			FVector TargetLoc = Thief->GetActorLocation();
			FVector ToTarget = TargetLoc - Agent->GetActorLocation();

			if (ToTarget.Length() > 100.0f) 
			{
				FVector Direction = ToTarget.GetSafeNormal();
				FVector NewLocation = Agent->GetActorLocation() + (Direction * 400.0f * DeltaTime);
            
				Agent->SetActorLocation(NewLocation, true); 
			}

			// save thief location - so that SearchState can use it
			Board->SetData("LastKnownPos", TargetLoc);
		}
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
       
		void OnEnter() override 
		{ 
			if (Agent) 
			{
				Board->SetData<double>("SearchStartTime", Agent->GetWorld()->GetTimeSeconds());
			}
		}
		void Update(float DeltaTime) override 
		{ 
			if (!Agent) return;

			FVector LastKnown = Board->GetData<FVector>("LastKnownPos");

			if (FVector::Distance(Agent->GetActorLocation(), LastKnown) > 50.0f)
			{
				FVector Direction = (LastKnown - Agent->GetActorLocation()).GetSafeNormal();
				FVector NewLocation = Agent->GetActorLocation() + (Direction * 300.0f * DeltaTime);
            
				Agent->SetActorLocation(NewLocation, true);
			}
		}
		void OnExit() override { }
       
	private:
		ASteeringAgent* Agent;
		Blackboard* Board;
	};
}