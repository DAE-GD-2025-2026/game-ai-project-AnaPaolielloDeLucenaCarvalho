#include "BTTask_NextPatrolPoint.h"
#include "DecisionMaking/GameAIController.h"
#include "BehaviorTree/BlackboardComponent.h"

UBTTask_NextPatrolPoint::UBTTask_NextPatrolPoint() 
{ 
	NodeName = "Get Next Patrol Point"; 
}

EBTNodeResult::Type UBTTask_NextPatrolPoint::ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
	AGameAIController* AICon = Cast<AGameAIController>(OwnerComp.GetAIOwner());
	UBlackboardComponent* BB = OwnerComp.GetBlackboardComponent();
	
	if (!AICon || !BB || AICon->PatrolRoute.Num() == 0) return EBTNodeResult::Failed;

	FVector Point = AICon->PatrolRoute[AICon->CurrentPatrolIndex];

	BB->SetValueAsVector(GetSelectedBlackboardKey(), Point);

	AICon->CurrentPatrolIndex = (AICon->CurrentPatrolIndex + 1) % AICon->PatrolRoute.Num();

	return EBTNodeResult::Succeeded;
}