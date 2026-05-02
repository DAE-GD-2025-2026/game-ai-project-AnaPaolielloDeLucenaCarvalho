#include "BTTask_ClearValue.h"
#include "BehaviorTree/BlackboardComponent.h"

UBTTask_ClearValue::UBTTask_ClearValue()
{
	NodeName = "Clear BB Value (C++)";
}

EBTNodeResult::Type UBTTask_ClearValue::ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{	
	UBlackboardComponent* BB = OwnerComp.GetBlackboardComponent();
	if (!BB) return EBTNodeResult::Failed;

	// clear key
	BB->ClearValue(GetSelectedBlackboardKey());

	return EBTNodeResult::Succeeded;
}