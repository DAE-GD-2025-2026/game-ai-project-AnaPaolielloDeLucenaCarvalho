#pragma once

#include "CoreMinimal.h"
#include "BehaviorTree/Tasks/BTTask_BlackboardBase.h"
#include "BTTask_NextPatrolPoint.generated.h"

UCLASS()
class GAMEAIPROG_API UBTTask_NextPatrolPoint : public UBTTask_BlackboardBase
{
	GENERATED_BODY()
public:
	UBTTask_NextPatrolPoint();
	virtual EBTNodeResult::Type ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;
};