// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "InputMappingContext.h"
#include "UObject/Object.h"
#include "InputMappingContextPriority.generated.h"

/**
 * 
 */
USTRUCT(BlueprintType)
struct GAMEAIPROG_API FInputMappingContextPriority
{
	GENERATED_BODY()
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	UInputMappingContext* InputMappingContext{};
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int Priority{};
};
