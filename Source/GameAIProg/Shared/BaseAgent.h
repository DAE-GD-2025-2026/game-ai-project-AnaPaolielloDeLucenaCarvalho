// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/CapsuleComponent.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "BaseAgent.generated.h"

/*
 * Provides common API for editing agent movement characteristics which are almost all held by
 * the character's CharacterMovementComponent. Feel free to directly use its API instead :)
 *
 * All Game AI character will inherit from this class.
 */

UCLASS()
class GAMEAIPROG_API ABaseAgent : public ACharacter
{
	GENERATED_BODY()

public:
	// Sets default values for this character's properties
	ABaseAgent();

protected:
	bool bIsDebugRenderingEnabled{true};
	
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;
	
	// BaseAgent Interface
	FVector2D GetPosition() const { return FVector2D{GetActorLocation().X, GetActorLocation().Y}; }
	void SetPosition(FVector2D const & NewPosition) { SetActorLocation(FVector{NewPosition, GetActorLocation().Z}); }
	float GetRotation() const { return GetActorRotation().Yaw; }
	
	float GetMaxLinearSpeed() const { return GetCharacterMovement()->GetMaxSpeed(); }
	void SetMaxLinearSpeed(float MaxSpeed) { GetCharacterMovement()->MaxWalkSpeed = MaxSpeed; }

	FVector2D GetLinearVelocity() const { return FVector2D{GetCharacterMovement()->Velocity}; }

	float GetMaxAngularSpeed() const { return GetCharacterMovement()->RotationRate.Yaw; }
	void SetMaxAngularSpeed(float maxAngularSpeed) { GetCharacterMovement()->RotationRate.Yaw = maxAngularSpeed; }

	float GetAngularVelocity() const { return GetCharacterMovement()->GetLastUpdateRotation().Yaw - GetActorRotation().Yaw; }

	bool IsAutoOrienting() const { return GetCharacterMovement()->bOrientRotationToMovement; }
	void SetIsAutoOrienting(bool bAutoOrient) { GetCharacterMovement()->bOrientRotationToMovement = bAutoOrient; }

	float GetMass() const { return GetCharacterMovement()->Mass; }
	void SetMass(float Mass) { GetCharacterMovement()->Mass = Mass; }

	bool GetDebugRenderingEnabled() const { return bIsDebugRenderingEnabled; }
	void SetDebugRenderingEnabled(bool IsEnabled) { this->bIsDebugRenderingEnabled = IsEnabled; }
	
	float GetCapsuleRadius() const { return GetCapsuleComponent()->GetScaledCapsuleRadius(); }
};
