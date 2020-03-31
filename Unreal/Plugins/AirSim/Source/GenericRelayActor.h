// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GenericRelayActor.generated.h"

UCLASS()
class AIRSIM_API AGenericRelayActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AGenericRelayActor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	UFUNCTION(BlueprintImplementableEvent)
	void SwitchDayLightState(bool enabled);

	UFUNCTION(BlueprintImplementableEvent)
	void SwitchAutoPilotMod(bool enabled);

	UFUNCTION(BlueprintImplementableEvent)
	void ResetEvent();

	UFUNCTION(BlueprintImplementableEvent)
	void SwitchFogMod(bool enabled);

	UFUNCTION(BlueprintImplementableEvent)
	void SetDirection(bool forward);

	UFUNCTION(BlueprintImplementableEvent)
	void SetTargetSpeed(float targetSpeed, float deltaSeconds);

};