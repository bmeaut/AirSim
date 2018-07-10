// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Engine/StaticMeshActor.h"
#include "RoadBeaconStaticMeshActor.generated.h"

UCLASS()
class AIRSIM_API ARoadBeaconStaticMeshActor : public AStaticMeshActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ARoadBeaconStaticMeshActor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	UPROPERTY(EditAnywhere, Category = "Connections")
	int32 ConnectionIdA;

	UPROPERTY(EditAnywhere, Category = "Connections")
	int32 ConnectionIdB;

	UPROPERTY(EditAnywhere, Category = "Connections")
	int32 ConnectionIdC;

	UPROPERTY(EditAnywhere, Category = "Connections")
	int32 ConnectionIdD;
	
};
