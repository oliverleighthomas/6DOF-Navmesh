// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/BoxComponent.h"
#include "SixDOFNavmeshModifier.generated.h"

UCLASS(Blueprintable)
class SIXDOFNAVMESH_API ASixDOFNavmeshModifier : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ASixDOFNavmeshModifier();

protected:
	virtual void BeginPlay() override;

public:	
	UBoxComponent* navmeshModifierBounds;

	UPROPERTY(EditAnywhere, Category = "Modifier Values")
		float costModifier = 2.f;
};
