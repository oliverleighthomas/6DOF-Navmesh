// Fill out your copyright notice in the Description page of Project Settings.


#include "SixDOFNavmeshModifier.h"

// Sets default values
ASixDOFNavmeshModifier::ASixDOFNavmeshModifier()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

	USceneComponent* root = CreateDefaultSubobject<USceneComponent>(FName("Root"));
	root->Mobility = EComponentMobility::Static;
	RootComponent = root;

	navmeshModifierBounds = CreateDefaultSubobject<UBoxComponent>(FName("Bounds"));
	navmeshModifierBounds->AttachToComponent(root, FAttachmentTransformRules::KeepRelativeTransform);
	navmeshModifierBounds->SetCollisionEnabled(ECollisionEnabled::NoCollision);
}

// Called when the game starts or when spawned
void ASixDOFNavmeshModifier::BeginPlay()
{
	Super::BeginPlay();
	
}

