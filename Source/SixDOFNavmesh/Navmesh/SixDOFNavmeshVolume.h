// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/BoxComponent.h"
#include "PrioritiyQueue.h"
#include "SixDOFNavmeshModifier.h"
#include "SixDOFNavmeshWorker.h"
#include "SixDOFNavmeshVolume.generated.h"

UENUM()
enum class ENavigabilityStatus : uint8
{
	Navigable,
	NonNavigable,
	HasChildren
};

UENUM()
enum class EPathfindingTaskStatus : uint8
{
	NotStarted,
	InProgress,
	TimedOut,
	Successful,
	Failed
};

USTRUCT()
struct FOctant
{
	GENERATED_USTRUCT_BODY();

public:
	FOctant() {};

	int32 id;

	TArray<FOctant> children;
	FOctant* parent;

	FVector center;
	FVector extent;

	int32 xIndex;
	int32 yIndex;
	int32 zIndex;

	int32 level = 0;
	int32 index = 0;
	float cost = 1.f;

	ENavigabilityStatus navigatable = ENavigabilityStatus::Navigable;

	void DrawDebug(UWorld* world);
	void Reset();
};

USTRUCT()
struct FPathfindingTask {
	GENERATED_USTRUCT_BODY();

	AActor* actor;

	FVector origin;
	FVector destination;

	FOctant* originOctant;
	FOctant* destinationOctant;

	PrioritiyQueue<FOctant*> open;
	TMap<FOctant*, FOctant*> closed;

	TArray<FVector> path;

	EPathfindingTaskStatus status = EPathfindingTaskStatus::NotStarted;
	float timeTaken;

	FPathfindingTask() {}
	FPathfindingTask(AActor* actor, FVector origin, FVector destination, FOctant* originOctant, FOctant* destinationOctant) :
		actor(actor), origin(origin), destination(destination), originOctant(originOctant), destinationOctant(destinationOctant)
	{
	}

	bool operator==(const FPathfindingTask& other) {
		return other.actor == actor && other.origin == origin && other.destination == destination;
	}
};

UCLASS()
class SIXDOFNAVMESH_API ASixDOFNavmeshVolume : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ASixDOFNavmeshVolume();

private:
	UBoxComponent* navmeshVolumeBounds;
	FCollisionQueryParams octantCollisionQueryParams;
	FCollisionObjectQueryParams octantCollisionObjectQueryParams;

	TArray<FOctant*> dynamicCollisionListeners;

	void GenerateVoxelGrid();
	void SubdivideOctree(FOctant& octant);
	bool CheckOctantCollision(FOctant& voxel);

	FOctant* FindOctantAtIndex(int32 x, int32 y, int32 z, int32 level);
	FOctant* FindOctantAtLocation(FVector location);
	FOctant* FindOctantWithinChildren(FVector location, TArray<FOctant>& children);
	TArray<FOctant*> FindOctantsAroundMesh(UPrimitiveComponent* mesh);

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:	
	virtual void Tick(float DeltaTime) override;

	TArray<TArray<TArray<FOctant>>> octants;
	TArray<ASixDOFNavmeshModifier*> modifiers;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SetUp")
		float octantSize = 500.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SetUp")
		int32 maxSubdivisionLevel = 5;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optimization")
		float percentUntilConsideredFull = 80.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optimization")
		int32 maxPathfindingTasksPerTick = 200;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optimization")
		float queryTimeOutLimit = 5.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision")
		TArray<TEnumAsByte<ECollisionChannel>> octantCollisionChannels;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collision")
		TArray<AActor*> ignoredActors;

	UFUNCTION(BlueprintCallable)
		void TempFindOctant(FVector location);

	UFUNCTION(BlueprintCallable)
		void DrawDebugNavmesh();
	UFUNCTION(BlueprintCallable)
		void DrawDebugAroundMesh(UPrimitiveComponent* mesh);

	UFUNCTION(BlueprintCallable)
		bool SchedulePathfindingTask(AActor* actor, FVector destination);

	void TickDynamicCollisionUpdates();
	void TickPathfindingUpdates(float deltaTime, int32 maxNumOfTasks);

private:
	int32 numOfOccupiedOctans = 0;
	int32 volumeXSize;
	int32 volumeYSize;
	int32 volumeZSize;

	SixDOFNavmeshWorker* worker;


	TQueue<FPathfindingTask> newPathfindingTasks;
	TArray<FPathfindingTask> activePathfindingTasks;

	TArray<FOctant*> FindNeighbors(FOctant* octant);
	void GetNeighbors(FOctant* octant, TArray<FOctant*>& neighbors);
	void AddNeighborChildren(FOctant* neighbor, TArray<int32> indices, TArray<FOctant*>& neighbors);

	void CalculatePath(FPathfindingTask& task);
	void CompletePathfindingTask(int32 index);
};
