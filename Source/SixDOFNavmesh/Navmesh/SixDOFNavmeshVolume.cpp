// Fill out your copyright notice in the Description page of Project Settings.


#include "SixDOFNavmeshVolume.h"
#include "DrawDebugHelpers.h"
#include "PrioritiyQueue.h"
#include "Algo/Reverse.h"
#include "Math/UnrealMathUtility.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetMathLibrary.h"

void FOctant::DrawDebug(UWorld* world) {
	if (navigatable != ENavigabilityStatus::HasChildren) {
		FColor color;
		uint8 depth;
		navigatable == ENavigabilityStatus::Navigable ? color = FColor::Green : color = FColor::Red;
		navigatable == ENavigabilityStatus::Navigable ? depth = 0U : depth = 1U;
		DrawDebugBox(world, center, extent, color, true, -1.f, depth, 2.0f);
	}
	else {
		for (auto child : children) {
			child.DrawDebug(world);
		}
	}
}

void FOctant::Reset() {
	children.Empty();
	level = 0;
	cost = 1.f;
	navigatable = ENavigabilityStatus::Navigable;
}

ASixDOFNavmeshVolume::ASixDOFNavmeshVolume()
{
	PrimaryActorTick.bCanEverTick = true;

	USceneComponent* root = CreateDefaultSubobject<USceneComponent>(FName("Root"));
	root->Mobility = EComponentMobility::Static;
	RootComponent = root;

	navmeshVolumeBounds = CreateDefaultSubobject<UBoxComponent>(FName("Bounds"));
	navmeshVolumeBounds->AttachToComponent(root, FAttachmentTransformRules::KeepRelativeTransform);
	navmeshVolumeBounds->SetCollisionEnabled(ECollisionEnabled::NoCollision);

	FVector extent = navmeshVolumeBounds->GetScaledBoxExtent();
	FTransform transform = navmeshVolumeBounds->GetComponentTransform();
	FVector center = transform.TransformPosition(extent);
	navmeshVolumeBounds->SetRelativeLocation(center);

	octantCollisionQueryParams = FCollisionQueryParams(FName("6DOFCollisionQuery"));

	for (auto channel : octantCollisionChannels) {
		octantCollisionObjectQueryParams.AddObjectTypesToQuery(channel);
	}
}

// Called when the game starts or when spawned
void ASixDOFNavmeshVolume::BeginPlay()
{
	Super::BeginPlay();

	worker = new SixDOFNavmeshWorker(this);

	octantCollisionQueryParams.AddIgnoredActors(ignoredActors);

	GenerateVoxelGrid();
}

void ASixDOFNavmeshVolume::EndPlay(const EEndPlayReason::Type EndPlayReason) {
	Super::EndPlay(EndPlayReason);

	worker->Stop();
	delete worker;
}

void ASixDOFNavmeshVolume::TempFindOctant(FVector location) {
	FOctant* octant = FindOctantAtLocation(location);
	if (octant) {
		octant->DrawDebug(GetWorld());
		TArray<FOctant*> neighbors;
		GetNeighbors(octant, neighbors);
		for (auto neighbor : neighbors) {
			neighbor->DrawDebug(GetWorld());
		}
	}
	else UE_LOG(LogTemp, Warning, TEXT("OCTANT NOT FOUND"));
}

void ASixDOFNavmeshVolume::Tick(float DeltaTime) {
	Super::Tick(DeltaTime);

	//UKismetSystemLibrary::FlushPersistentDebugLines(GetWorld());
	//DrawDebugNavmesh();
}

void ASixDOFNavmeshVolume::TickDynamicCollisionUpdates() {
	for (auto listener : dynamicCollisionListeners) {
		listener->Reset();
		SubdivideOctree(*listener);
	}
}

void ASixDOFNavmeshVolume::TickPathfindingUpdates(float deltaTime, int32 maxNumOfTasks) {
	int32 numOfTasks = activePathfindingTasks.Num();
	if (maxNumOfTasks > numOfTasks) maxNumOfTasks = numOfTasks;

	for (int32 i = 0; i < maxNumOfTasks; ++i) {
		auto& task = activePathfindingTasks[i];

		if (task.timeTaken > queryTimeOutLimit) {
			task.status = EPathfindingTaskStatus::TimedOut;
			UE_LOG(LogTemp, Warning, TEXT("Pathfinding timed out!"));
			CompletePathfindingTask(i);
			continue;
		}

		CalculatePath(task);

		if (task.status == EPathfindingTaskStatus::Failed) {
			UE_LOG(LogTemp, Warning, TEXT("No path was found."));
			CompletePathfindingTask(i);
			continue;
		}

		if (task.status == EPathfindingTaskStatus::Successful) {

			UE_LOG(LogTemp, Warning, TEXT("Path found!"));
			FOctant* prev = task.destinationOctant;
			while (prev->id != task.originOctant->id) {
				FOctant* next = task.closed[prev];
				task.path.Add(next->center);
				prev = next;
			}

			Algo::Reverse(task.path);
			CompletePathfindingTask(i);
			continue;
		}

		task.timeTaken += deltaTime;
	}
}


void ASixDOFNavmeshVolume::CompletePathfindingTask(int32 index) {
	activePathfindingTasks.RemoveAtSwap(index);
}

void ASixDOFNavmeshVolume::CalculatePath(FPathfindingTask& task) {
	FOctant* curr = task.open.Top();
	if (!curr) {
		task.status = EPathfindingTaskStatus::Failed;
		return;
	}

	FOctant* destination = task.destinationOctant;
	if (task.closed.Contains(destination)) {
		task.status = EPathfindingTaskStatus::Successful;
		return;
	}

	task.open.Pop();

	TArray<FOctant*> neighbors;
	GetNeighbors(curr, neighbors);

	for (auto neighbor : neighbors) {
		if (neighbor->navigatable != ENavigabilityStatus::Navigable || task.closed.Contains(neighbor)) continue;
		task.closed.Add(neighbor, curr);
		float cost = FVector::Dist(neighbor->center, task.destination) + FVector::Dist(curr->center, neighbor->center);
		task.open.Push(neighbor, cost);
	}
}

void ASixDOFNavmeshVolume::GetNeighbors(FOctant* octant, TArray<FOctant*>& neighbors) {
	FVector centerLocation = octant->center;
	float offset = octant->extent.X + 1;
	FOctant* neighbor;

	FVector left = FVector(centerLocation.X - offset, centerLocation.Y, centerLocation.Z);
	neighbor = FindOctantAtLocation(left);
	if (neighbor) {
		if (octant->level < neighbor->level) AddNeighborChildren(neighbor, TArray<int32>{0, 1, 4, 5}, neighbors);
		else neighbors.Emplace(neighbor);
	}

	FVector right = FVector(centerLocation.X + offset, centerLocation.Y, centerLocation.Z);
	neighbor = FindOctantAtLocation(right);
	if (neighbor) {
		if (octant->level < neighbor->level) AddNeighborChildren(neighbor, TArray<int32>{2, 3, 6, 7}, neighbors);
		else neighbors.Emplace(neighbor);
	}

	FVector back = FVector(centerLocation.X, centerLocation.Y - offset, centerLocation.Z);
	neighbor = FindOctantAtLocation(back);
	if (neighbor) {
		if (octant->level < neighbor->level) AddNeighborChildren(neighbor, TArray<int32>{1, 3, 5, 7}, neighbors);
		else neighbors.Emplace(neighbor);
	}

	FVector front = FVector(centerLocation.X, centerLocation.Y + offset, centerLocation.Z);
	neighbor = FindOctantAtLocation(front);
	if (neighbor) {
		if (octant->level < neighbor->level) AddNeighborChildren(neighbor, TArray<int32>{0, 2, 4, 6}, neighbors);
		else neighbors.Emplace(neighbor);
	}

	FVector bottom = FVector(centerLocation.X, centerLocation.Y, centerLocation.Z - offset);
	neighbor = FindOctantAtLocation(bottom);
	if (neighbor) {
		if (octant->level < neighbor->level) AddNeighborChildren(neighbor, TArray<int32>{4, 5, 6, 7}, neighbors);
		else neighbors.Emplace(neighbor);
	}

	FVector top = FVector(centerLocation.X, centerLocation.Y, centerLocation.Z + offset);
	neighbor = FindOctantAtLocation(top);
	if (neighbor) {
		if (octant->level < neighbor->level) AddNeighborChildren(neighbor, TArray<int32>{0, 1, 2, 3}, neighbors);
		else neighbors.Emplace(neighbor);
	}
}

void ASixDOFNavmeshVolume::AddNeighborChildren(FOctant* neighbor, TArray<int32> indices, TArray<FOctant*>& neighbors) {
	TArray<FOctant> children = neighbor->children;
	if (children.Num() == 0) {
		neighbors.Emplace(neighbor);
		return;
	}
	else {
		for (int i = 0; i < indices.Num(); ++i) {
			AddNeighborChildren(&children[indices[i]], indices, neighbors);
		}
	}
}


bool ASixDOFNavmeshVolume::SchedulePathfindingTask(AActor* actor, FVector destination) {
	FOctant* destinationOctant = FindOctantAtLocation(destination);
	if (!destinationOctant) {
		UE_LOG(LogTemp, Warning, TEXT("Destination is out-of-bounds."));
		return false;
	}

	FOctant* originOctant = FindOctantAtLocation(actor->GetActorLocation());
	if (!originOctant) {
		UE_LOG(LogTemp, Warning, TEXT("Origin is out-of-bounds."));
		return false;
	}

	FPathfindingTask task(actor, actor->GetActorLocation(), destination, originOctant, destinationOctant);
	task.open.Push(originOctant, originOctant->cost);

	UE_LOG(LogTemp, Warning, TEXT("Task scheduled!"));
	activePathfindingTasks.Add(task);
	return true;
}



TArray<FOctant*> ASixDOFNavmeshVolume::FindNeighbors(FOctant* octant) {
	TArray<FOctant*> neighbors;

	int32 left = octant->xIndex - 1;
	int32 right = octant->xIndex + 1;

	int32 back = octant->yIndex - 1;
	int32 front = octant->yIndex + 1;

	int32 bottom = octant->zIndex - 1;
	int32 top = octant->zIndex + 1;

	if (left >= 0) neighbors.Emplace(&octants[left][octant->yIndex][octant->zIndex]);
	if (right < octants.Num()) neighbors.Emplace(&octants[right][octant->yIndex][octant->zIndex]);

	if (back >= 0) neighbors.Emplace(&octants[octant->xIndex][back][octant->zIndex]);
	if (front < octants[octant->xIndex].Num()) neighbors.Emplace(&octants[octant->xIndex][front][octant->zIndex]);

	if (bottom >= 0) neighbors.Emplace(&octants[octant->xIndex][octant->yIndex][bottom]);
	if (top < octants[octant->xIndex][octant->yIndex].Num()) neighbors.Emplace(&octants[octant->xIndex][octant->yIndex][top]);

	return neighbors;
}


FOctant* ASixDOFNavmeshVolume::FindOctantAtIndex(int32 x, int32 y, int32 z, int32 level) {
	return nullptr;
}

FOctant* ASixDOFNavmeshVolume::FindOctantAtLocation(FVector location) {
	int32 x = (location.X - GetActorLocation().X) / octantSize;
	int32 y = (location.Y - GetActorLocation().Y) / octantSize;
	int32 z = (location.Z - GetActorLocation().Z) / octantSize;

	FOctant* octant = (octants.IsValidIndex(x) && octants[x].IsValidIndex(y) && octants[x][y].IsValidIndex(z)) ? &octants[x][y][z] : nullptr;
	if (!octant) return nullptr;

	if (octant->navigatable == ENavigabilityStatus::HasChildren) return FindOctantWithinChildren(location, octant->children);

	return octant;
}

FOctant* ASixDOFNavmeshVolume::FindOctantWithinChildren(FVector location, TArray<FOctant>& children) {
	for (auto iter = children.begin(); iter != children.end(); ++iter) {
		FOctant& child = *iter;
		FVector max = child.center + child.extent;
		FVector min = child.center - child.extent;

		if (min.X <= location.X && location.X <= max.X &&
			min.Y <= location.Y && location.Y <= max.Y &&
			min.Z <= location.Z && location.Z <= max.Z) {
			if (child.navigatable == ENavigabilityStatus::HasChildren) return FindOctantWithinChildren(location, child.children);
			return &child;
		}
	}

	return nullptr;
}

TArray<FOctant*> ASixDOFNavmeshVolume::FindOctantsAroundMesh(UPrimitiveComponent* mesh) {
	TArray<FOctant*> octantsAroundMesh;

	FVector meshExtents = mesh->Bounds.BoxExtent;
	FVector meshMinBounds = (mesh->Bounds.Origin - meshExtents);
	FVector meshMaxBounds = (mesh->Bounds.Origin + meshExtents);

	for (int32 x = meshMinBounds.X; x <= meshMaxBounds.X; x++)
	{
		for (int32 y = meshMinBounds.Y; y <= meshMaxBounds.Y; y++)
		{
			for (int32 z = meshMinBounds.Z; z <= meshMaxBounds.Z; z++)
			{
				FOctant* octant = FindOctantAtLocation(FVector(x, y, z));
				if (octant && !octantsAroundMesh.Contains(octant)) octantsAroundMesh.Add(octant);
			}
		}
	}

	return octantsAroundMesh;
}

void ASixDOFNavmeshVolume::GenerateVoxelGrid() {
	float xPos = GetActorLocation().X;
	float yPos = GetActorLocation().Y;
	float zPos = GetActorLocation().Z;

	FVector boxScale = navmeshVolumeBounds->GetScaledBoxExtent() * 2;
	int32 xSize = FMath::CeilToInt(boxScale.X / octantSize);
	int32 ySize = FMath::CeilToInt(boxScale.Y / octantSize);
	int32 zSize = FMath::CeilToInt(boxScale.Z / octantSize);

	double start = FPlatformTime::Seconds();

	int32 id = 0;
	octants.Reserve(xSize);
	for (int i = 0; i < xSize; ++i) {
		octants.Emplace();
		octants[i].Reserve(ySize);
		for (int j = 0; j < ySize; ++j) {
			octants[i].Emplace();
			octants[i][j].Reserve(zSize);
			for (int k = 0; k < zSize; ++k) {
				FOctant octant;

				octant.id = id;
				++id;

				octant.xIndex = i;
				octant.yIndex = j;
				octant.zIndex = k;

				float x = octantSize * i + xPos + (octantSize / 2);
				float y = octantSize * j + yPos + (octantSize / 2);
				float z = octantSize * k + zPos + (octantSize / 2);

				octant.center = FVector(x, y, z);
				octant.extent = FVector(octantSize, octantSize, octantSize) * 0.5;

				SubdivideOctree(octant);
				octants[i][j].Emplace(octant);
			}
		}
	}
	double end = FPlatformTime::Seconds();
	UE_LOG(LogTemp, Warning, TEXT("Created grid of %i octants in %f seconds."), id, end - start);

	TArray<AActor*> navModifierActors;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), ASixDOFNavmeshModifier::StaticClass(), navModifierActors);
	for (auto actor : navModifierActors) {
		ASixDOFNavmeshModifier* navModifier = Cast<ASixDOFNavmeshModifier>(actor);
		if (navModifier) modifiers.Add(navModifier);
	}

	//DrawDebugNavmesh();
}

bool ASixDOFNavmeshVolume::CheckOctantCollision(FOctant& octant) {
	const int32 totalCellCount = 125;
	const int32 countUntilEarlyExit = totalCellCount * (percentUntilConsideredFull * .01f);
	int32 occupiedCellCount = 0;

	TArray<FOverlapResult> outOverlaps;
	FCollisionShape shape = FCollisionShape::MakeBox(octant.extent);
	bool overlapped = GetWorld()->OverlapMultiByObjectType(outOverlaps, octant.center, FQuat::Identity, octantCollisionObjectQueryParams, shape, octantCollisionQueryParams);

	if (overlapped) {
		octant.navigatable = ENavigabilityStatus::NonNavigable;

		FVector fractionExtent = shape.GetExtent() * 0.2f;
		float size = shape.GetBox().Size();
		const FVector dimension = FVector(size * .2f, size * .2f, size * .2f);

		FVector minBounds = octant.center - shape.GetExtent();
		FVector maxBounds = octant.center + shape.GetExtent();
		FVector cellSize = (maxBounds - minBounds) / 5;

		for (int32 X = 0; X < 5; X++) {
			for (int32 Y = 0; Y < 5; Y++) {
				for (int32 Z = 0; Z < 5; Z++) {
					if (occupiedCellCount >= countUntilEarlyExit) return true;

					FVector cellCenter = minBounds + FVector(X + 0.5f, Y + 0.5f, Z + 0.5f) * cellSize;
					FBox collisionShape(cellCenter - fractionExtent, cellCenter + fractionExtent);

					for (int32 j = 0; j < outOverlaps.Num(); ++j) {
						const FBox& actorShape = outOverlaps[j].GetActor()->GetComponentsBoundingBox();

						if (collisionShape.Min.X <= actorShape.Max.X && collisionShape.Max.X >= actorShape.Min.X &&
							collisionShape.Min.Y <= actorShape.Max.Y && collisionShape.Max.Y >= actorShape.Min.Y &&
							collisionShape.Min.Z <= actorShape.Max.Z && collisionShape.Max.Z >= actorShape.Min.Z) {
							++occupiedCellCount;
							break;
						}
					}
				}
			}
		}
	}

	return false;
}

void ASixDOFNavmeshVolume::SubdivideOctree(FOctant& octant) {
	bool octantFilled = CheckOctantCollision(octant);
	if (octant.level == maxSubdivisionLevel || octant.navigatable == ENavigabilityStatus::Navigable || octantFilled) return;

	octant.navigatable = ENavigabilityStatus::HasChildren;
	octant.children.Reserve(8);
	FVector quarterSize = octant.extent * 0.5f;
	for (int i = 0; i < 8; ++i) {
		FVector center;
		center.X = (i & 1) ? quarterSize.X : -quarterSize.X;
		center.Y = (i & 2) ? quarterSize.Y : -quarterSize.Y;
		center.Z = (i & 4) ? quarterSize.Z : -quarterSize.Z;
		center += octant.center;

		FOctant child;
		child.center = center;
		child.extent = quarterSize;
		child.level = octant.level + 1;
		child.index = i;
		child.parent = &octant;

		child.xIndex = octant.xIndex;
		child.yIndex = octant.yIndex;
		child.zIndex = octant.zIndex;

		SubdivideOctree(child);
		octant.children.Add(child);
	}
}

void ASixDOFNavmeshVolume::DrawDebugNavmesh() {
	for (auto octantX : octants) {
		for (auto octantY : octantX) {
			for (auto octantZ : octantY) {
				octantZ.DrawDebug(GetWorld());
			}
		}
	}
}

void ASixDOFNavmeshVolume::DrawDebugAroundMesh(UPrimitiveComponent* mesh) {
	TArray<FOctant*> octantsToDraw = FindOctantsAroundMesh(mesh);

	for (auto octant : octantsToDraw) {
		octant->Reset();

		octant->DrawDebug(GetWorld());
	}

	TArray<FOctant*> newOctantsToDraw = FindOctantsAroundMesh(mesh);
	for (auto newOctant : newOctantsToDraw) {
		SubdivideOctree(*newOctant);
		newOctant->DrawDebug(GetWorld());
	}
}