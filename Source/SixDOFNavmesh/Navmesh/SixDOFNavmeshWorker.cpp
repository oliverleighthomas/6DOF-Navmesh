// Fill out your copyright notice in the Description page of Project Settings.


#include "SixDOFNavmeshWorker.h"
#include "SixDOFNavmeshVolume.h"

SixDOFNavmeshWorker::SixDOFNavmeshWorker(ASixDOFNavmeshVolume* volume) :
	volume{ volume }
{
	thread = FRunnableThread::Create(this, TEXT("6DOF Navmesh Worker"));
}

SixDOFNavmeshWorker::~SixDOFNavmeshWorker() {
	if (thread) {
		thread->Kill();
		delete thread;
	}
}

bool SixDOFNavmeshWorker::Init() {
	UE_LOG(LogTemp, Warning, TEXT("The 6DOF navmesh worker thread has been started."));
	return true;
}

uint32 SixDOFNavmeshWorker::Run() {
	while (shouldRun && volume) {
		volume->TickDynamicCollisionUpdates();
		volume->TickPathfindingUpdates(tickTime, volume->maxPathfindingTasksPerTick);

		FPlatformProcess::Sleep(tickTime);
	}

	return 0;
}

void SixDOFNavmeshWorker::Stop() {
	shouldRun = false;
	thread->WaitForCompletion();
}
