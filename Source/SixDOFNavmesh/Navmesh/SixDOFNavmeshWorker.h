// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"

class ASixDOFNavmeshVolume;

class SIXDOFNAVMESH_API SixDOFNavmeshWorker : public FRunnable
{
public:
	SixDOFNavmeshWorker(ASixDOFNavmeshVolume* volume);
	virtual ~SixDOFNavmeshWorker();

	bool Init() override;
	uint32 Run() override;
	void Stop() override;

private:
	FRunnableThread* thread;
	ASixDOFNavmeshVolume* volume;
	bool shouldRun = true;

	float tickTime = 0.03f;
};
