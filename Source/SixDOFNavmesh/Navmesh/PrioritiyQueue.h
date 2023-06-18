// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

template <typename T>
class SIXDOFNAVMESH_API PrioritiyQueue
{
public:
	PrioritiyQueue() {
		queue.Heapify();
	}

	~PrioritiyQueue() {}

public:
	void Push(T data, float priority) {
		queue.HeapPush(PriorityQueueNode(data, priority));

	}

	void Pop() {
		if (IsEmpty()) return;

		PriorityQueueNode dummy;
		queue.HeapPop(dummy);
	}

	T Top() {
		return !IsEmpty() ? queue.HeapTop().data : NULL;
	}

	bool IsEmpty() const {
		return queue.Num() == 0;
	}

private:
	struct PriorityQueueNode {
		T data;
		float priority;

		PriorityQueueNode() {}
		PriorityQueueNode(T data, float priority) : data{ data }, priority{ priority }
		{
		}

		bool operator<(const PriorityQueueNode other) const {
			return priority < other.priority;
		}
	};

    TArray<PriorityQueueNode> queue;
};
