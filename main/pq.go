package main

import (
	"container/heap"
)

type Item struct {
	value    Node
	priority int
	index    int
}

type PriorityQueue []*Item

func (pq PriorityQueue) Len() int {
	return len(pq)
}

func (pq PriorityQueue) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].index = i
	pq[j].index = j
}

func (pq PriorityQueue) Less(i, j int) bool {
	return pq[i].priority < pq[j].priority
}

func (piq *PriorityQueue) Push(x interface{}) {
	n := len(*piq)
	item := x.(*Item)
	item.index = n
	*piq = append(*piq, item)
}
func (piq *PriorityQueue) Pop() interface{} {
	old := *piq
	n := len(old)
	item := old[n-1]
	item.index = -1
	*piq = old[0 : n-1]
	return item
}

func (pq *PriorityQueue) Update(item *Item, value Node, priority int) {
	item.value = value
	item.priority = priority
	heap.Fix(pq, item.index)
}
