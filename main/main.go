package main

import (
	"container/heap"
	"fmt"
	"os"
	"regexp"
	"sort"
	"strconv"
	"strings"
)

type Node struct {
	id        int
	name      string
	neighbors []Path
}

type Router struct {
	name      string
	id        int
	neighbors map[string]RouterPath
	updated   bool
}

type Path struct {
	node *Node
	cost int
}

type RouterPath struct {
	router *Router
	cost   int
}

type Item struct {
	value    Node
	priority int
	index    int
}

type PriorityQueue []*Item

func (pq PriorityQueue) Len() int            { return len(pq) }
func (pq PriorityQueue) Less(i, j int) bool  { return pq[i].priority < pq[j].priority }
func (pq PriorityQueue) Swap(i, j int)       { pq[i], pq[j] = pq[j], pq[i]; pq[i].index, pq[j].index = i, j }
func (pq *PriorityQueue) Push(x interface{}) { *pq = append(*pq, x.(*Item)) }
func (pq *PriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	*pq = old[0 : n-1]
	return item
}

func (pq *PriorityQueue) Update(item *Item, value Node, priority int) {
	item.value = value
	item.priority = priority
	heap.Fix(pq, item.index)
}

func (router *Router) updateDistanceVector(neighbor *Router, neighborDistances map[string]RouterPath) bool {
	updated := false
	for neighborname, edgeneighbor := range neighborDistances {
		neighborCost, ok := router.neighbors[neighbor.name]
		if !ok {
			router.neighbors[neighbor.name] = RouterPath{router: neighbor, cost: int(^uint(0) >> 1)}
			neighborCost = router.neighbors[neighbor.name]
		}
		newDist := neighborCost.cost + edgeneighbor.cost
		othernewdist, ok := router.neighbors[neighborname]
		if !ok {
			router.neighbors[neighborname] = RouterPath{router: edgeneighbor.router, cost: int(^uint(0) >> 1)}
			othernewdist = router.neighbors[neighborname]
		}
		if newDist < othernewdist.cost {
			othernewdist.cost = newDist
			updated = true
		}
	}
	return updated
}

func (router *Router) getRouterPathString() string {
	var res strings.Builder
	for _, neighborpath := range router.neighbors {
		res.WriteString(strconv.Itoa(neighborpath.cost) + " ")
	}
	return res.String()
}

type DjikstraResult struct {
	costs []int
	paths []int
}

func Dijkstra(source *Node, graph []*Node) DjikstraResult {
	pq := make(PriorityQueue, 0)
	costs := make([]int, len(graph))
	paths := make([]int, len(graph))
	heap.Init(&pq)

	for i := range graph {
		costs[i] = int(^uint(0) >> 1)
		paths[i] = -1
		if graph[i] == source {
			heap.Push(&pq, &Item{value: *graph[i], priority: 0})
			costs[i] = 0
		}
	}

	for pq.Len() > 0 {
		curr := heap.Pop(&pq).(*Item)
		currNode := &curr.value

		for _, neighbor := range currNode.neighbors {
			newCost := costs[currNode.id] + neighbor.cost
			if newCost < costs[neighbor.node.id] {
				costs[neighbor.node.id] = newCost
				paths[neighbor.node.id] = currNode.id
				heap.Push(&pq, &Item{value: *neighbor.node, priority: newCost})
			}
		}
	}

	return DjikstraResult{costs: costs, paths: paths}
}

func BellmanFord(source int, graph []*Router, timestep int, maxIterations int) bool {
	iteration := timestep
	converged := false
	convergencecount := 0

	nextHop := make([]int, len(graph))
	for i := 0; i < len(nextHop); i++ {
		if i == source {
			nextHop[i] = source
		} else {
			nextHop[i] = -1
		}
	}

	for iteration < maxIterations && convergencecount < 5 {
		converged = true
		for j := 0; j < len(graph); j++ {
			for k := 0; k < len(graph[j].neighbors); k++ {
				neighborRouter, ok := graph[j].neighbors[graph[k].name]
				if !ok {
					graph[j].neighbors[graph[k].name] = RouterPath{router: graph[k], cost: int(^uint(0) >> 1)}
					neighborRouter = graph[j].neighbors[graph[k].name]
				}
				updated := graph[j].updateDistanceVector(neighborRouter.router, neighborRouter.router.neighbors)
				if updated {
					nextHop[j] = neighborRouter.router.id
					converged = false
				}
			}
		}
		if converged {
			convergencecount += 1
		}

		for j := 0; j < len(graph); j++ {
			// fmt.Println("Router: " + graph[j].name + " Distance Vector: " + graph[j].getRouterPathString())
			WriteStateBellman(graph, *graph[j], iteration, nextHop)
		}
		iteration++
	}
	return convergencecount >= 5
}

func WriteStateBellman(routerGraph []*Router, source Router, timestep int, nexthop []int) {
	path := "topology_DV_" + source.name + ".txt"
	var file *os.File

	if _, err := os.Stat(path); err == nil {
		file, err = os.OpenFile(path, os.O_APPEND|os.O_WRONLY|os.O_CREATE, 0600)
		if err != nil {
			fmt.Println("Error opening file:", err)
			return
		}
	} else {
		file, err = os.Create(path)
		if err != nil {
			fmt.Println("Error creating file:", err)
			return
		}
	}

	outputSlice := []struct {
		destination         string
		nexthop             string
		cost                string
		distancevectorslice map[string]RouterPath
	}{}

	for i := 0; i < len(routerGraph); i++ {
		var sum string

		if source.neighbors[routerGraph[i].name].cost == int(^uint(0)>>1) {
			sum = "N"
		} else {
			sum = strconv.Itoa(source.neighbors[routerGraph[i].name].cost)
		}
		outputSlice = append(outputSlice, struct {
			destination         string
			nexthop             string
			cost                string
			distancevectorslice map[string]RouterPath
		}{
			destination:         routerGraph[i].name,
			nexthop:             routerGraph[nexthop[routerGraph[i].id]].name,
			cost:                sum,
			distancevectorslice: source.neighbors,
		})
	}
	sort.Slice(outputSlice, func(i, j int) bool {
		return outputSlice[i].destination < outputSlice[j].destination
	})
	for i := 0; i < len(outputSlice); i++ {
		file.WriteString(fmt.Sprintf("%d	%s	%s	%s %s \n", timestep, outputSlice[i].destination, outputSlice[i].nexthop, outputSlice[i].cost, getDistanceVectorString(outputSlice[i].distancevectorslice)))
	}
}

type distancevectoroutput struct {
	name string
	cost string
}

func getDistanceVectorString(distancevectormap map[string]RouterPath) string {
	distancevectorslice := make([]distancevectoroutput, len(distancevectormap))
	for name, path := range distancevectormap {
		var newpath string
		if path.cost == int(^uint(0)>>1) {
			newpath = "N"
		} else {
			newpath = strconv.Itoa(path.cost)
		}
		distancevectorslice = append(distancevectorslice, distancevectoroutput{name: name, cost: newpath})
	}
	sort.Slice(distancevectorslice, func(i, j int) bool {
		return distancevectorslice[i].name < distancevectorslice[j].name
	})
	var res strings.Builder
	for i := 0; i < len(distancevectorslice); i++ {
		if i > 0 {
			res.WriteString("    ")
		}
		res.WriteString(distancevectorslice[i].cost)
	}
	return res.String()
}

func WriteStateDjikstra(graph []*Node, costs []int, paths []int, time int, source Node) {
	path := "topology_SPF_" + source.name + ".txt"
	var file *os.File

	if _, err := os.Stat(path); err == nil {
		file, err = os.OpenFile(path, os.O_APPEND|os.O_WRONLY|os.O_CREATE, 0600)
		if err != nil {
			fmt.Println("Error opening file:", err)
			return
		}
	} else {
		file, err = os.Create(path)
		if err != nil {
			fmt.Println("Error creating file:", err)
			return
		}
	}

	nodesWithCost := []struct {
		node *Node
		cost int
		path string
	}{}

	for i, cost := range costs {

		path := []string{}
		curr := i
		for curr != -1 {
			path = append([]string{graph[curr].name}, path...)
			curr = paths[curr]
		}

		nodesWithCost = append(nodesWithCost, struct {
			node *Node
			cost int
			path string
		}{
			node: graph[i],
			cost: cost,
			path: strings.Join(path, ", "),
		})
	}

	sort.Slice(nodesWithCost, func(i, j int) bool {
		return nodesWithCost[i].cost < nodesWithCost[j].cost
	})

	for _, nwc := range nodesWithCost {
		file.WriteString(fmt.Sprintf("%d | %-3s %-3d %-20s\n", time, nwc.node.name, nwc.cost, nwc.path))
	}

	file.WriteString("\n")
	file.Close()
}

func ConstructGraph(lines []string) ([]*Node, map[string]*Node) {
	nodeMap := make(map[string]*Node)
	routerMap := make(map[string]*Router)
	edgePattern := regexp.MustCompile(`(\d+):([A-Za-z]),([A-Za-z]),(\d+)`)

	var currentTime int
	graph := []*Node{}
	routerGraph := []*Router{}

	for _, line := range lines {
		if line == "" {
			continue
		}
		matches := edgePattern.FindStringSubmatch(line)
		if len(matches) != 5 {
			continue
		}

		time, _ := strconv.Atoi(matches[1])
		nodeAName, nodeBName, costStr := matches[2], matches[3], matches[4]
		cost, _ := strconv.Atoi(costStr)

		if time != currentTime {
			for i := 0; i < len(graph); i++ {
				source := graph[i]
				results := Dijkstra(source, graph)
				WriteStateDjikstra(graph, results.costs, results.paths, currentTime, *source)
				BellmanFord(source.id, routerGraph, currentTime, time)
			}
			currentTime = time
		}

		nodeA, existsA := nodeMap[nodeAName]
		routerA := routerMap[nodeAName]
		if !existsA {
			nodeA = &Node{id: len(nodeMap), name: nodeAName}
			routerA = &Router{id: len(nodeMap), name: nodeAName, neighbors: make(map[string]RouterPath)}
			routerA.neighbors[routerA.name] = RouterPath{router: routerA, cost: 0}
			nodeMap[nodeAName] = nodeA
			routerMap[nodeAName] = routerA
			graph = append(graph, nodeA)
			routerGraph = append(routerGraph, routerA)
		}

		nodeB, existsB := nodeMap[nodeBName]
		routerB := routerMap[nodeBName]
		if !existsB {
			nodeB = &Node{id: len(nodeMap), name: nodeBName}
			routerB = &Router{id: len(nodeMap), name: nodeBName, neighbors: make(map[string]RouterPath)}
			routerB.neighbors[routerB.name] = RouterPath{router: routerB, cost: 0}
			nodeMap[nodeBName] = nodeB
			routerMap[nodeBName] = routerB
			graph = append(graph, nodeB)
			routerGraph = append(routerGraph, routerB)
		}

		nodeA.neighbors = append(nodeA.neighbors, Path{node: nodeB, cost: cost})
		nodeB.neighbors = append(nodeB.neighbors, Path{node: nodeA, cost: cost})
		routerA.neighbors[nodeBName] = RouterPath{router: routerB, cost: cost}
		routerB.neighbors[nodeAName] = RouterPath{router: routerA, cost: cost}
		// routerA.neighbors = append(routerA.neighbors, RouterPath{router: routerB, cost: cost})
		// routerB.neighbors = append(routerB.neighbors, RouterPath{router: routerB, cost: cost})
	}

	for i := 0; i < len(graph); i++ {
		source := graph[i]
		results := Dijkstra(source, graph)
		WriteStateDjikstra(graph, results.costs, results.paths, currentTime, *source)
	}

	return graph, nodeMap
}

func main() {
	data, err := os.ReadFile("input.txt")
	if err != nil {
		panic(err)
	}
	lines := strings.Split(string(data), "\n")
	ConstructGraph(lines)
}
