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

// Node for Djikstras using an adjacency list
type Node struct {
	id        int
	name      string
	neighbors []Path
}

// Router struct for Bellman Ford, neighbors is a list of direct connections and pathindex is a list of
// indexes of saved paths in the path list.
type Router struct {
	name      string
	id        int
	neighbors map[string]int
	nexthop   map[int]int
	pathindex []int
}

// Routing path represents an edge used in bellman ford algorith
type RoutingPath struct {
	routerA Router
	routerB Router
	cost    int
}

// Path represents an edge used in the Djikstra algorithm
type Path struct {
	node *Node
	cost int
}

// Item is used in priority queue
type Item struct {
	value    Node
	priority int
	index    int
}

// PriorityQueue used for Djikstra's
type PriorityQueue []*Item

// Basic PQ Implementation
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

// Result from Djikstra's algorithm struct
type DjikstraResult struct {
	costs []int
	paths []int
}

// Find shortest paths across the graph and return the result
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

// Output used from Bellman Ford Algorithm
type BellmanOutput struct {
	nexthop              string
	distancevectorstring string
	cost                 string
	destination          string
}

// Writes the current state of the Bellman Ford/DV Algorithm from source
func WriteStateBellman(currentDistanceVector [][]int, source Router, timestep int, routerGraph []*Router) {
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
	outputSlice := []BellmanOutput{}
	for i := 0; i < len(currentDistanceVector); i++ {
		var distancevectorstring string
		// If there is a link add that nodes distance vector
		_, ok := source.neighbors[routerGraph[i].name]
		if source.id == i {
			// distancevectorstring = getOtherDistanceVectorString(source, len(routerGraph), routerGraph)
			distancevectorstring = getDistanceVectorString(currentDistanceVector[i])
		} else if ok && timestep != 0 {
			distancevectorstring = getDistanceVectorString(currentDistanceVector[i])
		} else {
			distancevectorstring = getBlankDistanceVectorString(len(currentDistanceVector))
		}
		var nexthop string
		nexthopcheck, ok := source.nexthop[i]
		if !ok {
			nexthop = "N"
		} else {
			nexthop = routerGraph[nexthopcheck].name
		}
		tempcost := currentDistanceVector[source.id][i]
		var cost string
		if tempcost == int(^uint(0)>>1) {
			cost = "N"
		} else {
			cost = strconv.Itoa(tempcost)
		}
		outputSlice = append(outputSlice, BellmanOutput{distancevectorstring: distancevectorstring, cost: cost, nexthop: nexthop, destination: routerGraph[i].name})
	}
	sort.Slice(outputSlice, func(i, j int) bool {
		return outputSlice[i].destination < outputSlice[j].destination
	})
	for i := 0; i < len(outputSlice); i++ {
		// 		file.WriteString(fmt.Sprintf("%d | %-3s %-3d %-20s\n", time, nwc.node.name, nwc.cost, nwc.path))
		file.WriteString(fmt.Sprintf("%d  %2s  %2s  %2s  | %2s\n", timestep, outputSlice[i].destination, outputSlice[i].nexthop, outputSlice[i].cost, outputSlice[i].distancevectorstring))
	}
	file.WriteString("\n")
}

// Gets the distance vector into string form to write
func getDistanceVectorString(distancevector []int) string {
	var res strings.Builder
	for i := 0; i < len(distancevector); i++ {
		if i > 0 && distancevector[i] >= 10 && distancevector[i] != int(^uint(0)>>1) {
			res.WriteString("  ")
		} else if i > 0 {
			res.WriteString("   ")
		}
		cost := distancevector[i]
		if cost == int(^uint(0)>>1) {
			res.WriteString("N")
		} else {
			res.WriteString(strconv.Itoa(distancevector[i]))
		}
	}
	return res.String()
}

// If there is not a link between the two routers use a blank distance vector string
func getBlankDistanceVectorString(routerGraphLength int) string {
	var res strings.Builder
	for i := 0; i < routerGraphLength; i++ {
		if i > 0 {
			res.WriteString("   ")
		}
		res.WriteString("N")
	}
	return res.String()
}

// Writes the state of the graph after running Djikstra algorithm
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
		file.WriteString(fmt.Sprintf("%d |  %2s  %2d  %-20s\n", time, nwc.node.name, nwc.cost, nwc.path))
	}

	file.WriteString("\n")
	file.Close()
}

// Construct routing table [x][y][z] = cost at x,y,z
// Where x = timestep
// Where y = distance vector at x,y
// Where z = destination
func ConstructRoutingTable(lengthOfGraph int) [][][]int {
	result := make([][][]int, 100)

	for i := range result {
		result[i] = make([][]int, lengthOfGraph)
		for j := range result[i] {
			result[i][j] = make([]int, lengthOfGraph)
			for k := range result[i][j] {
				result[i][j][k] = int(^uint(0) >> 1)
			}
		}
	}
	return result
}

// Initially each router should only know of a path to itself at cost 0
func InitializeRoutingTable(routingGraph []*Router) [][][]int {
	routingTable := ConstructRoutingTable(len(routingGraph))
	for _, router := range routingGraph {
		routingTable[0][router.id][router.id] = 0
	}
	return routingTable
}

// Need to copy values instead of just returning reference table as values will be modified in routing table
func GetGlobalDistanceVector(routingTable [][][]int, timestep int) [][]int {
	newDistanceVector := make([][]int, len(routingTable[timestep]))

	for i := range routingTable[timestep] {
		newDistanceVector[i] = make([]int, len(routingTable[timestep][i]))
		copy(newDistanceVector[i], routingTable[timestep][i])
	}

	return newDistanceVector
}

// Gets the current distance vector by copying over the distance vector from previous timestep
func GetLocalDistanceVector(source Router, routingTable [][][]int, timestep int, routergraph []*Router) [][]int {
	var prevtimestep int
	result := [][]int{}
	if timestep > 0 {
		prevtimestep = timestep - 1
	} else {
		prevtimestep = timestep
	}
	for i := 0; i < len(routingTable[timestep]); i++ {
		if source.id == i {
			result = append(result, routingTable[timestep][i])
		} else {
			// If the source has a INITIAL path to the neighbor add the costs to the result
			// Otherwise add a list of max int
			_, ok := source.neighbors[routergraph[i].name]
			if !ok {
				tempresult := make([]int, len(routingTable[prevtimestep][0]))
				for j := 0; j < len(routingTable[prevtimestep][0]); j++ {
					tempresult[j] = int(^uint(0) >> 1)
				}
				result = append(result, tempresult)
			} else {
				result = append(result, routingTable[prevtimestep][i])
			}
		}
	}
	return result
}

// To add new Paths at timestep i
// If routingtable[i - 1][source][destination] > newpath(source, desination, cost)
// Add path to timestep i
// Else stay with old path
// Write updated path at timestep i
func updateDistanceVector(source Router, routingTable [][][]int, timestep, destinationID, newCost int) [][]int {
	currentDistanceVector := routingTable[timestep]
	currentDistanceVector[source.id][destinationID] = newCost
	source.nexthop[destinationID] = destinationID

	return currentDistanceVector
}

func BellmanFord(source Router, routingTable [][][]int, timestep int, pathList []*RoutingPath) (bool, [][][]int) {
	// For each edge
	// where u = startingedge
	// where v = destinationedge
	// See if distance from source to u + distance from u to v is less than distance of source to v
	// If it is update routingtable[timestep + 2][source][v] to be source to u + distrance from u to v
	// set next hop to v to be u
	updateCount := 0
	for _, path := range pathList {
		u := path.routerA
		v := path.routerB
		tempdistance := routingTable[timestep][source.id][u.id] + routingTable[timestep][u.id][v.id]
		if tempdistance > 0 && tempdistance < routingTable[timestep][source.id][v.id] {
			routingTable[timestep+2][source.id][v.id] = tempdistance
			source.nexthop[v.id] = u.id
			updateCount += 1
		}
	}
	if updateCount > 0 {
		return false, routingTable
	} else {
		return true, routingTable
	}
}

// Using the input in lines construct the graph run simulation and Djikstra's algorithm
func ConstructGraph(lines []string) int {
	nodeMap := make(map[string]*Node)
	routerMap := make(map[string]*Router)
	edgePattern := regexp.MustCompile(`(\d+):([A-Za-z]),([A-Za-z]),(\d+)`)
	var currentTime int
	graph := []*Node{}
	routerGraph := []*Router{}
	pathList := []*RoutingPath{}
	currentRouteIndex := 0
	var routingTable [][][]int
	even := true

	for i, line := range lines {
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

		// If the time is updated run simulation up to that point
		// OR if all lines have been read run simulation
		if time != currentTime || i == len(lines)-2 {
			// Write truth table at time
			for i := 0; i < len(graph); i++ {
				source := graph[i]
				results := Dijkstra(source, graph)
				WriteStateDjikstra(graph, results.costs, results.paths, currentTime, *source)
			}
			if routingTable == nil {
				// If routing table has not been initialized
				// Initialize it
				routingTable = InitializeRoutingTable(routerGraph)
				currentTime += 1
				for i := 0; i < len(routerGraph); i++ {
					WriteStateBellman(GetLocalDistanceVector(*routerGraph[i], routingTable, currentTime-1, *&routerGraph), *routerGraph[i], currentTime-1, routerGraph)
				}
			}
			// before adding new paths copy over previous pathse
			if currentRouteIndex < len(pathList) {
				routingTable[currentTime] = GetGlobalDistanceVector(routingTable, currentTime-1)
				routingTable[currentTime+1] = GetGlobalDistanceVector(routingTable, currentTime)
			}
			for i := currentRouteIndex; i < len(pathList); i++ {
				// Add all the paths to the routing table
				path := pathList[i]
				updateDistanceVector(path.routerA, routingTable, currentTime+1, path.routerB.id, path.cost)
				if (currentTime+1)%2 == 1 {
					even = false
				} else {
					even = true
				}
			}
			currentTime += 1
			currentRouteIndex = len(pathList)
			simulation := true
			convergencecount := 0
			for simulation {
				// If converged 5 iterations in a row write the output and return 1
				if convergencecount == 5 || currentTime >= 100 {
					for i := range len(routerGraph) {
						WriteStateBellman(GetLocalDistanceVector(*routerGraph[i], routingTable, currentTime-1, *&routerGraph), *routerGraph[i], currentTime-1, routerGraph)
					}
					return 1
				}
				// If simulated up to new input you can break
				if currentTime == time {
					break
				}
				// Write the states from last iteration of simulation before modifying new ones
				for i := range len(routerGraph) {
					WriteStateBellman(GetLocalDistanceVector(*routerGraph[i], routingTable, currentTime-1, *&routerGraph), *routerGraph[i], currentTime-1, routerGraph)
				}
				var convergence bool
				updated := 0
				var newRoutingTable [][][]int
				var intcheck int
				// Use offsetting because we are copying routing table over every 2 iterations and need to change when we do
				// if we are adding new nodes
				if even {
					intcheck = 0
				} else {
					intcheck = 1
				}
				// Every other iteration copy over routingtable at last time to new time and new time + 1
				if currentTime != 99 && currentTime%2 == intcheck {
					routingTable[currentTime+1] = GetGlobalDistanceVector(routingTable, currentTime)
					if currentTime < 97 {
						routingTable[currentTime+2] = GetGlobalDistanceVector(routingTable, currentTime+1)
					}
				}
				// Conduct Bellman Ford from all sources
				for i := 0; i < len(routerGraph) && currentTime < 97; i++ {
					convergence, newRoutingTable = BellmanFord(*routerGraph[i], routingTable, currentTime, pathList)

					routingTable = newRoutingTable
					if !convergence {
						updated += 1
					}
				}
				// If it wasn't modified increment convergence
				if updated == 0 {
					convergencecount += 1
				}
				currentTime += 1
			}
			currentTime = time
		}

		nodeA, existsA := nodeMap[nodeAName]
		routerA := routerMap[nodeAName]
		// If node doesn't exist create necessary node
		if !existsA {
			nodeA = &Node{id: len(nodeMap), name: nodeAName}
			routerA = &Router{id: len(nodeMap), name: nodeAName, neighbors: make(map[string]int), nexthop: make(map[int]int), pathindex: []int{}}
			routerA.nexthop[routerA.id] = routerA.id
			nodeMap[nodeAName] = nodeA
			routerMap[nodeAName] = routerA
			graph = append(graph, nodeA)
			routerGraph = append(routerGraph, routerA)
		}

		nodeB, existsB := nodeMap[nodeBName]
		routerB := routerMap[nodeBName]
		if !existsB {
			nodeB = &Node{id: len(nodeMap), name: nodeBName}
			routerB = &Router{id: len(nodeMap), name: nodeBName, neighbors: make(map[string]int), nexthop: make(map[int]int), pathindex: []int{}}
			routerB.nexthop[routerB.id] = routerB.id
			nodeMap[nodeBName] = nodeB
			routerMap[nodeBName] = routerB
			graph = append(graph, nodeB)
			routerGraph = append(routerGraph, routerB)
		}
		// Check if node already has a path
		foundNodeA := false
		for i := 0; i < len(nodeA.neighbors); i++ {
			if nodeA.neighbors[i].node == nodeB {
				nodeA.neighbors[i].cost = cost
				foundNodeA = true
				break
			}
		}
		foundNodeB := false
		for i := 0; i < len(nodeB.neighbors); i++ {
			if nodeB.neighbors[i].node == nodeA {
				nodeB.neighbors[i].cost = cost
				foundNodeB = true
				break
			}
		}
		if !foundNodeA {
			nodeA.neighbors = append(nodeA.neighbors, Path{node: nodeB, cost: cost})
		}
		if !foundNodeB {
			nodeB.neighbors = append(nodeB.neighbors, Path{node: nodeA, cost: cost})
		}
		routerA.neighbors[routerB.name] = cost
		routerB.neighbors[routerA.name] = cost
		// Check if router already has path in edge list if so it will need to be updated
		foundpathA := false
		for pathindex := range routerA.pathindex {
			if routerB.name == pathList[pathindex].routerB.name {
				pathList[pathindex].cost = cost
				foundpathA = true
			}
		}
		foundpathB := false
		for pathindex := range routerB.pathindex {
			if routerA.name == pathList[pathindex].routerA.name {
				pathList[pathindex].cost = cost
				foundpathB = true
			}
		}
		// If path doesn't exist make a new path
		if !foundpathA {
			pathList = append(pathList, &RoutingPath{routerA: *routerA, routerB: *routerB, cost: cost})
			routerA.pathindex = append(routerA.pathindex, len(pathList)-1)
		}
		if !foundpathB {
			pathList = append(pathList, &RoutingPath{routerA: *routerB, routerB: *routerA, cost: cost})
			routerB.pathindex = append(routerB.pathindex, len(pathList)-1)
		}
	}
	for i := 0; i < len(graph); i++ {
		source := graph[i]
		results := Dijkstra(source, graph)
		WriteStateDjikstra(graph, results.costs, results.paths, currentTime, *source)
	}

	return -1
}

func main() {
	data, err := os.ReadFile("input.txt")
	if err != nil {
		panic(err)
	}
	lines := strings.Split(string(data), "\n")
	lines = append(lines, "")
	ConstructGraph(lines)
}
