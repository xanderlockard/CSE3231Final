package main

import (
	"container/heap"
	"math"
	"os"
	"regexp"
	"strconv"
)

type Node struct {
	id        int
	name      string
	neighbors []Path
}

type DjikstraResult struct {
	resultSlice []int
	pathSlice   []int
}

type Path struct {
	node Node
	cost int
}

func newNode(name string, id int) Node {
	node := Node{id: id, name: name}
	return node
}

func newPath(cost int, node Node) Path {
	path := Path{cost: cost, node: node}
	return path
}

func (n *Node) addNeighbor(node Node, cost int) {
	path := newPath(cost, node)
	n.neighbors = append(n.neighbors, path)
}

func djikstra(source Node, graph []Node) DjikstraResult {
	pq := make(PriorityQueue, 0)
	cost := make([]int, len(graph))
	path := make([]int, len(graph))
	visited := map[int]bool{}
	for i := 0; i < len(graph); i++ {
		if graph[i].id != source.id {
			pq.Push(&Item{value: graph[i], priority: math.MaxInt, index: graph[i].id})
			cost[graph[i].id] = math.MaxInt
			path[graph[i].id] = -1
			visited[graph[i].id] = false
		}
	}

	cost[source.id] = 0
	pq.Push(&Item{value: source, priority: 0, index: len(graph)})
	heap.Init(&pq)

	for pq.Len() > 0 {
		curr := heap.Pop(&pq).(*Item)
		for j := 0; j < len(curr.value.neighbors); j++ {
			tempcost := cost[curr.value.id] + curr.value.neighbors[j].cost
			if tempcost < cost[curr.value.neighbors[j].node.id] && !visited[curr.value.id] {
				cost[curr.value.neighbors[j].node.id] = tempcost
				path[curr.value.neighbors[j].node.id] = curr.value.id
				visited[curr.value.id] = true
				pq[curr.value.neighbors[j].node.id].priority = tempcost
				heap.Fix(&pq, curr.value.neighbors[j].node.id)
			}
		}
		heap.Fix(&pq, curr.value.id)
	}
	return DjikstraResult{resultSlice: cost, pathSlice: path}
}

func main() {
	data := readinput("input.txt")
	ConstructGraphFromByteArray(data)
}

func readinput(filepath string) []byte {
	data, err := os.ReadFile(filepath)
	if err != nil {
		panic(err)
	}
	return data
}

func ConstructGraphFromByteArray(bytearr []byte) {
	startchar := byte('0')
	newlinechar := byte('\n')
	graphstateslice := []Node{}
	r := regexp.MustCompile(`(\d+):([A-Za-z]),([A-Za-z]),(\d+)`)
	nodeint := 0
	nodeset := make(map[string]int)
	for i := 0; i < len(bytearr); {
		a := bytearr[i]
		j := i
		linelength := 0
		if a != startchar {
			resultslice := []DjikstraResult{}
			for j := 0; j < len(graphstateslice); j++ {
				result := djikstra(graphstateslice[j], graphstateslice)
				resultslice = append(resultslice, result)
			}
		}
		for a != newlinechar {
			linelength += 1
			j++
			a = bytearr[j]
		}
		m := r.FindStringSubmatch(string(bytearr[i:(i + linelength)]))
		nodeaname := m[2]
		nodebname := m[3]
		cost, _ := strconv.Atoi(m[4])
		nodea, ok := nodeset[nodeaname]
		nodeb, okb := nodeset[nodebname]
		if !ok && !okb {
			nodea := newNode(nodeaname, nodeint)
			nodeint++
			nodeb := newNode(nodebname, nodeint)
			nodeint++
			nodea.addNeighbor(nodeb, cost)
			nodeb.addNeighbor(nodea, cost)
			graphstateslice = append(graphstateslice, nodea, nodeb)
			nodeset[nodeaname], nodeset[nodebname] = nodea.id, nodeb.id
		} else if !ok {
			nodea := newNode(nodeaname, nodeint)
			nodea.addNeighbor(graphstateslice[nodeb], cost)
			graphstateslice[nodeb].addNeighbor(nodea, cost)
			nodeint++
			graphstateslice = append(graphstateslice, nodea)
			nodeset[nodeaname] = nodea.id
		} else if !okb {
			nodeb := newNode(nodebname, nodeint)
			nodeint++
			graphstateslice[nodea].addNeighbor(nodeb, cost)
			nodeb.addNeighbor(graphstateslice[nodea], cost)
			graphstateslice = append(graphstateslice, nodeb)
			nodeset[nodebname] = nodeb.id
		} else {
			graphstateslice[nodea].addNeighbor(graphstateslice[nodeb], cost)
			graphstateslice[nodeb].addNeighbor(graphstateslice[nodea], cost)
		}
		i += linelength + 1
	}
}
