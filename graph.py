import re
import heapq


class Node:
    def __init__(self, name):
        self.paths = {}
        self.name = name
    
    def addpath(self, node: Node, cost):
        self.paths.update({node.name: (node, cost)})

def buildgraph(filestring):
    currenttimestamp = 0
    graph = []
    nodes = {}
    for line in filestring.splitlines():
        if line[0] != currenttimestamp:
            getshortestpaths(graph)
        else:
            pattern = r'([A-Z]), ([A-Z]), (\d+)'
            match = re.find(pattern, line)
            nodea = nodes.get(match[0])
            nodeb = nodes.get(match[1])
            if not nodea:
                nodea = Node(match[0])
                nodes.update({match[0]: nodea})
                graph.append(nodea)
            if not nodeb:
                nodeb = nodes.get(match[1])
                nodes.update({match[1]: nodeb})
                graph.append(nodeb)
            nodea.addpath(nodeb, match[2])
            nodeb.addpath(nodea, match[2])

def getshortestpaths(graph):
    for node in graph:
        output = {}
        unvisited = set()
        pq = []
        for innernode in graph:
            if innernode == node:
                output.update({innernode.name: (0, [])})
                heapq.heappush(pq, (0, innernode.name))
            else:
                output.update({innernode.name: (int('inf'), [])})
                heapq.heappush(pq, (int('inf'), innernode.name))