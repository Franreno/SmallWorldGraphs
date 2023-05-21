from random import random, randint, shuffle
from node import Node
from edge import Edge
from typing import List, Tuple
from queue import PriorityQueue

class SmallWorld:
    """Small world graph implementation"""

    numberOfVertices: int
    """Number of vertices"""

    numberOfEdges: int
    """Number of edges"""

    reconnectProbability: float
    """The reconnect probability `p`"""

    nodeList: List[Node]
    """ List of nodes from the graph """

    adjacencyList: List[List[Edge]]
    """ 
    List of adjacency of the graph 
    `adjacencyList[0] = [(0,1), (0,2) ...]`
    """

    def __init__(
        self, _numberOfVertices: int, _numberOfEdges: int, _reconnectProbability: float
    ) -> None:
        self.numberOfVertices = _numberOfVertices
        self.numberOfEdges = _numberOfEdges
        self.reconnectProbability = _reconnectProbability

        print("Generating vertices")
        self.nodeList = self.generateVertices()
        print("Generating edges")
        self.adjacencyList = self.generateEdges()
        print("Reconnecting edges")
        self.reconnectEdges()

    def generateVertices(self) -> List[Node]:
        """Generates all vertices using `randint` for the coordinates

        Returns:
            list[Node]: The list of nodes created
        """
        return [
            Node(
                idNum,
                randint(0, self.numberOfVertices),
                randint(0, self.numberOfVertices),
            )
            for idNum in range(0, self.numberOfVertices)
        ]

    def generateEdges(self) -> List[List[Edge]]:
        """Creates the adjacency list for the graph

        Returns:
            list[list[Edge]]: The adjacency list created
        """
        auxAdjacencyList = [[] for _ in range(self.numberOfVertices)]
        # Create `numberOfEdges` edges for every node in nodeList
        for node in self.nodeList:
            # Calculate distance from all other nodes
            distancesFromNode = self.calculateDistanceFromAllNodes(node)
            distancesFromNode.sort(key=lambda x: x[1])
            # The first item of list is the node itself
            # Therefore it must be removed
            distancesFromNode.pop(0)

            subDistancesList = distancesFromNode[0 : self.numberOfEdges]

            shuffle(subDistancesList)

            # Add the subDistancesList to the adjancecny list
            auxAdjacencyList[node.id] = [
                Edge(node, subDist[0], subDist[1]) for subDist in subDistancesList
            ]

        return auxAdjacencyList

    # def depthFirstSearch(self, startNodeIndex: int, targetNodeIndex: int) -> Tuple[List[int], float]:
    #     visited = [False] * self.numberOfVertices
    #     path = []
    #     distance = 0.0

    #     self._dfs(startNodeIndex, targetNodeIndex, visited, path, distance)

    #     return path, distance

    # def _dfs(self, currentNodeIndex: int, targetNodeIndex: int, visited: List[bool], path: List[int], distance: float):
    #     visited[currentNodeIndex] = True
    #     path.append(currentNodeIndex)

    #     if currentNodeIndex == targetNodeIndex:
    #         return

    #     neighbors = self.adjacencyList[currentNodeIndex]
    #     for edge in neighbors:
    #         neighborIndex = edge.node2.id
    #         if not visited[neighborIndex]:
    #             distance += edge.weight
    #             self._dfs(neighborIndex, targetNodeIndex, visited, path, distance)
    #             if path[-1] == targetNodeIndex:
    #                 return

    #     # Backtrack if the target node is not found from the current node
    #     path.pop()
    #     distance -= edge.weight

    def depthFirstSearch(self, startNodeIndex: int, targetNodeIndex: int) -> Tuple[List[int], float]:
        visited = [False] * self.numberOfVertices
        path = []
        # distance = 0.0
        distance = []
        distance.append(0.0)

        self._dfs(startNodeIndex, targetNodeIndex, visited, path, distance)

        return path, distance[0]

    def _dfs(
        self,
        currentNodeIndex: int,
        targetNodeIndex: int,
        visited: List[bool],
        path: List[int],
        distance: float,
    ):
        visited[currentNodeIndex] = True
        path.append(currentNodeIndex)

        if currentNodeIndex == targetNodeIndex:
            return

        neighbors = self.adjacencyList[currentNodeIndex]
        for edge in neighbors:
            neighborIndex = edge.node2.id
            if not visited[neighborIndex]:
                distance[0]+= edge.weight
                # print(distance)
                self._dfs(neighborIndex, targetNodeIndex, visited, path, distance)
                if path[-1] == targetNodeIndex:
                    return
                distance[0] -= edge.weight

        # Backtrack if the target node is not found from the current node
        path.pop()

    def bestFirstSearch(self, startNodeIndex: int, targetNodeIndex: int) -> Tuple[List[int], float]:
        visited = [False] * self.numberOfVertices
        path = []
        distance = self._bestFirstSearch(startNodeIndex, targetNodeIndex, visited, path)

        return path, distance

    def _bestFirstSearch(
        self,
        startNodeIndex: int,
        targetNodeIndex: int,
        visited: List[bool],
        path: List[int],
    ) -> float:
        priorityQueue = PriorityQueue()
        priorityQueue.put((0, startNodeIndex))  # (heuristic value, node index)

        while not priorityQueue.empty():
            _, currentNodeIndex = priorityQueue.get()

            if visited[currentNodeIndex]:
                continue

            visited[currentNodeIndex] = True
            path.append(currentNodeIndex)

            if currentNodeIndex == targetNodeIndex:
                return self.calculatePathDistance(path)

            neighbors = self.adjacencyList[currentNodeIndex]
            for edge in neighbors:
                neighborIndex = edge.node2.id
                if not visited[neighborIndex]:
                    priority = self.calculateHeuristic(neighborIndex, targetNodeIndex)
                    priorityQueue.put((priority, neighborIndex))

        return 0.0

    def calculateHeuristic(self, nodeIndex: int, targetNodeIndex: int) -> float:
        node = self.nodeList[nodeIndex]
        targetNode = self.nodeList[targetNodeIndex]
        return node.geometricDistanceFromNode(targetNode)

    def calculatePathDistance(self, path: List[int]) -> float:
        distance = 0.0
        for i in range(len(path) - 1):
            currentNodeIndex = path[i]
            nextNodeIndex = path[i + 1]
            neighbors = self.adjacencyList[currentNodeIndex]
            for edge in neighbors:
                if edge.node2.id == nextNodeIndex:
                    distance += edge.weight
                    break

        return distance

    def reconnectEdges(self):
        """Reconnect all edges with `reconnectProbability` value."""
        # iterate through all edges
        for nodeId, edges in enumerate(self.adjacencyList):
            for edge in edges:
                randomTestNumber: float = random()
                # only reconnect if randomNumber is less than the probability
                if randomTestNumber < self.reconnectProbability:
                    # Reconnect
                    randomNode = self.selectRandomNodeFromNodeList(edge)
                    edge.reconnect(randomNode)

    def selectRandomNodeFromNodeList(self, originEdge: Edge) -> Node:
        """Selects a random node from the node list without selecting
        nodes that are already connected to the `node1` from `originEdge`

        Args:
            originEdge (Edge): The edge that is being reconnected. Used to check `node1`

        Returns:
            Node: The random node chosen
        """
        edgeIndex = originEdge.node1.id

        # Node ids that are already connected to node1
        notSelectableIds = [edgeIndex]

        # get the edge list for the `node1`
        edgeList: list[Edge] = self.adjacencyList[edgeIndex]
        for edge in edgeList:
            # only add the `node2` ids
            notSelectableIds.append(edge.node2.id)

        sucess = False
        randomNode = None
        # Keeps trying to get a random node from `nodeList`
        # That isnt present on `notSelectableIds`
        while sucess == False:
            randomNode = self.nodeList[randint(0, self.numberOfVertices - 1)]
            if randomNode.id not in notSelectableIds:
                sucess = True

        return randomNode

    def calculateDistanceFromAllNodes(self, origin: Node) -> List[Tuple[Node, float]]:
        """Calculates distance from one node to all other nodes

        Args:
            origin (Node): The node on which all distances are being calculated upon

        Returns:
            list[tuple[Node, float]]: The list of `Node` and `distance` from the `origin`node.
        """
        return [
            (node, origin.geometricDistanceFromNode(node)) for node in self.nodeList
        ]

    def toString(self) -> str:
        auxListStr: list[str] = []
        for index, edges in enumerate(self.adjacencyList):
            auxStr = f"{ self.nodeList[index].toString()} Edges: \n"
            for edge in edges:
                auxStr += f"\t{edge.toString()}\n"

            auxListStr.append(auxStr)

        return "------------------------------------------------------------------------------------\n\n".join(
            auxListStr
        )
