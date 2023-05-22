from edge import Edge
from node import Node
from small_world import SmallWorld
from time import time
from typing import List, Dict, Tuple
from queue import PriorityQueue


class AlgorithmSubResult:
    path: List[Node]
    distance: float
    timeUsed: float

    def __init__(
        self,
        _path: List[Node],
        _distance: float,
        _timeUsed: float,
    ) -> None:
        self.path = _path
        self.distance = _distance
        self.timeUsed = _timeUsed


class AlgorithmResult:
    meanDistance: float
    meanTime: float
    pathsUsed: List[int]

    def calculateValues(self, results: List[AlgorithmSubResult]):
        sumDistances = 0
        sumTimes = 0
        for result in results:
            sumDistances += result.distance
            sumTimes += result.timeUsed

        self.meanDistance = sumDistances / len(results)
        self.meanTime = sumTimes / len(results)
        self.pathsUsed.append(result.path)

        return self.meanDistance, self.meanTime


class Algorithms:
    listOfOriginNodes: List[int]
    listOfDestinyNodes: List[int]
    smallWorld: SmallWorld

    def __init__(
        self,
        _listOfOriginNodes: List[int],
        _listOfDestinyNodes: List[int],
        _smallWorld: SmallWorld,
    ) -> None:
        self.listOfOriginNodes = _listOfOriginNodes
        self.listOfDestinyNodes = _listOfDestinyNodes
        self.smallWorld = _smallWorld

    def runAllAlgorithms(self) -> Dict[str, AlgorithmResult]:
        algorithmsSubResults = []
        for origin, destiny in zip(self.listOfOriginNodes, self.listOfDestinyNodes):
            algorithmsSubResults.append(self.depthFirstSearch(origin, destiny))

        dfsAlgorithmResult = AlgorithmResult().calculateValues(algorithmsSubResults)

        algorithmsSubResults = []
        for origin, destiny in zip(self.listOfOriginNodes, self.listOfDestinyNodes):
            algorithmsSubResults.append(self.bfs(origin, destiny))

        bfsAlgorithmResult = AlgorithmResult().calculateValues(algorithmsSubResults)

        algorithmsSubResults = []
        for origin, destiny in zip(self.listOfOriginNodes, self.listOfDestinyNodes):
            algorithmsSubResults.append(self.bestFirstSearch(origin, destiny))

        bestFirstAlgorithmResult = AlgorithmResult().calculateValues(
            algorithmsSubResults
        )

        algorithmsSubResults = []
        for origin, destiny in zip(self.listOfOriginNodes, self.listOfDestinyNodes):
            algorithmsSubResults.append(self.aStarEuclidian(origin, destiny))

        aStarEuclidianAlgorithmResult = AlgorithmResult().calculateValues(
            algorithmsSubResults
        )

        algorithmsSubResults = []
        for origin, destiny in zip(self.listOfOriginNodes, self.listOfDestinyNodes):
            algorithmsSubResults.append(self.aStarManhattan(origin, destiny))

        aStarManhattanAlgorithmResult = AlgorithmResult().calculateValues(
            algorithmsSubResults
        )

        return {
            "dfsAlgorithmResult": f"Mean Time: {dfsAlgorithmResult[1]} ; Mean Distance: {dfsAlgorithmResult[0]}",
            "bfsAlgorithmResult": f"Mean Time: {bfsAlgorithmResult[1]} ; Mean Distance: {bfsAlgorithmResult[0]}",
            "bestFirstAlgorithmResult": f"Mean Time: {bestFirstAlgorithmResult[1]} ; Mean Distance: {bestFirstAlgorithmResult[0]}",
            "aStarEuclidianAlgorithmResult": f"Mean Time: {aStarEuclidianAlgorithmResult[1]} ; Mean Distance: {aStarEuclidianAlgorithmResult[0]}",
            "aStarManhattanAlgorithmResult": f"Mean Time: {aStarManhattanAlgorithmResult[1]} ; Mean Distance: {aStarManhattanAlgorithmResult[0]}",
        }

    def depthFirstSearch(
        self, startNodeIndex: int, targetNodeIndex: int
    ) -> Tuple[List[int], float, float]:
        startTime = time()
        visited = [False] * self.smallWorld.numberOfVertices
        path = []
        distance = []
        distance.append(0.0)

        self.dfs(
            startNodeIndex, targetNodeIndex, visited, path, distance, self.smallWorld
        )

        endTime = time() - startTime

        return AlgorithmSubResult(path, distance[0], endTime)

    def dfs(
        self,
        currentNodeIndex: int,
        targetNodeIndex: int,
        visited: List[bool],
        path: List[int],
        distance: List[float],
        smallWorld: SmallWorld,
    ):
        visited[currentNodeIndex] = True
        path.append(currentNodeIndex)

        if currentNodeIndex == targetNodeIndex:
            return

        neighbors = self.smallWorld.adjacencyList[currentNodeIndex]
        for edge in neighbors:
            neighborIndex = edge.node2.id
            if not visited[neighborIndex]:
                distance[0] += edge.weight
                self.dfs(
                    neighborIndex, targetNodeIndex, visited, path, distance, smallWorld
                )
                if path[-1] == targetNodeIndex:
                    return
                distance[0] -= edge.weight

        # Backtrack if the target node is not found from the current node
        path.pop()

    def bfs(self, origin: int, destiny: int):
        visitedNodes = [False] * self.smallWorld.numberOfVertices
        path: List[int] = []
        queue: List[Tuple[Edge, float]] = []
        answer: Tuple[List[int], float] = None
        distance = 0

        startTime = time()
        queue.append((Edge(None, self.smallWorld.nodeList[origin], 0.0), 0.0))
        path.append(origin)
        visitedNodes[origin] = True

        while queue:
            nextTuple = queue.pop(0)
            nextIndex = nextTuple[0].node2.id
            nextDist = nextTuple[1]

            distance = nextDist

            path.append(nextIndex)

            if nextIndex == destiny:
                answer = path, distance
                break

            for edge in self.smallWorld.adjacencyList[nextIndex]:
                otherVerticeId = edge.node2.id
                if not visitedNodes[otherVerticeId]:
                    queue.append((edge, nextDist + edge.weight))
                    visitedNodes[otherVerticeId] = True

        endTime = time() - startTime

        if answer is None:
            return AlgorithmSubResult([], 0.0, endTime)

        return AlgorithmSubResult(path, distance, endTime)

    def bestFirstSearch(
        self, startNodeIndex: int, targetNodeIndex: int
    ) -> Tuple[List[int], float, float]:
        startTime = time()

        visited = [False] * self.smallWorld.numberOfVertices
        path = []
        distance = self._bestFirstSearch(
            startNodeIndex, targetNodeIndex, visited, path, self.smallWorld
        )

        endTime = time() - startTime

        return AlgorithmSubResult(path, distance, endTime)

    def _bestFirstSearch(
        self,
        startNodeIndex: int,
        targetNodeIndex: int,
        visited: List[bool],
        path: List[int],
        smallWorld: SmallWorld,
    ) -> float:
        priorityQueue = PriorityQueue()
        priorityQueue.put((0, startNodeIndex))  # (distance value, node index)

        while not priorityQueue.empty():
            distance, currentNodeIndex = priorityQueue.get()

            if visited[currentNodeIndex]:
                continue

            visited[currentNodeIndex] = True
            path.append(currentNodeIndex)

            if currentNodeIndex == targetNodeIndex:
                return distance

            neighbors = smallWorld.adjacencyList[currentNodeIndex]
            for edge in neighbors:
                neighborIndex = edge.node2.id
                if not visited[neighborIndex]:
                    priority = distance + edge.weight
                    priorityQueue.put((priority, neighborIndex))
        return 0.0

    def calculateEuclidian(
        self, smallWorld: SmallWorld, nodeIndex: int, targetNodeIndex: int
    ) -> float:
        node = smallWorld.nodeList[nodeIndex]
        targetNode = smallWorld.nodeList[targetNodeIndex]
        return node.geometricDistanceFromNode(targetNode)

    def calculateManhattan(
        self, smallWorld: SmallWorld, nodeIndex: int, targetNodeIndex: int
    ) -> float:
        node = smallWorld.nodeList[nodeIndex]
        targetNode = smallWorld.nodeList[targetNodeIndex]
        return node.manhattanDistanceFromNode(targetNode)

    def aStarEuclidian(
        self, startNodeIndex: int, targetNodeIndex: int
    ) -> Tuple[List[int], float, float]:
        visited = [False] * self.smallWorld.numberOfVertices
        path = []

        startTime = time()
        distance = self._aStar(
            self.calculateEuclidian,
            startNodeIndex,
            targetNodeIndex,
            visited,
            path,
            self.smallWorld,
        )
        endTime = time() - startTime

        return AlgorithmSubResult(path, distance, endTime)

    def aStarManhattan(
        self, startNodeIndex: int, targetNodeIndex: int
    ) -> Tuple[List[int], float, float]:
        visited = [False] * self.smallWorld.numberOfVertices
        path = []

        startTime = time()
        distance = self._aStar(
            self.calculateManhattan,
            startNodeIndex,
            targetNodeIndex,
            visited,
            path,
            self.smallWorld,
        )
        endTime = time() - startTime

        return AlgorithmSubResult(path, distance, endTime)

    def _aStar(
        self,
        heurFunction,
        startNodeIndex: int,
        targetNodeIndex: int,
        visited: List[bool],
        path: List[int],
        smallWorld: SmallWorld,
    ) -> float:
        priorityQueue = PriorityQueue()
        priorityQueue.put(
            (0, [startNodeIndex, 0])
        )  # (heuristic value, [node index, distance from origin])

        while not priorityQueue.empty():
            _, currentNode = priorityQueue.get()

            if visited[currentNode[0]]:
                continue

            visited[currentNode[0]] = True
            path.append(currentNode[0])

            if currentNode[0] == targetNodeIndex:
                return currentNode[1]

            neighbors = smallWorld.adjacencyList[currentNode[0]]
            for edge in neighbors:
                neighborIndex = edge.node2.id
                if not visited[neighborIndex]:
                    priority = (
                        currentNode[1]
                        + edge.weight
                        + heurFunction(smallWorld, neighborIndex, targetNodeIndex)
                    )
                    priorityQueue.put(
                        (priority, [neighborIndex, currentNode[1] + edge.weight])
                    )
        return 0.0
