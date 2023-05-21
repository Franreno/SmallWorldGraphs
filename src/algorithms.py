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

    def calculateValues(self, results: List[AlgorithmSubResult]):
        sumDistances = 0
        sumTimes = 0
        for result in results:
            sumDistances += result.distance
            sumTimes += result.timeUsed

        self.meanDistance = sumDistances / len(results)
        self.meanTime = sumTimes / len(results)

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
            algorithmsSubResults.append(self.aStar(origin, destiny))

        aStarAlgorithmResult = AlgorithmResult().calculateValues(algorithmsSubResults)

        return {
            "dfsAlgorithmResult": f"Mean Time: {dfsAlgorithmResult[1]} ; Mean Distance: {dfsAlgorithmResult[0]}",
            "bfsAlgorithmResult": bfsAlgorithmResult,
            "bestFirstAlgorithmResult": f"Mean TIme: {bestFirstAlgorithmResult[1]} ; Mean Distance: {bestFirstAlgorithmResult[0]}",
            "aStarAlgorithmResult": aStarAlgorithmResult,
        }

    def depthFirstSearch(self, startNodeIndex: int, targetNodeIndex: int) -> Tuple[List[int], float, float]:
        startTime = time()
        visited = [False] * self.smallWorld.numberOfVertices
        path = []
        distance = []
        distance.append(0.0)

        self.dfs(startNodeIndex, targetNodeIndex, visited, path, distance, self.smallWorld)

        endTime = time() - startTime

        return AlgorithmSubResult(path, distance[0], endTime)

    def dfs(
        self,
        currentNodeIndex: int,
        targetNodeIndex: int,
        visited: List[bool],
        path: List[int],
        distance: List[float],
        smallWorld: SmallWorld
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
                self.dfs(neighborIndex, targetNodeIndex, visited, path, distance,smallWorld)
                if path[-1] == targetNodeIndex:
                    return
                distance[0] -= edge.weight

        # Backtrack if the target node is not found from the current node
        path.pop()
        

    def bfs(self, origin: int, destiny: int):
        startTime = time()
        # Algorithm
        endTime = time() - startTime

        return AlgorithmSubResult([], 0, endTime)

    def bestFirstSearch(self,startNodeIndex: int, targetNodeIndex: int) -> Tuple[List[int], float, float]:
        startTime = time()

        visited = [False] * self.smallWorld.numberOfVertices
        path = []
        distance = self._bestFirstSearch(startNodeIndex, targetNodeIndex, visited, path, self.smallWorld)

        endTime = time() - startTime

        return AlgorithmSubResult(path, distance, endTime)

    def _bestFirstSearch(
        self,
        startNodeIndex: int,
        targetNodeIndex: int,
        visited: List[bool],
        path: List[int],
        smallWorld: SmallWorld
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
                return self.calculatePathDistance(smallWorld, path)

            neighbors = smallWorld.adjacencyList[currentNodeIndex]
            for edge in neighbors:
                neighborIndex = edge.node2.id
                if not visited[neighborIndex]:
                    priority = self.calculateHeuristic(smallWorld ,neighborIndex, targetNodeIndex)
                    priorityQueue.put((priority, neighborIndex))

        return 0.0

    def calculateHeuristic(self, smallWorld: SmallWorld,nodeIndex: int, targetNodeIndex: int) -> float:
        node = smallWorld.nodeList[nodeIndex]
        targetNode = smallWorld.nodeList[targetNodeIndex]
        return node.geometricDistanceFromNode(targetNode)

    def calculatePathDistance(self, smallWorld: SmallWorld,path: List[int]) -> float:
        distance = 0.0
        for i in range(len(path) - 1):
            currentNodeIndex = path[i]
            nextNodeIndex = path[i + 1]
            neighbors = smallWorld.adjacencyList[currentNodeIndex]
            for edge in neighbors:
                if edge.node2.id == nextNodeIndex:
                    distance += edge.weight
                    break

        return distance

    def aStar(self, origin: int, destiny: int):
        startTime = time()
        # Algorithm
        endTime = time() - startTime

        return AlgorithmSubResult([], 0, endTime)
