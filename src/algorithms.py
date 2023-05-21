from node import Node
from small_world import SmallWorld
from time import time
from typing import List, Dict


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

        self.meanDistance = sumDistances / len(result)
        self.meanTime = sumTimes / len(result)


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
        self.destiny = _listOfOriginNodes
        self.listOfDestinyNodes = _listOfDestinyNodes
        self.smallWorld = _smallWorld

    def runAllAlgorithms(self) -> Dict[str, AlgorithmResult]:
        algorithmsSubResults = []
        for origin, destiny in zip(self.listOfOriginNodes, self.listOfDestinyNodes):
            algorithmsSubResults.append(self.dfs(origin, destiny))

        dfsAlgorithmResult = AlgorithmResult().calculateValues(algorithmsSubResults)

        algorithmsSubResults = []
        for origin, destiny in zip(self.listOfOriginNodes, self.listOfDestinyNodes):
            algorithmsSubResults.append(self.bfs(origin, destiny))

        bfsAlgorithmResult = AlgorithmResult().calculateValues(algorithmsSubResults)

        algorithmsSubResults = []
        for origin, destiny in zip(self.listOfOriginNodes, self.listOfDestinyNodes):
            algorithmsSubResults.append(self.bestFirst(origin, destiny))

        bestFirstAlgorithmResult = AlgorithmResult().calculateValues(
            algorithmsSubResults
        )

        algorithmsSubResults = []
        for origin, destiny in zip(self.listOfOriginNodes, self.listOfDestinyNodes):
            algorithmsSubResults.append(self.aStar(origin, destiny))

        aStarAlgorithmResult = AlgorithmResult().calculateValues(algorithmsSubResults)

        return {
            "dfsAlgorithmResult": dfsAlgorithmResult,
            "bfsAlgorithmResult": bfsAlgorithmResult,
            "bestFirstAlgorithmResult": bestFirstAlgorithmResult,
            "aStarAlgorithmResult": aStarAlgorithmResult,
        }

    def dfs(self, origin: int, destiny: int):
        startTime = time()
        # Algorithm
        endTime = time() - startTime

        return AlgorithmSubResult([], 0, endTime)

    def bfs(self, origin: int, destiny: int):
        startTime = time()
        # Algorithm
        endTime = time() - startTime

        return AlgorithmSubResult([], 0, endTime)

    def bestFirst(self, origin: int, destiny: int):
        startTime = time()
        # Algorithm
        endTime = time() - startTime

        return AlgorithmSubResult([], 0, endTime)

    def aStar(self, origin: int, destiny: int):
        startTime = time()
        # Algorithm
        endTime = time() - startTime

        return AlgorithmSubResult([], 0, endTime)
