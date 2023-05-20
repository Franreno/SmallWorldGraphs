from node import Node
from small_world import SmallWorld
from time import time


class AlgorithmSubResult:
    path: list[Node]
    distance: float
    timeUsed: float

    def __init__(
        self,
        _path: list[Node],
        _distance: float,
        _timeUsed: float,
    ) -> None:
        self.path = _path
        self.distance = _distance
        self.timeUsed = _timeUsed


class AlgorithmResult:
    meanDistance: float
    meanTime: float

    def calculateValues(self, results: list[AlgorithmSubResult]):
        sumDistances = 0
        sumTimes = 0
        for result in results:
            sumDistances += result.distance
            sumTimes += result.timeUsed

        self.meanDistance = sumDistances / len(result)
        self.meanTime = sumTimes / len(result)


class Algorithms:
    listOfOriginNodes: list[int]
    listOfDestinyNodes: list[int]
    smallWorld: SmallWorld

    def __init__(
        self,
        _listOfOriginNodes: list[int],
        _listOfDestinyNodes: list[int],
        _smallWorld: SmallWorld,
    ) -> None:
        self.destiny = _listOfOriginNodes
        self.listOfDestinyNodes = _listOfDestinyNodes
        self.smallWorld = _smallWorld

    def runAllAlgorithms(self) -> dict[str, AlgorithmResult]:
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
