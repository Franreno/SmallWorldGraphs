import random
from algorithms import Algorithms
from small_world import SmallWorld
import sys

from vizualizer import Vizualizer


def main() -> None:
    sys.setrecursionlimit(int(10e4))
    sm = SmallWorld(100, 4, 0.1)

    # Creating random starting nodes and

    originNodes = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    destinyNodes = [12, 13, 24, 35, 42, 54, 66, 73, 81, 93]

    algorithms = Algorithms(originNodes, destinyNodes, sm)

    # print(algorithms.runAllAlgorithms())
    # results = algorithms.runAllAlgorithms()

    viz = Vizualizer(sm)

    sucess = False
    result = None
    rand1 = None
    rand2 = None
    while not sucess:
        rand1 = random.randint(0, sm.numberOfVertices)
        rand2 = random.randint(0, sm.numberOfVertices)
        result = algorithms.aStarManhattan(rand1, rand2)
        if result.distance != 0.0:
            sucess = True

    print(f"rodei para {rand1} e {rand2}")
    bfsResult = algorithms.bfs(rand1, rand2)
    dfsResult = algorithms.depthFirstSearch(rand1, rand2)
    bestFS = algorithms.bestFirstSearch(rand1, rand2)
    astarEuclidianResult = algorithms.aStarEuclidian(rand1, rand2)
    astarManhattanResult = algorithms.aStarManhattan(rand1, rand2)

    viz.visualize(
        "Breadth first search",
        bfsResult.path,
        rand1,
        rand2,
        bfsResult.distance,
        "images/SameGraph/BFS_",
    )

    viz.visualize(
        "Depth first search",
        dfsResult.path,
        rand1,
        rand2,
        dfsResult.distance,
        "images/SameGraph/DFS_",
    )

    viz.visualize(
        "Best first search",
        bestFS.path,
        rand1,
        rand2,
        bestFS.distance,
        "images/SameGraph/BestFS_",
    )

    viz.visualize(
        "Astar euclidian",
        astarEuclidianResult.path,
        rand1,
        rand2,
        astarEuclidianResult.distance,
        "images/SameGraph/aStarEuclidian_",
    )

    viz.visualize(
        "Astar manhattan",
        astarManhattanResult.path,
        rand1,
        rand2,
        astarManhattanResult.distance,
        "images/SameGraph/aStarManhattan_",
    )


if __name__ == "__main__":
    main()
