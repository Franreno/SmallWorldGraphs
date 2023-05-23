import random
from algorithms import Algorithms
from small_world import SmallWorld
import sys

from vizualizer import Vizualizer


def main() -> None:
    sys.setrecursionlimit(int(10e4))
    sm = SmallWorld(100, 4, 0.1)

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
    viz.visualize("Astar manhattan", result.path, rand1, rand2)


if __name__ == "__main__":
    main()
