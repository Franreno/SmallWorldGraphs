from typing import List, Tuple
import random
from algorithms import Algorithms
from small_world import SmallWorld
import sys

from vizualizer import Vizualizer


def main() -> None:
    sys.setrecursionlimit(int(10e4))
    print("Criando grafo 10%")
    sm10Percent = SmallWorld(2000, 7, 0.1)
    print("Criando grafo 5%")
    sm5Percent = SmallWorld(2000, 7, 0.05)
    print("Criando grafo 1%")
    sm1Percent = SmallWorld(2000, 7, 0.01)

    # p = 0.1
    originNodes10Percent, destinyNodes10Percent = createListOfNodesFromGraph(
        10, sm10Percent
    )
    print("\nRodando algoritmos para (n=2000, k=7, p=0.1) [10%]")
    print(f"Nos de origem: {originNodes10Percent}")
    print(f"Nos de destino: {destinyNodes10Percent}")
    algorithms10Percent = Algorithms(
        originNodes10Percent, destinyNodes10Percent, sm10Percent
    )
    results10Percent = algorithms10Percent.runAllAlgorithms()
    print(results10Percent)

    # p = 0.05
    originNodes5Percent, destinyNodes5Percent = createListOfNodesFromGraph(
        10, sm5Percent
    )
    print("\nRodando algoritmos para (n=2000, k=7, p=0.05) [5%]")
    print(f"Nos de origem: {originNodes5Percent}")
    print(f"Nos de destino: {destinyNodes5Percent}")
    algorithms5Percent = Algorithms(
        originNodes5Percent, destinyNodes5Percent, sm5Percent
    )
    results5Percent = algorithms5Percent.runAllAlgorithms()
    print(results5Percent)

    # p = 0.01
    originNodes1Percent, destinyNodes1Percent = createListOfNodesFromGraph(
        10, sm1Percent
    )
    print("\nRodando algoritmos para (n=2000, k=7, p=0.01) [1%]")
    print(f"Nos de origem: {originNodes1Percent}")
    print(f"Nos de destino: {destinyNodes1Percent}")
    algorithms1Percent = Algorithms(
        originNodes1Percent, destinyNodes1Percent, sm1Percent
    )
    results1Percent = algorithms1Percent.runAllAlgorithms()
    print(results1Percent)


def createListOfNodesFromGraph(
    amountOfNodes: int, sm: SmallWorld
) -> Tuple[List[int], List[int]]:
    testAlgorithms = Algorithms([], [], sm)

    originNodes = []
    destinyNodes = []
    for _ in range(amountOfNodes):
        randomNode1 = random.randint(0, sm.numberOfVertices)
        randomNode2 = random.randint(0, sm.numberOfVertices)
        # Use the algorithm to check if there is a path between these two nodes
        success = False
        while not success:
            result = testAlgorithms.bfs(randomNode1, randomNode2)
            if result.distance != 0.0:
                success = True
                break

            randomNode1 = random.randint(0, sm.numberOfVertices)
            randomNode2 = random.randint(0, sm.numberOfVertices)

        originNodes.append(randomNode1)
        destinyNodes.append(randomNode2)

    return originNodes, destinyNodes


if __name__ == "__main__":
    main()
