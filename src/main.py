from algorithms import Algorithms
from small_world import SmallWorld


def main() -> None:
    sm = SmallWorld(100, 4, 0.1)
    print(sm.toString())

    originNodes = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    destinyNodes = [12, 13, 24, 35, 42, 54, 66, 73, 81, 93]

    algorithms = Algorithms(originNodes, destinyNodes, sm)
    # results = algorithms.runAllAlgorithms()


if __name__ == "__main__":
    main()
