from math import sqrt


class Node:
    """Node class for the graph"""

    id: int
    xCord: float
    yCord: float

    def __init__(self, _id, _xCord, _yCord) -> None:
        self.id = _id
        self.xCord = _xCord
        self.yCord = _yCord

    def geometricDistanceFromNode(self, node2: "Node") -> float:
        """Returns the euclidian distance from self node and another node.

        Args:
            node2 (Node): The second node

        Returns:
            float: The euclidian distance between nodes
        """
        return sqrt((node2.xCord - self.xCord) ** 2 + (node2.yCord - self.yCord) ** 2)

    def manhattanDistanceFromNode(self, node2: "Node") -> float:
        """Returns the manhattan distance from self node and another node.

        Args:
            node2 (Node): The second node

        Returns:
            float: The euclidian distance between nodes
        """
        return abs(node2.xCord - self.xCord) + abs(node2.yCord - self.yCord)

    def getXandY(self) -> tuple[int, int]:
        return self.xCord, self.yCord

    def toString(self) -> str:
        return f"id: {self.id}, x: {self.xCord}, y: {self.yCord}"
