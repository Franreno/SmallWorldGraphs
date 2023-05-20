from node import Node


class Edge:
    node1: Node
    node2: Node
    weight: float

    def __init__(self, _node1: Node, _node2: Node, _dist: float) -> None:
        self.node1 = _node1
        self.node2 = _node2
        self.weight = _dist

    def reconnect(self, reconnectNode: Node) -> None:
        """Reconnect `node2` to `reconnectNode`

        Args:
            reconnectNode (Node): The node that will be reconnected to `node1`
        """
        self.node2 = reconnectNode
        self.weight = self.node1.geometricDistanceFromNode(self.node2)

    def toString(self) -> str:
        return f"Node1: {self.node1.toString()}, Node2: {self.node2.toString()}, weight: {self.weight}"
