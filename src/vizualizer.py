import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import plotly.graph_objects as go
from typing import List, Dict, Tuple
from node import Node


from small_world import SmallWorld


class Vizualizer:
    nxGraph: nx.Graph
    smallWorld: SmallWorld

    def __init__(self, _sm: SmallWorld) -> None:
        self.smallWorld = _sm
        self.nxGraph = self._createGraphFromSmallWorld()

    def _createGraphFromSmallWorld(self):
        G = nx.Graph()

        for node in self.smallWorld.nodeList:
            G.add_node(node.id, data=node)

        for edges in self.smallWorld.adjacencyList:
            for edge in edges:
                G.add_edge(edge.node1.id, edge.node2.id)

        return G

    def visualize(
        self,
        algorithmTitle: str,
        path: List[int],
        origin: int,
        destiny: int,
        distance: float,
        pathToSave: str,
    ):
        G = self.nxGraph

        node_x = []
        node_y = []
        for node in G.nodes():
            x, y = G.nodes[node]["data"].getXandY()
            node_x.append(x)
            node_y.append(y)

        figDict = dict(
            # data=[],
            layout=dict(),
            frames=[],
        )

        slider = dict(
            active=0,
            yanchor="top",
            xanchor="left",
            currentvalue=dict(
                font=dict(size=20), prefix="Passo: ", visible=True, xanchor="right"
            ),
            transition=dict(duration=500, easing="cubic-in-out"),
            pad=dict(b=10, t=20),
            len=0.9,
            x=0.1,
            y=0,
            steps=[],
        )

        titleStr = f"Passos para o algoritmo {algorithmTitle}. ({origin} -> {destiny}) Dist: {str(distance)[0:4]}"
        figDict["layout"]["hovermode"] = "closest"
        figDict["layout"]["showlegend"] = False
        figDict["layout"]["title"] = titleStr

        playButton = dict(
            args=[
                None,
                dict(
                    frame=dict(duration=500, redraw=False),
                    fromcurrent=True,
                    transition=dict(duration=500, easing="quadratic-in-out"),
                ),
            ],
            label="Play",
            method="animate",
        )

        pauseButton = dict(
            args=[
                [None],
                dict(
                    frame=dict(duration=0, redraw=False),
                    mode="immediate",
                    transition=dict(duration=0),
                ),
            ],
            label="Pause",
            method="animate",
        )

        figDict["layout"]["updatemenus"] = [
            dict(
                buttons=[playButton, pauseButton],
                direction="left",
                pad=dict(r=10, t=87),
                showactive=True,
                type="buttons",
                x=0.1,
                xanchor="right",
                y=0,
                yanchor="top",
            )
        ]

        figDict["layout"]["sliders"] = [slider]

        nodeTraces = {}
        edgeTraces = {}

        graphEdges = self.nxGraph.edges()
        nodesLoc = []
        for node in self.nxGraph.nodes():
            x, y = self.nxGraph.nodes[node]["data"].getXandY()
            nodesLoc.append([x, y])

        for i in range(len(path) + 1):
            nodeTraces[i] = self.createNodeTrace(nodesLoc, path, i, origin, destiny)

        for i in range(len(path) + 1):
            edgeTraces[i] = self.createEdgeTraces(graphEdges, nodesLoc, path, i)

        figDict["data"] = edgeTraces[0] + [nodeTraces[0]]

        for i in range(len(path) + 1):
            frame = dict(data=edgeTraces[i] + [nodeTraces[i]], name=str(i))

            figDict["frames"].append(frame)

            slider_step = dict(
                args=[
                    [i],
                    dict(
                        frame=dict(duration=500, redraw=False),
                        mode="immediate",
                        transition=dict(duration=500),
                    ),
                ],
                label=str(i),
                method="animate",
            )
            slider["steps"].append(slider_step)

        fig = go.Figure(figDict)

        fig.show()
        fig.write_html(pathToSave + f"{origin}_{destiny}.html")

    def createNodeTrace(
        self,
        nodesLoc: List[List[int]],
        path: List[int],
        pathStep: int,
        origin: int,
        destiny: int,
    ):
        nodes = list(self.nxGraph.nodes())
        colorMark = ["#000000"] * len(nodes)
        sizes = [10] * len(nodes)
        nodesLoc2 = np.array(nodesLoc)

        for i in range(len(path[0:pathStep])):
            colorMark[path[i]] = "#ff0000"
            sizes[path[i]] = 20

        colorMark[nodes[origin]] = "#0000ff"
        colorMark[nodes[destiny]] = "#00ff00"

        node_trace = dict(
            type="scatter",
            x=nodesLoc2[:, 0],
            y=nodesLoc2[:, 1],
            mode="markers",
            hoverinfo="text",
            marker=dict(color=colorMark, size=sizes),
            text=self.createNodeMarkerAndText(),
        )

        return node_trace

    def createEdgeTraces(
        self,
        edges: List[Tuple[int, int]],
        nodesLoc: List[List[int]],
        path: List[int],
        pathStep: int,
    ):
        width = [0.5] * len(edges)
        markers = ["#000000"] * len(edges)
        subList = path[0:pathStep]

        # From here subList is even.
        indexThatNeedToBeMarked = []
        edgesList = list(edges)
        for i in range(0, len(subList)):
            if i + 2 > len(subList):
                break

            pathNode1 = subList[i]
            pathNode2 = subList[i + 1]
            locIndex = None

            for index, edge in enumerate(edgesList):
                if edge[0] == pathNode1 and edge[1] == pathNode2:
                    locIndex = index
                elif edge[0] == pathNode2 and edge[1] == pathNode1:
                    locIndex = index

            # Find if there is a connection on edgeList with another member of the subList
            if locIndex is None:
                for otherMember in subList:
                    for index, edge in enumerate(edgesList):
                        if (edge[0] == pathNode2 and edge[1] == otherMember) or (
                            edge[0] == otherMember and edge[1] == pathNode2
                        ):
                            locIndex = index
                    if locIndex != None:
                        break

            indexThatNeedToBeMarked.append(locIndex)

        # Ensure that the last two are marked
        if len(subList) not in (0, 1):
            lastNode = subList[-1]
            lastToLastNode = subList[-2]

            locIndex = None

            for index, edge in enumerate(edgesList):
                if edge[0] == lastNode and edge[1] == lastToLastNode:
                    locIndex = index
                elif edge[0] == lastToLastNode and edge[1] == lastNode:
                    locIndex = index

            # Find if there is a connection on edgeList with another member of the subList
            if locIndex is None:
                for otherMember in subList:
                    for index, edge in enumerate(edgesList):
                        if (edge[0] == lastNode and edge[1] == otherMember) or (
                            edge[0] == otherMember and edge[1] == lastNode
                        ):
                            locIndex = index
                    if locIndex != None:
                        break

            if locIndex is not None and locIndex not in indexThatNeedToBeMarked:
                indexThatNeedToBeMarked.append(locIndex)

        for index in indexThatNeedToBeMarked:
            markers[index] = "#ff0000"
            width[index] = 2.0

        edges_list = [
            dict(
                type="scatter",
                x=[nodesLoc[e[0]][0], nodesLoc[e[1]][0]],
                y=[nodesLoc[e[0]][1], nodesLoc[e[1]][1]],
                mode="lines",
                line=dict(width=width[k], color=markers[k]),
            )
            for k, e in enumerate(edges)
        ]

        return edges_list

    def createNodeMarkerAndText(self):
        G = self.nxGraph
        node_text = []
        for node, adjacencies in enumerate(G.adjacency()):
            node_text.append(f"Id: {G.nodes[node]['data'].id}")

        return node_text
