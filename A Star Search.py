# A* Search By Md.Shafiul Haque Johny

import heapq


class Graph:

    def init(self):

        self.edges = {}

    def neighbours(self, edge_id):

        return self.edges[edge_id]


class PriorityQueue:

    def __init__(self):

        self.elements = []

    def empty(self):

        return len(self.elements) == 0

    def put(self, item, priority):

        heapq.heappush(self.elements, (priority, item))

    def get(self):

        return heapq.heappop(self.elements)[1]


def A_Star_Search(graph, source, destination, heuristic):

    minHeap = PriorityQueue()
    minHeap.put(source, 0)
    came_from = {}
    cost_so_far = {}
    came_from[source] = None
    cost_so_far[source] = 0

    while not minHeap.empty():

        N = minHeap.get()

        if N == destination:
            break

        for node in graph.neighbours[N]:

            next_vertex = node[0]
            edge_weight = node[1]
            new_cost = cost_so_far[N] + edge_weight

            if next_vertex not in cost_so_far or new_cost < cost_so_far[next_vertex]:

                cost_so_far[next_vertex] = new_cost
                priority = new_cost + heuristic[next_vertex]
                minHeap.put(next_vertex, priority)
                came_from[next_vertex] = N

    return came_from, cost_so_far


def print_path(came_from, Destination):

  N_node = Destination

  if came_from[N_node] != None:

    print_path(came_from, came_from[N_node])

  if N_node != 'D':

    print(N_node, "->", end=' ')

  else:

    print(N_node)


if __name__ == "__main__":

  inputGraph = Graph()

  inputGraph.neighbours = \
  {
    'S': [('A', 1), ('B', 4)],
    'A': [('D', 12), ('B', 2), ('C', 5)],
    'B': [('C', 2)],
    'C': [('D', 3)],
  }

  heuristic = \
  {
      'S': 7,
      'A': 6,
      'B': 2,
      'C': 1,
      'D': 0,
  }

  (came_from, cost_so_far) = A_Star_Search(inputGraph, 'S', 'D', heuristic)

  print("The optimal cost to reach destination is", cost_so_far['D'], "units")

  print("The optimal path is : ")

  print_path(came_from, 'D')












