from collections import deque
from heapq import heappush, heappop 
from functools import reduce

def shortest_shortest_path(graph, source):
  def shortest_helper(visited, frontier, depth):
    if len(frontier) == 0:
      return visited
    else:
      distance, depth, node = heappop(frontier)
      if node in visited:
        return shortest_helper(visited, frontier, depth)
      else:
        visited[node] = distance, depth
        for neighbor, weight in graph[node]:
          heappush(frontier, (distance + weight, depth + 1, neighbor))                
        return shortest_helper(visited, frontier, 0)
        
  frontier = []
  heappush(frontier, (0, 0, source))
  visited = dict()
  return shortest_helper(visited, frontier, 0)
  """
    Params: 
      graph.....a graph represented as a dict where each key is a vertex
                and the value is a set of (vertex, weight) tuples (as in the test case)
      source....the source node
      
    Returns:
      a dict where each key is a vertex and the value is a tuple of
      (shortest path weight, shortest path number of edges). See test case for example.
    """

    
    
def test_shortest_shortest_path():

    graph = {
                's': {('a', 1), ('c', 4)},
                'a': {('b', 2)}, # 'a': {'b'},
                'b': {('c', 1), ('d', 4)}, 
                'c': {('d', 3)},
                'd': {},
                'e': {('d', 0)}
            }
    result = shortest_shortest_path(graph, 's')
    # result has both the weight and number of edges in the shortest shortest path
    assert result['s'] == (0,0)
    assert result['a'] == (1,1)
    assert result['b'] == (3,2)
    assert result['c'] == (4,1)
    assert result['d'] == (7,2)
    
    
def bfs_path(graph, source):
  def bfs_helper_depths(visited, frontier, cur_depth, depths, parents, prev_node):
    if len(frontier) == 0:
      return parents
    else:
      visited_new = visited | frontier
      for v in visited_new - visited:
        curr_node = v
        depths[v] = cur_depth
        parents[v] = prev_node
      visited = visited_new 
      frontier_neighbors = reduce(set.union, [graph[f] for f in frontier])
      frontier = set(frontier_neighbors) - visited
      prev_node = curr_node
      
      return bfs_helper_depths(visited, frontier, cur_depth+1, depths, parents, prev_node)    

  depths = dict()
  visited = set()
  frontier = set([source])
  parents = dict()
  return bfs_helper_depths(visited, frontier, 0, depths, parents, source)
  """
    Returns:
      a dict where each key is a vertex and the value is the parent of 
      that vertex in the shortest path tree.
  """

def get_sample_graph():
     return {'s': {'a', 'b'},
            'a': {'b'},
            'b': {'c'},
            'c': {'a', 'd'},
            'd': {}
            }

def test_bfs_path():
    graph = get_sample_graph()
    parents = bfs_path(graph, 's')
    assert parents['a'] == 's'
    assert parents['b'] == 's'    
    assert parents['c'] == 'b'
    assert parents['d'] == 'c'
    
def get_path(parents, destination):
  source = 's'
  def get_path_helper(parents, destination, previous):
    if parents[destination] == source:
      return source + previous
    else:
      parent = parents[destination]
      previous = parent + previous
      return get_path_helper(parents, parent, previous)

  previous = ''
  return get_path_helper(parents, destination, previous)
    
  """
    Returns:
      The shortest path from the source node to this destination node 
      (excluding the destination node itself). See test_get_path for an example.
  """

def test_get_path():
    graph = get_sample_graph()
    parents = bfs_path(graph, 's')
    assert get_path(parents, 'd') == 'sbc'
