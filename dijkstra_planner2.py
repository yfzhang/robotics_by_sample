from collections import defaultdict
from heapq import *

def dijkstra(edges, start, goal):
    graph = defaultdict(list)
    for l,r,c in edges:
        graph[l].append((c,r))

    q, seen = [(0, start, ())], set()
    while q:
        (cost,v1,path) = heappop(q)
        if v1 not in seen:
            seen.add(v1)
            path = (v1, path)
            if v1 == goal: return (cost, path)

            for c, v2 in graph.get(v1, ()):
                if v2 not in seen:
                    heappush(q, (cost+c, v2, path))

    return float("inf")

if __name__ == "__main__":
    edges = [
        ("A", "B", 7),
        ("A", "D", 5),
        ("B", "C", 8),
        ("B", "D", 9),
        ("B", "E", 7),
        ("C", "E", 5),
        ("D", "E", 15),
        ("D", "F", 6),
        ("E", "F", 8),
        ("E", "G", 9),
        ("F", "G", 11)
    ]

    print "=== Dijkstra ==="
    print edges
    print "A -> E:"
    print dijkstra(edges, "A", "E")
    print "F -> G:"
    print dijkstra(edges, "F", "G")