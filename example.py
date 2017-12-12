### example ###

from graph_max_flow import *

invalid_data = np.array([[0, 10, 0, 8, 0, 0],
                [1, 0, 5, 2, 0, 0],
                [0, 0, 0, 0, 0, 7],
                [0, 0, 0, 0, 10, 0],
                [0, 0, 8, 0, 0, 10],
                [0, 0, 0, 0, 0, 0]])

# invalid_attempt = Graph(invalid_data)

valid_data = np.array([[0, 10, 0, 8, 0, 0],
                [0, 0, 5, 2, 0, 0],
                [0, 0, 0, 0, 0, 7],
                [0, 0, 0, 0, 10, 0],
                [0, 0, 8, 0, 0, 10],
                [0, 0, 0, 0, 0, 0]])

EdmondKarp_graph = Graph(valid_data)
Dinic_graph = Graph(valid_data)
PushRelable_graph = Graph(valid_data)

print "Max flow for the given valid graph."

print "\nmax flow using Edmond Karp :"
print EdmondKarp_graph.EdmondKarp()
print "\nmax flow using Dinic :"
print Dinic_graph.Dinic()
print "\nmax flow using Push Relable :"
print PushRelable_graph.PushRelable()