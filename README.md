# MaxFlow


The maximum flow problem is an optimization problem seeking the feasible flow through a single-source, single-sink flow network. A flow network G=(V, E) is a directed graph where each edge (u,v) in the graph, has a capacity (c >=0 ). A flow network must follow these properties; there are no self-loops, there are a single source and a single sink, and there is at least a single path between the source to the sink. The flow in the network is bounded within the constraints of the total capacity of the network.   
  
The Ford-Fulkerson method to solve maximum flow problem follows this general pseudo-code (Cormen, ):  
  
FORD-FULKERSON-METHOD (G, s, t)  
1  initialize flow f to 0  
2  while there exists an augmenting path p in the residual network G_f  
3     augment flow f along p  
4  return f  
  
The algorithm keeps augmenting to the network flow until the residual network has no more augmenting paths. There are many implementations of this method. The naive implementation runs in complexity O(E|f|); hence, assuming a worst case of an addition of one unit of flow per iteration in which an update of the flow in each edge is occurring. This implementation is not very sustainable in large networks.     
    
A use of breadth-first search reduces the dependency on the value of the flow and creates a polynomial solution. The first improvement over the naive model was created by Edmonds-Karp (1972) and runs in complexity O(VE^2), another variation was found by Dinic (1970) running O(EV^2). The emphasis on the vertices performs better in dense graphs. The Dinic method set the grounds for the Push Relabel method, running in O(V^3). The following section explains the operation of each one of these algorithms.  
  
### Edmonds-Karp  
  This algorithm uses a BFS to fins in each iteration the shortest augmenting path between the source and the sink. In each iteration, the bounding capacity is calculated, and a suitable amount of flow is being sent back along the augmented path. Doing so until the residual network is exhausted.The max flow is calculated by adding each iteration path flow.
  
   Each path contains at least one edge. A single BFS runs O(E), and push along the path is in its worst case O(E). Since the length of the paths never decreases (as promised by the use of BFS) and the length of a path can never exceed V, each edge can be found V times. Leading to an overall worse case running O(VE^2). 
     
![picture](https://github.com/karnigili/MaxFlow/blob/master/illustrations/EdmondKarp.png)

### Dinic's
  The algorithm uses BFS to mark the nodes' distance from the source. Progress in a path can be made only in edges (u,v) where the distance to the source [u-s] is smaller than the distance [v-s] (each distance is bounded by the distance from the source s to the sink t). These distances reduce the number of edges in the graph. Each iteration finds x paths with a uniform distance to the source and using a single DFS to calculate the bounding flow for these paths. Similarly, the calculated flow is sent back, and the process repeats. Once all possible edges where used in a given vertex (meaning, it can not reach the sink anymore), it is being marked as dead as is taken off the residual network. The loop stops when the residual network is exhausted. 


  The longest path possible from the source to the sink is V, so the number of iterations is bounded by O(V). Each iteration, as stated above, computes the flow using a single DFS (O(E)), then sends the relevant flow back and marks the exhausted vertices (bound by the number of vertices O(V)). Resulting in an inner complexity of O(VE) leading to a total complexity of O(EV^2).  

![picture](https://github.com/karnigili/MaxFlow/blob/master/illustrations/Dinic.png)

### Push Relabel
  This algorithm optimizes using local push of excess flow of each vertex from source to the edges in the graph. Using two main operations; push and relabel. (1) Push: moving flow from u to v bounded by the residual capacity of the edge (u,v). A push is applicable only is the residual is positive, and the distance [u-s] is less than [v-s]. (2) A relabel operation ensures a continuation of these conditions. Initially, the source sends its flow, possibility creating excess flow in its neighbors; if the excess flow is not pushable, a relabeling occurs, and the process repeats until the vertex's flow is within capacity. The relabeling process can move a vertex's distance only one unit above its neighbors at each iteration. The algorithm keeps track on active nodes (with the excess flow) by keep pushing them to the front of the queue.  
  
  The amount of discharging pushes that is being performed is at most 2VE, due to the existential relabel between a discharging push to a new non-discharging push. The probability of each vertex to appear again in the discharging queue before discharging to be V^2, running for each vertex, leads to V^3 non-discharging pushes. Using the lemma saying that V<=E<=V^2, the conclusion of the total complexity O(V^3+VE) is equal to O(V^3).  


![picture](https://github.com/karnigili/MaxFlow/blob/master/illustrations/PushRelabel.png)

Inspired by 

Cormen, T., Leiseron, C., Rivest, R. and Stein, C. (2017). Introduction to algorithms. 3rd ed.

Friis, J. (2014). An experimental comparison of max flow algorithms. [online] Available at: http://An experimental comparison of max flow algorithms.

## Getting Started

Follow [example.py](example.py) for a demonstration of the functions

### Prerequisites

```
import numpy as np
```


## Authors

* **Gili Karni** 


## License

This project is licensed under the MIT License - see the [LICENSE.txt](LICENSE.txt) file for details

