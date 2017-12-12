import numpy as np


class Graph (object):
    '''
    A flow graph object

    Calculates max flow using different methods
    '''

    def __init__(self, data=None):
        n, m = np.shape(data)
        assert n == m, "data must be square"

        # edge properties
        self.capacity = data
        self.flow = np.zeros((n, n), dtype=np.int)

        self.size = n

        # vertices properties
        self.vertices = np.array([i for i in range(self.size)])
        self.excess = [0] * self.size
        self.distance = [0] * self.size
        self.level = [-1] * self.size
        self.seen = [0] * self.size


        assert sum(self.capacity[-1]) == 0 and sum(self.capacity[:, 0]) \
        == 0 , "source must be first, sink must be last"

        self.source = 0
        self.sink = self.size - 1

    def _search(self, traverse, origin=None, goal=None, DFS=False):
        '''
        objective : find an augmented path between given origin and goal.
        uses DFS or BFS

        parameters :
            self - object
            traverse - a 1D array, holds the traverse of the path,
                traverse[successor] = current
            origin - int, an index for the origin, the default is the source
            goal - int, an index for the goal, the default is sink
            DFS - bool, True will search depth-first and False will
                search breadth-first.
            the default is BFS

        output
            True or False; indicator whether a path exists

        '''

        # set path origin and goal indexes
        goal = goal or self.sink
        origin = origin or self.source

        # avoid double visit
        visited = [False] * self.size

        # a queue that states the next indexes to visit, starting with origin
        to_visit = [origin]

        # mark origin as visited
        visited[origin] = True

        # to determine the edge; predecessor:u 
        predecessor = origin

        while to_visit:

            # pops the next index to visit, base of the search method chosen
            if DFS:
                current = to_visit.pop(0)
            # BFS
            else:  
                current = to_visit.pop()

            # iterates on all possible steps
            for successor in range(self.size):

                # a viable next step is one that was not visited and that its 
                # residual is possitive
                if (not visited[successor]) and \
                (self.capacity[(current, successor)] \
                    - self.flow[(current, successor)] > 0):

                    # mark as visited and add to the queue to explore
                    visited[successor] = True
                    to_visit.append(successor)

                    # adds to the traverse
                    traverse[successor] = current

        return visited[goal]

    def _max_flow_search_FF(self, origin=None, goal=None, data=False,\
     DFS=False):
        '''
        objective : finds the max flow for a given graph

        parameters :
            self - object
            origin - int, an index for the origin, the default is the source
            goal - int, an index for the goal, the default is sink
            data - bool, returns extended information if marked True
            DFS - bool, True will search depth-first and False will search 
            breadth-first. the default is BFS

        output
            default - float of max flow
            with data - {max_flow: float of max flow, iteration: 
             a dictionary with all traverses taken}
    
        complexity 
            O(VE^2)
        '''

        goal = goal or self.sink
        origin = origin or self.source

        #extendend information 
        presented_data = {}

        # initiate vars
        traverse = [-1] * self.size
        max_flow = 0
        iter_count = 0
        path_flow = float('inf')

        # before augmented path were exhusted 
        while self._search(traverse, origin, goal, DFS):

            iter_count += 1
            presented_data[iter_count] = []

            # walk backwards, find the smallest viable flow 
            # for an augmented path
            current = goal
            while current != origin :

                presented_data[iter_count].append(current)

                # the predecessor for the current index 
                pred = traverse[current]

                # calculate the residual, the path flow is the smaller 
                # between the edge residual and the existinf flow
                residual_flow = (self.capacity[(pred, current)] - \
                    self.flow[(pred, current)])
                path_flow = min(path_flow, residual_flow)

                current = pred

            # add the current path flow to the total flow
            max_flow += path_flow

            # walk backwards, update the flow on the edges based on 
            # the path flow
            current = goal
            while current != origin :

                # the predecessor for the current index 
                pred = traverse[current]

                self.flow[(pred, current)] += path_flow
                self.flow[(current, pred)] -= path_flow

                current = pred

        if data:
            return {'max_flow' : max_flow, 'iteration': \
            [{i: [0] + presented_data[i][::-1]} \
            for i  in range(1, iter_count+1)]}
        return max_flow

    def EdmondKarp(self, origin=None, goal=None, data=False):
        '''
        objective : an external func, uses _max_flow_search with BFS 
            Finds the shortest augmenting path from s to t, using BFS.
            For each path, it finds the bounding capacity and sends as 
            much flow. This process is repeating until there are no more 
            augmented path in the residual graph

        parameters :
            self - object
            origin - int, an index for the origin, the default is the source
            goal - int, an index for the goal, the default is sink
            data - bool, returns extended information if marked True
            
        output
            default - float of max flow
            with data - {max_flow: float of max flow, iteration: a dictionary 
                with all traverses taken}
        
        complexity 
            O(VE^2)
            A single BFS takes O(E), the push of flow on this augmented path 
            (with at least one edge), hence O(E).
            This process can be repeated bounded by the number of vertexes, 
            hence O(V). The total complexity, therefore, O(VE^2)
            
        '''
        return self._max_flow_search_FF(origin=origin, goal=goal, \
            data=data, DFS=False)

    def _BFS_using_levels(self, origin=None, goal=None):
        '''
        objective : bfs using the vertex level

        parameters : 
            origin: int, the default is the source
            goal: int, the default is the sink
        
        output: bool: True if the goals level is higher than 0 

        '''

        # set path origin and goal indexes

        origin = origin or self.source
        goal = goal or self.sink

        self.level = [-1] * self.size 
        self.level[origin] = 0

        to_visit = [origin]

        while to_visit :

            current = to_visit.pop(0)

            for successor in range(self.size):
                if self.capacity[(current, successor)] - \
                self.flow[(current, successor)] \
                 > 0 and self.level[successor] < 0:
                    self.level[successor] = self.level[current] + 1
                    to_visit.append(successor)


        return self.level

    def _send_flow(self, origin=None, goal=None, max_flow=float('inf')):
        '''
        objective : find the max flow based on DFS

        parameters : 
            origin: int, the default is the source
            goal: int, the default is the sink
            max_flow: int, default is the inf
        
        output : 0 if no path, flow if path found

        '''
        
        goal = goal or self.sink
        origin = origin or self.source

        # sink reached 
        if origin == goal :
            return max_flow

        # traverse all neighbors 
        for successor in range(self.size):

            # if a viable edge, a positive residual and d(u)<d(v)
            if ((self.capacity[(origin, successor)] - \
                self.flow[(origin, successor)] > 0 )
                and (self.level[origin] + 1 == self.level[successor])):
                
                # finds the bounding flow
                current_flow = \
                min(max_flow, self.capacity[(origin, successor)] \
                    - self.flow[(origin, successor)])
                
                path_flow = self._send_flow(successor, goal, current_flow)
                
                # if the flow is viable, update the path.
                if path_flow > 0:

                    self.flow[(origin, successor)] += path_flow
                    self.flow[(successor, origin)] -= path_flow

                    return path_flow
        return 0

    def _max_flow_search_D(self, origin=None, goal=None):
        '''
        objective : iteration on the augmented paths 

        parameters : 
            origin: int, the default is the source
            goal: int, the default is the sink

        
        output: the graph's max flow

        '''
        # initialzing vars
        goal = goal or self.sink
        origin = origin or self.source
        total_flow = 0


        # while there is path on the residual graph
        while True :
            # finds the path using BFS and marks their reachability to the sink
            self.level = self._BFS_using_levels(origin, goal)

            # if reached sink
            if self.level[goal] < 0:
                return total_flow

            # using the above paths to compute flow
            while True:
                path_flow = self._send_flow(origin, goal)

                if path_flow == 0:
                    break

                # incrementing flow
                total_flow += path_flow

    def Dinic(self, origin=None, goal=None):
        '''
        objective: creates the next path using BFS, marks vertexes 
            based on their distance from s. choosing only edges (u,v) 
            such d(u)<d(v), resulting in a graph with fewer edges, 
            and creates a graph where all path have the same length, 
            found by a single DFS. 
            Repeating only on the part of the graph that reaches t 
            (using the found blocking flow)


        ** performance exceeds EK for dense graphs.

        parameters : 
            origin: int, the default is the source
            goal: int, the default is the sink
            max_flow: int, default is the inf
        
        output: max flow

        complexity:
            O(EV^2)
            Each iteration finds a longer set of paths than the previous one. 
            Each DFS running on these paths (removing the unreachable vertexes) 
            takes O(E). The maximum length of these paths is V, 
            going through all vertexes. Hence, finding a viable 
            path takes O(VE). This process can be repeated bounded by 
            the number of vertexes, hence O(V). Reaching total complexity 
            of O(EV^2)




        '''

        return self._max_flow_search_D(origin=origin, goal=goal)

    def _push(self, u, v):
        '''
        objective: pushes the excess to a viable next vertex

        parameters :
            self - object
            v - int, index of an overflowing vertex
            u - int, index of a potential neighbor to move excess to 
            
        output
            none

        '''
        # caculates the viable transferable flow, residual or the full excess
        delta_flow = min(self.excess[u], (self.capacity[(u, v)] - \
            self.flow[(u, v)]))

        # modify the flow and excess in both vertexes, and edges, 
        # based on the delta in flow
        self.flow[(u, v)] += delta_flow
        self.flow[(v, u)] -= delta_flow

        self.excess[u] -= delta_flow
        self.excess[v] += delta_flow

    def _relabel(self, u):
        '''
        objective : modifies a vertex distnce to enable push 
            (to create d(u) < d(v))

        parameters :
            self - object
            u - int, index of the vertex to change the distance
            
        output
            none

        '''
        # initialize 
        min_distance = float('inf')

        # finds the min distance possible and changes the given 
        # vertex distance by this number
        for v in range(self.size):
            if (self.capacity[(u, v)] - self.flow[(u, v)]) > 0 :
                min_distance = min(min_distance, self.distance[v])
                self.distance[u] = min_distance + 1

    def _discharge(self, u):
        '''
        objective: iterating on an active vertex with an excess until resolved

        parameters :
            self - object
            u - int, index of a vertex with an excess flow, 
                keep pushing flow until it disactivates (= excess flow = 0)
            
        output
            none

        '''

        while self.excess[u] > 0 :

            # iterating through all possibel vertexes
            if self.seen[u] < self.size :

                v = self.seen[u]

                # verifies the conditions to push: positive residual on the edge 
                # and lower distance on the vertex the flow will be pushed to
                # pushes if viable or moves to the next vertex
                if (self.capacity[(u, v)] - self.flow[(u, v)] > 0) \
                and (self.distance[u] > self.distance[v]):
            
                    self._push(u, v)

                else:
                    self.seen[u] += 1
             
             # if a push is not possible, relabel
            else:
                self._relabel(u)
                self.seen[u] = 0

    def PushRelable(self, origin=None, goal=None):

        '''
        objective: unlike the previous two, this algorithm optimizes 
        based on local decisions. Front to back, this algorithm iterates 
        on the list of vertexes in the graph repeatedly selecting an 
        overflowing vertex and discharge it: using push and relabel until 
        the vertex deactivates.

        ** performance exceeds Dinic for dense graphs.


        parameters :
            self - object
            origin - int, an index for the origin, the default is the source
            goal - int, an index for the goal, the default is sink
            
        output
            graph max flow

        complexity 
            O(V^3)
            Each vertex can be relabeled 2V times; saturating pushes 
            (uses the full residual capacity) 
            And a nonsaturating pushes. There is a max of V saturating pushes 
            can be done on any edge. Hence max of 2V saturating pushes. 
            Pushes for an edge and its linked edge (numbered E) are O(VE). 
            There are O(V^2) relabling operation called on V vertexes - O(V^3). 
            Adding these O(V^3+VE) = O(V^3) 
        '''
        # initiates vars
        n = self.size

        goal = goal or self.sink
        origin = origin or self.source

        # inter_nodes, not sink or source, are viable for push
        inter_nodes = [i for i in range(n) if i != origin and i != goal]

        # pushes max capacity from source
        self.distance[origin] = n
        self.excess[origin] = float('inf')

        # resolves excess on source neighbors
        for v in range(n):
            self._push(origin, v)


        potential_vertex = 0

        while potential_vertex < len(inter_nodes):
            u = inter_nodes[potential_vertex]

            # resolve ecxess flow from u.
            pred_distance = self.distance[u]
            self._discharge(u)

            # checks whether te vertex was relabled
            if self.distance[u] > pred_distance :

                # move to front selection rule
                # inserts the vertex to the front and set the search back to 0
                inter_nodes.insert(0, inter_nodes.pop(potential_vertex))
                potential_vertex = 0

            else:
                potential_vertex += 1

        return sum (self.flow[origin])

