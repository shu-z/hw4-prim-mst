import numpy as np
import heapq
from typing import Union

class Graph:

    def __init__(self, adjacency_mat: Union[np.ndarray, str]):
        """
    
        Unlike the BFS assignment, this Graph class takes an adjacency matrix as input. `adjacency_mat` 
        can either be a 2D numpy array of floats or a path to a CSV file containing a 2D numpy array of floats.

        In this project, we will assume `adjacency_mat` corresponds to the adjacency matrix of an undirected graph.
    
        """
        if type(adjacency_mat) == str:
            self.adj_mat = self._load_adjacency_matrix_from_csv(adjacency_mat)
        elif type(adjacency_mat) == np.ndarray:
            self.adj_mat = adjacency_mat
        else: 
            raise TypeError('Input must be a valid path or an adjacency matrix')
        self.mst = None


    def _load_adjacency_matrix_from_csv(self, path: str) -> np.ndarray:
        with open(path) as f:
            return np.loadtxt(f, delimiter=',')


      def _get_edges(self, idx):
        """ 
        Takes in index of adjacency matrix and returns list of outgoing edges
        Edges are tuples in format (edge weight, destination node)
        
        """
        
        adj=self.adj_mat
        
        idx_edges=np.where(adj[idx]!=0)[0]  
        outgoing_edges=list(zip(adj[idx, idx_edges], idx_edges))

        return(outgoing_edges)



    def construct_mst(self):
        """
        TODO: Given `self.adj_mat`, the adjacency matrix of a connected undirected graph, implement Prim's 
        algorithm to construct an adjacency matrix encoding the minimum spanning tree of `self.adj_mat`. 
            
        `self.adj_mat` is a 2D numpy array of floats. Note that because we assume our input graph is
        undirected, `self.adj_mat` is symmetric. Row i and column j represents the edge weight between
        vertex i and vertex j. An edge weight of zero indicates that no edge exists. 
        
        This function does not return anything. Instead, store the adjacency matrix representation
        of the minimum spanning tree of `self.adj_mat` in `self.mst`. We highly encourage the
        use of priority queues in your implementation. Refer to the heapq module, particularly the 
        `heapify`, `heappop`, and `heappush` functions.
        """
        adj=self.adj_mat

        #init mst adjacency matrix
        mst=np.zeros(adj.shape)
        
        #pick random node to start 
        init_node=random.randint(0, adj.shape[0]-1)

        #add first node to visisted      
        visited=[init_node]
        
        #store all outgoing edges from visited in a priority queue
        #heapify will work with list of tuples where tuple is (distance, node)        
        edge_queue=self._get_edges(init_node)   
        hq.heapify(edge_queue)
      
        prev_node=init_node
        #while not all vertices in visited 
        while len(visited)<adj.shape[0]:
            
            #pop lowest weight edge from priority queue 
            edge, node=hq.heappop(edge_queue)

            
            #check if destination node of edge in visited
            if node not in visited:

                #add edge to mst
                mst[visited[-1], node]=edge
            
                #add destination node to visited
                visited.append(node)
                
                #get outgoing edges from node, and add to priority queue                 
                for i in self._get_edges(node): 
                    hq.heappush(edge_queue, i) 
 
        
        
        #make sure mst is symmetric!
        mst=np.triu(mst) + np.tril(mst.T, 1)
        
        self.mst = mst
