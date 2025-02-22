import pytest
import numpy as np
from mst import Graph
from sklearn.metrics import pairwise_distances


def check_mst(adj_mat: np.ndarray, 
              mst: np.ndarray, 
              expected_weight: int, 
              allowed_error: float = 0.0001):
    """
    
    Helper function to check the correctness of the adjacency matrix encoding an MST.
    Note that because the MST of a graph is not guaranteed to be unique, we cannot 
    simply check for equality against a known MST of a graph. 

    Arguments:
        adj_mat: adjacency matrix of full graph
        mst: adjacency matrix of proposed minimum spanning tree
        expected_weight: weight of the minimum spanning tree of the full graph
        allowed_error: allowed difference between proposed MST weight and `expected_weight`

    TODO: Add additional assertions to ensure the correctness of your MST implementation. For
    example, how many edges should a minimum spanning tree have? Are minimum spanning trees
    always connected? What else can you think of?

    """

    def approx_equal(a, b):
        return abs(a - b) < allowed_error

    total = 0
    for i in range(mst.shape[0]):
        for j in range(i+1):
            total += mst[i, j]
    assert approx_equal(total, expected_weight), 'Proposed MST has incorrect expected weight'



    def bfs(graph, start):
        """
        BFS adapted from HW2 to check if graph is fully connected
        """
       
        queue=[start]
        visited=[start]

        while queue:
            #pop first element of queue and get neighbors
            v=queue.pop(0)
            N=np.where(graph[v]!=0)[0] 

            #check each neighbor of current node
            for w in N:
                if w not in visited:
                    #add new frontier nodes to queue and visited
                    queue.append(w)
                    visited.append(w)
        #return list of nodes with order of BFS traversal
        return(visited)
    

    #check fully connected mst
    bfs_visited=bfs(mst, 0)
    bfs_visited.sort()
    assert(bfs_visited==list(range(0, adj_mat.shape[0]))), 'Proposed MST is not fully connected'
    
    #check edges mst
    assert ((adj_mat.shape[0]-1) == np.count_nonzero(np.triu(mst))), 'Proposed MST does not have correct number of edges'

    #check symmetric mst
    assert(np.allclose(mst, mst.T)), 'Proposed MST is not symmetric'
           

  

def test_mst_small():
    """
    Unit test for the construction of a minimum spanning tree on a small graph.  
    """
    file_path = './data/small.csv'
    g = Graph(file_path)
    g.construct_mst()
    check_mst(g.adj_mat, g.mst, 8)


def test_mst_single_cell_data():
    """
    Unit test for the construction of a minimum spanning tree using single cell
    data, taken from the Slingshot R package.

    https://bioconductor.org/packages/release/bioc/html/slingshot.html
    """
    file_path = './data/slingshot_example.txt'
    coords = np.loadtxt(file_path) # load coordinates of single cells in low-dimensional subspace
    dist_mat = pairwise_distances(coords) # compute pairwise distances to form graph
    g = Graph(dist_mat)
    g.construct_mst()
    check_mst(g.adj_mat, g.mst, 57.263561605571695)


def test_mst_edgecase():
    """
    TODO: Write at least one unit test for MST construction.
    Unit test for edge cases 
    """

    #test 1 node graph
    with pytest.raises(Exception, match='Graph has 1 or fewer nodes!'):
        g_single_node = Graph(np.array([0]))
        g_single_node.construct_mst()

    #test nonsymmetric graph
    with pytest.raises(Exception, match='Adjacency matrix is not symmetric!'):
        asymmetric_array=np.array([(0,4,3),(1,0,5),(2,4,0)])
        g_asymm = Graph(asymmetric_array)
        g_asymm.construct_mst()
    #  


    
