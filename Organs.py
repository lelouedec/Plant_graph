import open3d as o3d


## mother class node/organ
class Organ():
    """The base class for organs/Nodes of the plan graph

    Attributes:
        type   :   string naming the node type
        pts    :   numpy array corresponding to the pointcloud points of the organ
        origin :   numpy array corresponding to the global position or the organ (x,y,z)
        prev   :   parent edge
        next   :   children edges
    """
    def __init__(self,pts=None,type="",origin=None):
        self.type   = type
        self.pts    = pts
        self.origin = origin
        self.prev   = []
        self.next   = []

    def o3d_mesh(self):
        pass


#one type of node 
class Leaf(Organ):
    def __init__(self,area,pts,origin):
        Organ.__init__(self,pts, "Leaf",origin) 
        self.area  = area
        self.colour  = [0.0,1.0,0.0]

    def __str__(self):
        # A print function, so each object can be inspected by its main attributes, neatly formatted as a string
        return f"""Organ type: Leaf, Area: {self.area}"""
    
    def o3d_mesh(self):
        node = o3d.geometry.TriangleMesh.create_box(width=0.1, height=0.1, depth=0.1)
        node.paint_uniform_color(self.colour)
        node.translate(self.origin)
        return node


#one type of node 
class Crown(Organ):
    def __init__(self,pts,origin):
        Organ.__init__(self,pts, "Crown",origin) 

    def __str__(self):
        # A print function, so each object can be inspected by its main attributes, neatly formatted as a string
        return f"""Organ type: {self.type}, Origin: {self.origin}"""

    def o3d_mesh(self):
        node = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
        node.paint_uniform_color(self.colour)
        node.translate(self.origin)
        return node


#one type of node 
class Strawberry(Organ):
    def __init__(self,volume,pts,origin):
        Organ.__init__(self, pts,"Strawberry",origin) 
        self.volume  = volume
        self.colour  = [1.0,0.0,0.0]

    def __str__(self):
        # A print function, so each object can be inspected by its main attributes, neatly formatted as a string
        return f"""Organ type: {self.type}, Volume: {self.volume}"""

    def o3d_mesh(self):
        node = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
        node.paint_uniform_color(self.colour)
        node.translate(self.origin)
        return node


#one type of node 
class Junction(Organ):
    def __init__(self,pts,origin):
        Organ.__init__(self, pts,"Junction",origin) 

    def __str__(self):
        # A print function, so each object can be inspected by its main attributes, neatly formatted as a string
        return f"""Organ type: {self.type}"""
    





## mother class edges 
class Edge():
    """The base class for edges of the plan graph

    Attributes:
        nodes_idx   :  nodes connected to the edge, can be their index or None (the graph will look for the closest node to the end of the edge when None)
        type        :   type of edge, can be petiole, runner, branch etc....
        pts         :   numpy array corresponding to the points from the point clouds belonging to the edge
    """
    def __init__(self,pts,type="",nodes_idx=[0,1]):
        self.nodes_idx = nodes_idx
        self.type  = type
        self.pts = pts

    def o3d_mesh(self):
        pass


#one type of edges
class Petioles(Edge):
    def __init__(self,pts,vertices,nodes_idx):
        Edge.__init__(self, pts,"Petiole",nodes_idx) 
        self.vertices = vertices

    def o3d_mesh(self):      
        edges = [[j,j+1] for j in range(0,len(self.vertices)-1)]
        line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(self.vertices), lines=o3d.utility.Vector2iVector(edges))
        return line_set

