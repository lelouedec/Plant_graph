import open3d as o3d
import numpy as np 
import Sph_hmade
import trimesh
from scipy.special import sph_harm


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
    def __init__(self,pts,area,origin):
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
        self.colour = [1.0,0.0,1.0]

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
    """The base class for strawberries

    Attributes:
        volume    :   volume of the fruit
        colour    :   colour for open3d representation
        L         :   number of spherical harmonics degrees
        sph_coefs :   spherical harmonics coefficients
    """
    def __init__(self,pts,origin):
        Organ.__init__(self, pts,"Strawberry",origin) 
        self.colour  = [1.0,0.0,0.0]
        self.L = 10 
        self.sph_coefs = self.Compute_harmonics(pts)
        self.mesh = self.Mesh_from_harmonics()
        print("computing fruit volume with open3d....")
        self.volume = self.mesh.get_volume()

    def __str__(self):
        # A print function, so each object can be inspected by its main attributes, neatly formatted as a string
        return f"""Organ type: {self.type}, Volume: {self.volume}"""

    def o3d_mesh(self):
        node = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
        node.paint_uniform_color(self.colour)
        node.translate(self.origin)
        return node


    def compute_reconstruction(self,rho,points,theta,phi,idx):
        rho     =    rho[idx]
        points  = points[idx]
        theta   =  theta[idx]
        phi     =    phi[idx]
        coefs = Sph_hmade.forwardSHT(self.L,np.array([theta,phi,rho]),True)
        return coefs


    def Compute_harmonics(self,pts):

        pts = pts - pts.mean(0)
        xy = pts[:,0]**2 + pts[:,1]**2
        rho = np.sqrt(xy + pts[:,2]**2)
        phi = np.arctan2(np.sqrt(xy), pts[:,2]) # for elevation angle defined from Z-axis down
        theta = np.arctan2(pts[:,1], pts[:,0]) + np.pi


        idxs = [np.where(pts[:,1]<=0.02)[0],np.where(pts[:,1]>=-0.02)[0]]
        coefs = []
        for i in idxs:
            coef=self.compute_reconstruction(rho,pts,theta,phi,i)
            coefs.append(coef)

        coefs = np.array(coefs)

        return coefs


    def create_sph(self,harms,theta,phi):
        vals = []
        for h in harms:
            fcolors = sph_harm(h[0],h[1],theta , phi)
            fcolors = fcolors.real
            vals.append(fcolors)
        return np.array(vals)


    def create_harms_list(self):
        harms = []
        for l in range(0,self.L+1):
            if(l==0):
                harms.append([0,0,1.0])
            else:
                for m in range(-l,l+1):
                    harms.append([m,l,1.0])
        return harms

    def Mesh_from_harmonics(self):
        sphere = trimesh.load("./sphere.stl",  process=True, maintain_order=True)
        sphere_vertices = np.asarray(sphere.vertices)
        sphere_triangles = np.asarray(sphere.faces)

        xy = sphere_vertices[:,0]**2 + sphere_vertices[:,1]**2
        rho = np.sqrt(xy + sphere_vertices[:,2]**2)
        phi = np.arctan2(np.sqrt(xy), sphere_vertices[:,2]) # for elevation angle defined from Z-axis down
        theta = np.arctan2(sphere_vertices[:,1], sphere_vertices[:,0]) + np.pi

        x = np.sin(phi) * np.cos(theta) 
        y = np.sin(phi) * np.sin(theta)
        z = np.cos(phi)
        x = np.expand_dims(x,1)
        y = np.expand_dims(y,1)
        z = np.expand_dims(z,1)
        points = np.concatenate([x,y,z],1) 
        
        harms = self.create_harms_list()
        vals =  self.create_sph(harms,theta,phi)
        idxs = [np.where(points[:,1]<=0.02)[0],np.where(points[:,1]>=-0.02)[0]]
        # print(theta[idxs[0]].max(),theta[idxs[1]].min())

        verticesa = points[idxs[0]].copy()
        verticesb = points[idxs[1]].copy()
        coef1 = np.array(vals)[:,idxs[0]] * self.sph_coefs[0,:][:,None]
        coef1 = coef1.sum(0)
        verticesa = verticesa *coef1[:,None] 

        coef2 = np.array(vals)[:,idxs[1]] * self.sph_coefs[1,:][:,None]
        coef2 = coef2.sum(0)
        verticesb = verticesb *coef2[:,None]

        overlaps,id1,id2 = np.intersect1d(idxs[0], idxs[1],return_indices=True)
        verticesb[id2]   = (verticesa[id1] + verticesb[id2])/2.0
        vertices = points.copy()
        vertices[idxs[0]] = verticesa
        vertices[idxs[1]] = verticesb

        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(vertices)
        mesh.triangles = o3d.utility.Vector3iVector(sphere_triangles)
        # o3d.visualization.draw_geometries([mesh])
        return mesh


        


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

