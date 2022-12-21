

from Organs import *
import open3d as o3d
import numpy as np


class Plant():
    def __init__(self,nodes,edges):## the plant is just a collection of stems (petioles,branches,runners...) and nodes (fruit, leaves and crowns)
        self.edges = edges
        self.nodes = nodes

        
        ## set for all nodes the previous and nexts edges to navigate the tree
        for i,e in enumerate(edges):

            ## we first check that the two sides of the edge are not None, if they are we search for the closest node.
            if(e.nodes_idx[0]==None):
                e.nodes_idx[0] = self.find_closest_node(e.vertices[0])

            if(e.nodes_idx[1]==None):
                e.nodes_idx[1] = self.find_closest_node(e.vertices[-1])

            self.nodes[e.nodes_idx[0]].next.append(i)
            self.nodes[e.nodes_idx[1]].prev.append(i)


    def  find_closest_node(self,coordinate):
        min_dist = 1000.0
        id = None
        for i,n in enumerate(nodes):
            distance = np.sqrt( (coordinate[0]-n.origin[0]) * (coordinate[0]-n.origin[0]) + 
                                (coordinate[1]-n.origin[1]) * (coordinate[1]-n.origin[1]) + 
                                (coordinate[2]-n.origin[2]) * (coordinate[2]-n.origin[2])  
                             )
            print(distance,coordinate,n.origin)
            if(distance<=min_dist):
                min_dist=distance
                id = i
        return id

    def __str__(self):
        string_val = f""""""
        string_val =  string_val + self.nodes[0].__str__()+"\n"
        for i in range(0,len(self.edges)):
            string_val =  string_val + self.edges[i].type
            if(self.edges[i].nodes_idx[0]!=0):
                string_val =  string_val + self.nodes[self.edges[i].nodes_idx[0]].__str__()
            stem = ""
            for j in range(0,len(self.edges[i].vertices)):
                stem = stem +"--"+str(self.edges[i].vertices[j])
            string_val =  string_val + stem +"->"+ self.nodes[self.edges[i].nodes_idx[1]].__str__()+"\n"
        return string_val

    def Visualize(self):
        geometries = []        
        for i in range(0,len(self.edges)):
            geometries.append(self.edges[i].o3d_mesh())
            node = None
            if(self.nodes[self.edges[i].nodes_idx[-1]].type!="Junction"):
                node = self.nodes[self.edges[i].nodes_idx[-1]].o3d_mesh()
                geometries.append(node)
        o3d.visualization.draw_geometries(geometries)
    
    def print_branch(self,branch):
        val = ""
        val+= "P"
        val+=self.nodes[branch.nodes_idx[1]].type[0]
        if(self.nodes[branch.nodes_idx[1]].type=="Junction"):             
            for i in range(0,len(self.nodes[branch.nodes_idx[1]].next)):
                val+= "{"
                val+=self.print_branch(self.edges[self.nodes[branch.nodes_idx[1]].next[i]]) 
                val+="}"
        return val


    def L_representation(self):
        string = ""
        string+= "C"
        for i in range(0,len(self.nodes[0].next)):
            string+= "{" 
            string+= self.print_branch(self.edges[i])
            string+="}"

        print(string)



if __name__ == "__main__":
    
    dummy_points_cloud = np.array([[0.0,0.0,0.0],[0.0,2.0,0.0],[0.0,1.0,0.0]])
    ## nodes can be crown, leaves, fruits. each with different attributes
    nodes = [Crown(dummy_points_cloud,np.array([0,0,0])),
             Leaf(dummy_points_cloud,10,np.array([0.5,2.0,0.0])),
             Leaf(dummy_points_cloud,15,np.array([0.0,1.5,2.0])),
             Strawberry(dummy_points_cloud,50,np.array([0.0,1.5,-2.0])),
             Junction(dummy_points_cloud,np.array([2.0,1.5,0.0])),
             Leaf(dummy_points_cloud,3,np.array([2.5,2.0,0.0])),
             Strawberry(dummy_points_cloud,25,np.array([2.5,1.5,0.0]))]

    ##edges can be petioles, runners, branches etc.. each class as different attributes
    # We pass to the Petiole class the pointcloud of the edge, the coordinates of all the keypoints from begining to end, and the nodes it is connected to (can be None)
    edges = [Petioles(dummy_points_cloud,[[0.0,0.0,0.0],[0.0,1.0,0.0],[0.0,1.5,0.0],[0.5,2.0,0.0]],[0,1]),
             Petioles(dummy_points_cloud,[[0.0,0.0,0.0],[0.0,1.0,1.0],[0.0,1.5,2.0]],[0,2]),
             Petioles(dummy_points_cloud,[[0.0,0.0,0.0],[0.0,1.0,-1.0],[0.0,1.5,-2.0]],[0,3]),
             Petioles(dummy_points_cloud,[[0.0,0.0,0.0],[1.5,1.0,0.0],[2.0,1.5,0.0]],[0,4]),
             Petioles(dummy_points_cloud,[[2.0,1.5,0.0],[2.5,2.0,0.0]],[4,5]),
             Petioles(dummy_points_cloud,[[2.0,1.5,0.0],[2.5,1.5,0.0]],[4,None])
            ]

    plant = Plant(nodes,edges)

    print(plant)
    plant.L_representation()
    plant.Visualize()


