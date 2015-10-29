using UnityEngine;
using System.Collections;

public class MotionPlanner : MonoBehaviour {

	public GameObject Goal;
	public GameObject Agent;
	public Collider[] Obstacles;
	public static Vector3 minimum;
	public static Vector3 maximum;

	void Start () {
        print("Starting");
        minimum = new Vector3(-9, 1, -9);
        maximum = new Vector3(9, 1, 9);
        Graph g = new Graph(10, 2, Obstacles);
	}

	public class Node {
		public Vector3 position;
        public System.Collections.Generic.SortedList<float, Node> neighbors = new System.Collections.Generic.SortedList<float, Node>();
		private int neighborCount = 0;
        private int numNeighbors;
        GameObject NodeLight;

        public Node(){}
		public Node(Vector3 initPos, int k){
			position = initPos;
            //AddNodeLight();
            numNeighbors = k;
        }

		//Creates a node using boundary parameters to generate a random node for use with PRM
		public Node(Vector3 minBound, Vector3 maxBound, int k){
		// the Boundary is represented as a cube designated by the furthest vertices
		// minBound contains the minimum possible coordinates for the coordinate space
		// maxBound contains the maximum possible coordinates for the coordinate space
			position = new Vector3(Random.Range(minBound.x, maxBound.x), 1, Random.Range(minBound.z, maxBound.z));
            //AddNodeLight();
            numNeighbors = k;
		}

        private void AddNodeLight(){
            NodeLight = new GameObject("The Light");
            Light LightComp = NodeLight.AddComponent<Light>();
            LightComp.color = Color.magenta;
            NodeLight.transform.position = position;
            LightComp.range = 10F;
            print(NodeLight);
        }
		
		public bool addNeighbor(Node neighborNode, Collider[] obstacles){
			Ray nodeToNeighbor = new Ray(position, (neighborNode.position - position));
			RaycastHit hitInfo;
			float distance = (neighborNode.position - position).magnitude;
			for(int i = 0; i < obstacles.Length; i++){
				//don't add as closest neighbor if the neighbor node draws a line through a collider
				if( obstacles[i].Raycast (nodeToNeighbor, out hitInfo, distance) ){
                    print("collision");
					return false;
				}
			}
            if (neighborCount < numNeighbors)
            {
                print("initial neighbor");
                neighbors.Add(distance, neighborNode);
                neighborCount += 1;
            }
            else if (distance < neighbors.Keys[0])
            {
                print("replaced neighbor");
                neighbors.Remove(neighbors.Keys[0]);
                neighbors.Add(distance, neighborNode);
            }

			return true;
		}

		public bool willCollide(Vector3 checkPos, Collider[] obstacles) {
			Ray posToNode = new Ray(checkPos, (position - checkPos));
			RaycastHit hitInfo;
			float distance = (position - checkPos).magnitude;
			for(int i = 0; i < obstacles.Length; i++){
				if( obstacles[i].Raycast (posToNode, out hitInfo, distance) ){
					return true;
				}
			}
			return false;	
		}	
	}

	public class Graph {
		public Node[] contents;
		public int lastNode;
		
		public Graph(){}
		//Create a graph of N nodes with each node having k nearest neighbors
		public Graph(int n, int k, Collider[] Obstacles){
            contents = new Node[n];
			for(int i = 0; i < n; i++){
				contents[i] = new Node(minimum, maximum, k);
			}
			// in our contents of the graph, we find the k-nearest neighbors for each node
			// we first go through each node
			for(int i = 0; i < n; i++){
                // for each node we go through each other node and add if it is one of our nearest neighbors
                for (int j = 0; j < n; j ++){
                    if (i != j)
                    {
                        contents[i].addNeighbor(contents[j], Obstacles);
                    }
                }
                print(contents[i].neighbors.Count);
			}
		}
		
	
	}

}
