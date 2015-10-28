using UnityEngine;
using System.Collections;

public class MotionPlanner : MonoBehaviour {

	public GameObject Goal;
	public GameObject Agent;
	public Collider[] Obstacles;
	public Vector3 minimum;
	public Vector3 maximum;

	void Start () {
		
	}

	public class Node {
		public Vector3 position;
		public Node[] neighbors;
		private int lastNeighbor;

		public Node(){}
		public Node(Vector3 initPos){
			position = initPos;
		}

		//Creates a node using boundary parameters to generate a random node for use with PRM
		public Node(Vector3 minBound, Vector3 maxBound){
		// the Boundary is represented as a cube designated by the furthest vertices
		// minBound contains the minimum possible coordinates for the coordinate space
		// maxBound contains the maximum possible coordinates for the coordinate space
			position = new Vector3(Random.Range(minBound.x, maxBound.x), Random.Range(minBound.y, maxBound.y), Random.Range(minBound.z, maxBound.z));
		}
		
		public bool addNeighbor(Node neighborNode, Collider[] obstacles){
			Ray nodeToNeighbor = new Ray(position, (neighborNode.position - position));
			RaycastHit hitInfo;
			float distance = (neighborNode.position - position).magnitude;
			for(int i = 0; i < obstacles.Length; i++){
				//don't add as closest neighbor if the neighbor node draws a line through a collider
				if( obstacles[i].Raycast (nodeToNeighbor, out hitInfo, distance) ){
					return false;
				}
			}
			neighbors[lastNeighbor++] = neighborNode;
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
		public Vector3 minimum;
		public Vector3 maximum;
		
		public Graph(){}
		//Create a graph of N nodes with each node having k nearest neighbors
		public Graph(int n, int k){
			for(int i = 0; i < n; i++){
				contents[i] = new Node(minimum, maximum);
			}
			// in our contents of the graph, we find the k-nearest neighbors for each node
			// we first go through each node
			for(int i = 0; i < n; i++){
			// for each node we go through each other node and add if it is one of our nearest neighbors
			}
		}
		
	
	}

}
