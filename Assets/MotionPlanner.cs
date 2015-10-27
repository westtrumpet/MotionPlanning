using UnityEngine;
using System.Collections;

public class MotionPlanner : MonoBehaviour {

	public GameObject[] Walls;
	public GameObject[] ObstaclesObject;
	public GameObject Goal;
	public GameObject Agent;

	void Start () {
		
	}

	public class Node {
		public Vector3 position;
		public Node[] neighbors;
		private int lastNeighbor;

		public Node(){}
		public Node(Vector3 initPos)	{
			position = initPos;
		}
		public Node(Vector3 minBound, Vector3 maxBound){
		// the Boundary is represented as a cube designated by the furthest vertices
		// minBound contains the minimum possible coordinates for the coordinate space
		// maxBound contains the maximum possible coordinates for the coordinate space
			position = new Vector3(Random.Range(minBound.x, maxBound.x), Random.Range(minBound.y, maxBound.y), Random.Range(minBound.z, maxBound.z));
		}
	/*	
		public void addNeighbor(Node neighborNode){
			Collider[] Obstacles;
			int n;
			if(!Obstacles[n].bounds.Contains(neighborNode.position)) {
				neighbors[lastNeighbor++] = neighborNode;
			}
		}
	*/	
	}

}
