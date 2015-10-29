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


    public class Solve
    {
        Node[] nodes;
        Node start;
        Node goal;

        public Solve(Node[] nodesIn, Node startIn, Node goalIn){
            nodes = nodesIn;
            start = startIn;
            goal = goalIn;
        }

        /*public ArrayList AStarTemp(){
            System.Collections.Generic.SortedList<float, ArrayList> paths = new System.Collections.Generic.SortedList<float, ArrayList>();
            float curCost;
            ArrayList curList = new ArrayList();
            Node curNode;
            Node curNeighbor;
            float g;
            float h;
            System.Collections.Generic.SortedList<float, Node> f;
            // Populate branches of the graph with the neighbors of "Start"
            for (int i = 0; i < start.neighbors.Count; i ++){
                curList = new ArrayList();
                curList.Add(start.neighbors[i]);
                paths.Add(distance(start, start.neighbors[i]), curList);
            }
            while (paths.Count > 0){
                curCost = paths.Keys[0];
                curList = paths[curCost];
                paths.Remove(paths.Keys[0]);
                curNode = (Node)curList[curList.Count - 1];
                if (curNode == goal){
                    return curList;
                }
                f = new System.Collections.Generic.SortedList<float, Node>();
                if (curNode.neighbors.Count > 0){
                    for (int i = 0; i < curNode.neighbors.Count; i++)
                    {
                        curNeighbor = curNode.neighbors[i];
                        h = curCost + distance(curNode, curNeighbor);
                        g = distance(curNeighbor, goal);                                       
                        f.Add(g + h, curNeighbor);
                    }
                    curList.Add(paths.Values[0]);
                    paths.Add(f.Keys[0], curList);
                }
            }
            return new ArrayList();
        }*/


        public ArrayList Dijkstra()
        {
            System.Collections.Generic.SortedList<float, ArrayList> paths = new System.Collections.Generic.SortedList<float, ArrayList>();
            float curCost;
            float newCost;
            ArrayList curList = new ArrayList();
            ArrayList newList;
            Node curNode;
            Node curNeighbor;
            curList = new ArrayList();
            curList.Add(start);
            paths.Add(0, curList);

            while (paths.Count > 0)
            {
                curCost = paths.Keys[0];
                curList = paths[curCost];
                paths.Remove(paths.Keys[0]);
                curNode = (Node)curList[curList.Count - 1];

                if (curNode.neighbors.Count > 0)
                {
                    for (int i = 0; i < curNode.neighbors.Count; i++)
                    {
                        curNeighbor = curNode.neighbors[i];
                        newCost = curCost + distance(curNode, curNeighbor);
                        newList = new ArrayList();
                        newList.AddRange(curList);
                        newList.Add(curNeighbor);
                        paths.Add(newCost, newList);
                        if (curNeighbor.position == goal.position)
                        {
                            return newList;
                        }
                    }                
                }
            }
            return new ArrayList();
        }



        public ArrayList AStar(){
            ArrayList open = new ArrayList();
            ArrayList closed = new ArrayList();


            return new ArrayList();
        }



        private class ANode {




        }

        private float distance(Node n1, Node n2){
            return (n2.position - n1.position).magnitude;
        }

    }

}
