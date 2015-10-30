using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine.Events;
using UnityEngine.UI;

public class MotionPlanner : MonoBehaviour {

	public GameObject Goal;
	public GameObject Agent;
    public GameObject StartObj;
	public Collider[] Obstacles;
	public static Vector3 minimum;
	public static Vector3 maximum;
    public static List<Bounds> ConfigSpace;
    public GameObject startX;
    public GameObject startY;
    public GameObject goalX;
    public GameObject goalY;

	void Start () {
        print("Starting");
        minimum = new Vector3(-9, 1, -9);
        maximum = new Vector3(9, 1, 9);
        ConfigSpace = new List<Bounds>();
        buildConfigSpace();         
    }

    void Update()
    {
        // Run Dijkstra's algorithm
        if (Input.GetKeyDown("d"))
        {
            while (true)
            {
                Graph g = new Graph(20, 5, Obstacles);
                if (g.solnList.Count > 0)
                {
                    for (int i = 0; i < g.solnList.Count; i++)
                    {
                        //print(g.solnList[i].position);
                    }
                    break;
                }
                else
                {
                }
            }
        }
    }

    public void updatePos()
    {
        Agent.transform.position = new Vector3(startX.GetComponent<Dropdown>().value - 9, 1, startY.GetComponent<Dropdown>().value - 9);
        Goal.transform.position = new Vector3(goalX.GetComponent<Dropdown>().value - 9, 1, goalY.GetComponent<Dropdown>().value - 9);
        StartObj.transform.position = Agent.transform.position;
    }

    public void buildConfigSpace()
    {
        print("Agent Bounts: " + Agent.GetComponent<Collider>().bounds);
        print("size of Obstacles: " + Obstacles.Length);
        foreach (Collider c in Obstacles)
        {
            //Bounds b = new Bounds(c.bounds.center, (c.bounds.extents + Agent.GetComponent<Collider>().bounds.extents));
            print("Bounds: " + c.bounds);
            //print("New Bounds: " + b);
            c.bounds.Expand(Agent.GetComponent<Collider>().bounds.size);
            //ConfigSpace.Add(new Bounds(c.bounds.center, (c.bounds.size + Agent.GetComponent<Collider>().bounds.size)));
        }
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
            if (neighborCount < numNeighbors)
            {
                neighbors.Add(distance, neighborNode);
                neighborCount += 1;
            }
            else if (distance < neighbors.Keys[0])
            {
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

        private Node start;
        private Node goal;

        public List<Node> solnList;

        public Graph(){}
		//Create a graph of N nodes with each node having k nearest neighbors
		public Graph(int n, int k, Collider[] Obstacles){

            start = new Node(GameObject.Find("Agent").transform.position, k);
            goal = new Node(GameObject.Find("Goal").transform.position, k);

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
                contents[i].addNeighbor(goal, Obstacles);
			}
            // Find neighbors for starting node
            for (int i = 0; i < n; i++)
            {
                start.addNeighbor(contents[i], Obstacles);
            }
            start.addNeighbor(goal, Obstacles);

            Solve soln = new Solve(contents, start, goal);
            solnList = soln.Dijkstra();               
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


        public List<Node> Dijkstra(){
            List<Path> paths = new List<Path>();
            float curCost;
            float newCost;
            List<Node> curList = new List<Node>();
            List<Node> newList;
            Node curNode;
            Node curNeighbor;
            curList = new List<Node>();
            curList.Add(start);
            paths.Add(new Path(0, curList));

            int maxPaths = 0;

            while (paths.Count > 0 && paths.Count < 200)
            {
                if (paths.Count > maxPaths)
                {
                    maxPaths = paths.Count;
                }
                paths.Sort((x, y) => x.distance.CompareTo(y.distance));
                curCost = paths[0].distance;
                curList = paths[0].list;
                paths.RemoveAt(0);
                curNode = curList[curList.Count - 1];

                if (curNode.neighbors.Count > 0)
                {
                    for (int i = 0; i < curNode.neighbors.Count; i++)
                    {
                        curNeighbor = curNode.neighbors.Values[i];
                        if (!curList.Any(f => f.position == curNeighbor.position)){
                            newCost = curCost + distance(curNode, curNeighbor);
                            newList = new List<Node>();
                            newList.AddRange(curList);
                            newList.Add(curNeighbor);
                            paths.Add(new Path(newCost, newList));
                            if (curNeighbor.position == goal.position)
                            {
                                print("max size: " + maxPaths);
                                return newList;
                            }
                        }                
                    }     
                }
            }
            return new List<Node>();
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


        private class Path{
            public float distance;
            public List<Node> list;

            public Path(float distanceIn, List<Node> listIn){
                distance = distanceIn;
                list = listIn;
            }       
        }


    }

}
