using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Basic physics manager capable of simulating a given Simulable
/// implementation using diverse integration methods: explicit,
/// implicit, Verlet and semi-implicit.
/// </summary>
public class MassSpringCloth : MonoBehaviour 
{
	/// <summary>
	/// Default constructor. Zero all. 
	/// </summary>
	public MassSpringCloth()
	{
		paused = true;
		substeps = 5;
		timeStep = 0.01f;  // Default value 0.01f
		gravity = new Vector3 (0.0f, -9.81f, 0.0f);
		wind = new Vector3(3.4f, 0.0f, 0.0f);
		integrationMethod = Integration.Symplectic;
	}

	/// <summary>
	/// Integration method.
	/// </summary>
	public enum Integration
	{
		Explicit = 0,
		// Semi-Implicit Euler Integration Explicit in Velocity and Implicit in Position
		Symplectic = 1
	}

	#region InEditorVariables

	public bool paused;
	public int substeps;
	public float timeStep;
    public Vector3 gravity;
    public Vector3 wind;
	public Integration integrationMethod;
    public List<Node> nodes;
    public List<Spring> springs;

    #endregion

    #region OtherVariables
    
    // Variables for initializing (vertices - nodes) and (edges - springs)
    private Mesh _mesh;
    private Vector3[] _vertices;
    private int[] _triangles;

    private class Edge
    {
	    private int vertexA;
	    private int vertexB;
	    private int vertexOther;

	    public Edge(int vA, int vB)  // Delete when springs are fixed :)  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	    {
		    if (vA <= vB)  // The smallest vertex index is always in the first position
		    {
			    vertexA = vA;
			    vertexB = vB;
		    }
		    else
		    {
			    vertexA = vB;
			    vertexB = vA;
		    }
	    }
	    
	    public Edge(int vA, int vB, int vO)
	    {
		    if (vA <= vB)  // The smallest vertex index is always in the first position
		    {
			    vertexA = vA;
			    vertexB = vB;
		    }
		    else
		    {
			    vertexA = vB;
			    vertexB = vA;
		    }

		    vertexOther = vO;
	    }

	    public int GetVertexA()
	    {
		    return vertexA;
	    }
	    
	    public int GetVertexB()
	    {
		    return vertexB;
	    }

	    public int GetVertexOther()
	    {
		    return vertexOther;
	    }
	    
	    public override bool Equals(object obj)
	    {
		    // If the passed object is null
		    if (obj == null)
		    {
			    return false;
		    }
		    if (!(obj is Edge))
		    {
			    return false;
		    }
		    return (vertexA == ((Edge)obj).vertexA)
		           && (vertexB == ((Edge)obj).vertexB);
	    }
	    
	    public override int GetHashCode()
	    {
		    return vertexA.GetHashCode() ^ vertexB.GetHashCode();
	    }
    }
    
    #endregion

    #region MonoBehaviour

    public void Start()
    {
	    // Retrieve the mesh filter component
	    _mesh = GetComponent<MeshFilter>().mesh;
	    
	    // Assign each vertex of the mesh a node behaviour
	    Debug.Log("Assigning each vertex of the mesh a node behaviour...");
	    _vertices = _mesh.vertices;  // Retrieve mesh vertices
	    
	    int idxNode = 0;
	    foreach (Vector3 vertex in _vertices)
	    {
		    // First create the game object recipient and name it
		    GameObject nodeGameObject = new GameObject();
		    nodeGameObject.name = "Node " + idxNode;
		    nodeGameObject.AddComponent<Node>();  // Assign a node component into the new GameObject
		    
		    Node node = nodeGameObject.GetComponent<Node>();  // Get actual node component that is being processed
		    
		    node.GetComponent<Transform>().parent = this.transform;  // Assign this MassSpringCloth as parent transform of the nodes
		    
		    /////// ASK QUESTION: 2 options to solve this position assignment ///////////
		    //node.GetComponent<Transform>().position = transform.TransformPoint(vertex);  // Locate the nodes in the positions of the mesh vertices
		    node.pos = transform.TransformPoint(vertex);  // Locate the nodes in the positions of the mesh vertices (Global positions)
		    /////////////////////////////////////////////////////////////////////////////
		    
		    node.mass = 1.0f;  // Set node mass
		    // Quad fixing script (hardcoded)
		    //if (IdxNode == 2 || IdxNode == 3) node.isFixed = true;  // Set node 0 to fixed for debugging the simulation
		    // Full plane textile fixing script (hardcoded)
		    //if (IdxNode >= 55 && IdxNode <= 65) node.isFixed = true;  // Set middle nodes to fixed for bending simulation
		    //if (IdxNode >= 110 && IdxNode <= 120) node.isFixed = true;  // Set middle nodes to fixed for holding simulation

		    nodes.Add(node);  // Finally add node to the nodes list to iterate through all of them on fixed update loop
		    idxNode++;
	    }
	    
	    // ANNOTATION:
	    // An upgrade would be assigning Node behaviour to vertices of the mesh while iterating the triangles
	    // because triangle iterating is faster than vertex iterating, due to the complexity of the meshes of
	    // this assignment, this is not necessary but it still needs to be mentioned
	    
	    // Assign each edge of the mesh a spring behaviour
	    Debug.Log("Assigning each edge of the mesh a spring behaviour...");
	    _triangles = _mesh.triangles;
	    List<Edge> edges = new List<Edge>();

	    // Create all NodeA - NodeB springs and assign them an stiffness k
	    if (_triangles.Length % 3 == 0)  // Check if the number of indices is exact for a number of triangles
	    {
		    int topologyIdx = 1;
		    for (int i = 0; i < _triangles.Length / 3; ++i)  // Initialize edges vertex _triangles indices
		    {
			    // POSSIBLE OPTIMIZATION FOR MILLION VERTEX MESHES: CREATE NODES IN THIS LOOP!!! (Triangle iteration)
			    // Edges list initialization
			    edges.Add(new Edge(_triangles[topologyIdx - 1], _triangles[topologyIdx]));
			    edges.Add(new Edge(_triangles[topologyIdx], _triangles[topologyIdx + 1]));
			    edges.Add(new Edge(_triangles[topologyIdx + 1], _triangles[topologyIdx - 1]));
			    topologyIdx += 3;  // Update the _triangle access index (TopologyIdx) to step for the next 3 edges iter.
		    }

		    // Sort edges list by vertexA and vertexB values (that represent the indices of the triangles)
		    edges.Sort((e1, e2) =>
		    {
			    int vAcomparison = e1.GetVertexA().CompareTo(e2.GetVertexA());
			    if (vAcomparison == 0) return e1.GetVertexB().CompareTo(e2.GetVertexB());
			    return vAcomparison;
		    });
		    
		    for (int i = 0; i < edges.Count; ++i)
		    {
			    if (i != edges.Count - 1 && edges[i].Equals(edges[i + 1]))  // Duplicated edge is found
			    {
				    // ADD TRACTION SPRING FROM VERTEX_A TO VERTEX_B
				    // Initialize the actual processed edge as GameObjects
				    GameObject tractionSpringGameObject = new GameObject();
				    tractionSpringGameObject.name = "Traction Spring A_" + edges[i].GetVertexA() + " B_" + edges[i].GetVertexB();
				    tractionSpringGameObject.AddComponent<Spring>(); // Assign a spring component into the new GameObject
				    
				    // Retrieve the actual springs
				    Spring tractionSpring = tractionSpringGameObject.GetComponent<Spring>();
				    
				    tractionSpring.GetComponent<Transform>().parent = this.transform;  // Assign MassSpringCloth parent transform
				    
				    // Access the node idx and populate the spring components nodes
				    tractionSpring.nodeA = nodes[edges[i].GetVertexA()];
				    tractionSpring.nodeB = nodes[edges[i].GetVertexB()];
				    tractionSpring.stiffness = 500.0f; // Add a stiffness value to the traction spring
				    
				    // Finally add the spring to the springs list
				    springs.Add(tractionSpring);
				    
				    // FINALLY ADD FLEXION SPRING FROM VERTEX_A TO VERTEX_B
				    // Initialize the actual processed edge as GameObjects
				    GameObject flexionSpringGameObject = new GameObject();
				    flexionSpringGameObject.name = "Flexion Spring A_" + (edges[i + 1].GetVertexA() + 1) + " B_" + (edges[i + 1].GetVertexB() - 1);
				    flexionSpringGameObject.AddComponent<Spring>(); // Assign a spring component into the new GameObject
				    
				    // Retrieve the actual springs
				    Spring flexionSpring = flexionSpringGameObject.GetComponent<Spring>();
				    
				    flexionSpring.GetComponent<Transform>().parent = this.transform;  // Assign MassSpringCloth parent transform
				    
				    // Access the node idx and populate the spring components nodes
				    flexionSpring.nodeA = nodes[edges[i + 1].GetVertexA() + 1];
				    flexionSpring.nodeB = nodes[edges[i + 1].GetVertexB() - 1];
				    flexionSpring.stiffness = 100.0f; // Add a stiffness value to the flexion spring way less (<<) than a traction one
				    
				    // Finally add the spring to the springs list
				    springs.Add(flexionSpring);

				    // Skip the flexion spring index on the edges list and continue iterating
				    i++;
			    }
			    else  // Actual edge is not duplicated
			    {
				    // IN THIS CASE ALWAYS ADD TRACTION SPRING FROM VERTEX_A TO VERTEX_B
				    // Initialize the actual processed edge as GameObjects
				    GameObject tractionSpringGameObject = new GameObject();
				    tractionSpringGameObject.name = "Traction Spring A_" + edges[i].GetVertexA() + " B_" + edges[i].GetVertexB();
				    tractionSpringGameObject.AddComponent<Spring>(); // Assign a spring component into the new GameObject
				    
				    // Retrieve the actual springs
				    Spring tractionSpring = tractionSpringGameObject.GetComponent<Spring>();
				    
				    tractionSpring.GetComponent<Transform>().parent = this.transform;  // Assign MassSpringCloth parent transform
				    
				    // Access the node idx and populate the spring components nodes
				    tractionSpring.nodeA = nodes[edges[i].GetVertexA()];
				    tractionSpring.nodeB = nodes[edges[i].GetVertexB()];
				    tractionSpring.stiffness = 500; // Add a stiffness value to the traction spring
				    
				    // Finally add the spring to the springs list
				    springs.Add(tractionSpring);
			    }
		    }
	    }
	    
	    Debug.Log("Nodes and Springs initialization finished successfully!!!!");
    }

    public void Update()
	{
		if (Input.GetKeyUp (KeyCode.P))
			paused = !paused;
	}

    public void FixedUpdate()
    {
        if (paused)
            return; // Not simulating
        
        // Substeps simulation
        for (int step = 0; step < substeps; ++step)
        {
	        // Select integration method
	        switch (integrationMethod)
	        {
		        case Integration.Explicit:
			        StepExplicit();
			        break;
		        case Integration.Symplectic:
			        StepSymplectic();
			        break;
		        default:
			        throw new System.Exception("[ERROR] Should never happen!");
	        }
        }
        
        // Iterate through every vertex of the mesh and assign it's new position value, previously computed in nodes list
        for (int i = 0; i < nodes.Count; i++) _vertices[i] = transform.InverseTransformPoint(nodes[i].pos);
        _mesh.vertices = _vertices;  // Update mesh vertices position (Local positions)
        // For the springs it is enough to perform this node assignation in the FixedUpdate as long as
        // springs are only used to compute the elastic forces and apply them to the nodes, so only using nodes will work
    }
    
    #endregion

    /// <summary>
    /// Performs a simulation step in 1D using Explicit integration.
    /// </summary>
    private void StepExplicit()
	{
		// Update forces for each node of the mesh
		foreach (Node node in nodes)
		{
			node.force = Vector3.zero;
			node.ComputeForces();
		}

		// Update forces for each spring of the mesh
		foreach (Spring spring in springs)
		{
			spring.ComputeForces();
		}

		// Update explicit position and explicit velocity
		foreach (Node node in nodes)
		{
			if (!node.isFixed)
			{
				node.pos += timeStep * node.vel;
				node.vel += timeStep / node.mass * node.force;
			}
		}
	
		// Update the length of each spring after this step
		foreach (Spring spring in springs)
		{
			spring.UpdateLength();
		}
	}

	/// <summary>
	/// Performs a simulation step in 1D using Symplectic integration.
	/// </summary>
	private void StepSymplectic()
	{
		// Update forces for each node of the mesh
        foreach (Node node in nodes)
        {
            node.force = Vector3.zero;
            node.ComputeForces();
        }
        
        // Update forces for each spring of the mesh
        foreach (Spring spring in springs)
        {
            spring.ComputeForces();
        }

        // Update implicit position and explicit velocity
        foreach (Node node in nodes)
        {
            if (!node.isFixed)
            {
                node.vel += timeStep / node.mass * node.force;
                node.pos += timeStep * node.vel;
            }
        }

        // Update the length of each spring after this step
        foreach (Spring spring in springs)
        {
            spring.UpdateLength();
        }
	}
}
