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
    private Dictionary<int, List<Spring>> _springsReferences;
    private Dictionary<Spring, GameObject> _springGameObjects;

    private class Edge
    {
	    private int vertexA;
	    private int vertexB;
	    private int vertexOther;

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
	    
	    public override int GetHashCode() =>
		    (vertexA, vertexB, vertexOther).GetHashCode();
    }
    
    #endregion

    #region MonoBehaviour

    public void Start()
    {
	    // Retrieve the mesh filter component
	    _mesh = GetComponent<MeshFilter>().mesh;

	    // Initialize dictionaries
	    _springsReferences = new Dictionary<int, List<Spring>>();
	    _springGameObjects = new Dictionary<Spring, GameObject>();
	    
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
		    
		    node.GetComponent<Transform>().parent = transform;  // Assign this MassSpringCloth as parent transform of the nodes
		    
		    ////////////////////////// Should work both ways ////////////////////////////
		    //node.GetComponent<Transform>().position = transform.TransformPoint(vertex);  // Locate the nodes in the positions of the mesh vertices
		    node.pos = transform.TransformPoint(vertex);  // Locate the nodes in the positions of the mesh vertices (Global positions)
		    /////////////////////////////////////////////////////////////////////////////
		    
		    node.mass = 1.0f;  // Set node mass

		    nodes.Add(node);  // Finally add node to the nodes list to iterate through all of them on fixed update loop
		    idxNode++;
	    }

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
			    // FIRST ALWAYS ADD TRACTION SPRING FROM VERTEX_A TO VERTEX_B (3 SPRINGS PER TRIANGLE)
			    // Initialize the actual processed edge as GameObjects
			    GameObject tractionSpringGameObject1 = new GameObject();
			    tractionSpringGameObject1.name = "Traction Spring A_" + _triangles[topologyIdx - 1] + " B_" + _triangles[topologyIdx];
			    tractionSpringGameObject1.AddComponent<Spring>(); // Assign a spring component into the new GameObject
			    
			    GameObject tractionSpringGameObject2 = new GameObject();
			    tractionSpringGameObject2.name = "Traction Spring A_" + _triangles[topologyIdx] + " B_" + _triangles[topologyIdx + 1];
			    tractionSpringGameObject2.AddComponent<Spring>(); // Assign a spring component into the new GameObject
			    
			    GameObject tractionSpringGameObject3 = new GameObject();
			    tractionSpringGameObject3.name = "Traction Spring A_" + _triangles[topologyIdx + 1] + " B_" + _triangles[topologyIdx - 1];
			    tractionSpringGameObject3.AddComponent<Spring>(); // Assign a spring component into the new GameObject
				    
			    // Retrieve the actual springs
			    Spring tractionSpring1 = tractionSpringGameObject1.GetComponent<Spring>();
			    Spring tractionSpring2 = tractionSpringGameObject2.GetComponent<Spring>();
			    Spring tractionSpring3 = tractionSpringGameObject3.GetComponent<Spring>();
				    
			    tractionSpring1.GetComponent<Transform>().parent = transform;  // Assign MassSpringCloth parent transform
			    tractionSpring2.GetComponent<Transform>().parent = transform;  // Assign MassSpringCloth parent transform
			    tractionSpring3.GetComponent<Transform>().parent = transform;  // Assign MassSpringCloth parent transform
				    
			    // Access the node idx and populate the spring components nodes
			    tractionSpring1.nodeA = nodes[_triangles[topologyIdx - 1]];
			    tractionSpring1.nodeB = nodes[_triangles[topologyIdx]];
			    tractionSpring1.stiffness = 500; // Add a stiffness value to the traction spring
			    
			    tractionSpring2.nodeA = nodes[_triangles[topologyIdx]];
			    tractionSpring2.nodeB = nodes[_triangles[topologyIdx + 1]];
			    tractionSpring2.stiffness = 500; // Add a stiffness value to the traction spring
			    
			    tractionSpring3.nodeA = nodes[_triangles[topologyIdx + 1]];
			    tractionSpring3.nodeB = nodes[_triangles[topologyIdx - 1]];
			    tractionSpring3.stiffness = 500; // Add a stiffness value to the traction spring
				    
			    // Finally add the spring to the springs list and to the dictionaries
			    springs.Add(tractionSpring1);
			    springs.Add(tractionSpring2);
			    springs.Add(tractionSpring3);
			    
			    _springGameObjects.Add(tractionSpring1, tractionSpringGameObject1);
			    _springGameObjects.Add(tractionSpring2, tractionSpringGameObject2);
			    _springGameObjects.Add(tractionSpring3, tractionSpringGameObject3);

			    if (!_springsReferences.ContainsKey(_triangles[topologyIdx - 1]))
			    {
				    _springsReferences.Add(_triangles[topologyIdx - 1], new List<Spring>());
				    _springsReferences[_triangles[topologyIdx - 1]].Add(tractionSpring1);
			    }
			    else
			    {
				    _springsReferences[_triangles[topologyIdx - 1]].Add(tractionSpring1);
			    }
			    
			    if (!_springsReferences.ContainsKey(_triangles[topologyIdx]))
			    {
				    _springsReferences.Add(_triangles[topologyIdx], new List<Spring>());
				    _springsReferences[_triangles[topologyIdx]].Add(tractionSpring2);
			    }
			    else
			    {
				    _springsReferences[_triangles[topologyIdx]].Add(tractionSpring2);
			    }
			    
			    if (!_springsReferences.ContainsKey(_triangles[topologyIdx + 1]))
			    {
				    _springsReferences.Add(_triangles[topologyIdx + 1], new List<Spring>());
				    _springsReferences[_triangles[topologyIdx + 1]].Add(tractionSpring3);
			    }
			    else
			    {
				    _springsReferences[_triangles[topologyIdx + 1]].Add(tractionSpring3);
			    }
			    
			    // Edges list initialization
			    edges.Add(new Edge(_triangles[topologyIdx - 1], _triangles[topologyIdx], _triangles[topologyIdx + 1]));
			    edges.Add(new Edge(_triangles[topologyIdx], _triangles[topologyIdx + 1], _triangles[topologyIdx - 1]));
			    edges.Add(new Edge(_triangles[topologyIdx + 1], _triangles[topologyIdx - 1], _triangles[topologyIdx]));
			    topologyIdx += 3;  // Update the _triangle access index (TopologyIdx) to step for the next 3 edges iter.
		    }

		    // Sort edges list by vertexA and vertexB values (that represent the indices of the triangles)
		    edges.Sort((e1, e2) =>
		    {
			    int vAcomparison = e1.GetVertexA().CompareTo(e2.GetVertexA());
			    if (vAcomparison == 0) return e1.GetVertexB().CompareTo(e2.GetVertexB());
			    return vAcomparison;
		    });
		    
		    // Flexion springs creation (detect duplicated springs and create the corresponding traction and flexion springs)
		    for (int i = 0; i < edges.Count; ++i)
		    {
			    if (i != edges.Count - 1 && edges[i].Equals(edges[i + 1]))  // Duplicated edge is found
			    {
				    // REMOVE THE DETECTED DUPLICATED EDGE FROM SPRING LIST AND UPDATE IT WITH THE OPPOSITE DIAGONAL SPRING
				    // First remove the spring from the springs list and from the scene as it is a GameObject
				    foreach (Spring spring in _springsReferences[edges[i].GetVertexA()])
				    {
					    if (spring.nodeB.Equals(nodes[edges[i].GetVertexB()]))
					    {
						    springs.Remove(spring);
						    Destroy(_springGameObjects[spring]);
					    }
				    }

				    // FINALLY ADD FLEXION SPRING FROM edge[i].GetVertexOther() TO edge[i + 1].GetVertexOther()
				    // Initialize the actual processed edge as GameObjects
				    GameObject flexionSpringGameObject = new GameObject();
				    flexionSpringGameObject.name = "Flexion Spring A_" + edges[i].GetVertexOther() + " B_" + edges[i + 1].GetVertexOther();
				    flexionSpringGameObject.AddComponent<Spring>(); // Assign a spring component into the new GameObject
				    
				    // Retrieve the actual springs
				    Spring flexionSpring = flexionSpringGameObject.GetComponent<Spring>();
				    
				    flexionSpring.GetComponent<Transform>().parent = this.transform;  // Assign MassSpringCloth parent transform
				    
				    // Access the node idx and populate the spring components nodes
				    flexionSpring.nodeA = nodes[edges[i].GetVertexOther()];
				    flexionSpring.nodeB = nodes[edges[i + 1].GetVertexOther()];
				    flexionSpring.stiffness = 100.0f; // Add a stiffness value to the flexion spring way less (<<) than a traction one
				    
				    // Finally add the spring to the springs list
				    springs.Add(flexionSpring);

				    // Skip the flexion spring index on the edges list and continue iterating
				    i++;
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
