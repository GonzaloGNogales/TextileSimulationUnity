using UnityEngine;
using System.Collections.Generic;
using UnityEngine.UIElements;
using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic physics manager capable of simulating a given Simulable
/// implementation using diverse integration methods: explicit,
/// implicit, Verlet and semi-implicit.
/// </summary>
public class MassSpringCloth : MonoBehaviour {
	/// <summary>
	/// Default constructor. Zero all. 
	/// </summary>
	public MassSpringCloth() {
		paused = true;
		substeps = 1;
		timeStep = 0.01f;  // Default value 0.01f
		gravity = new Vector3 (0.0f, -9.81f, 0.0f);
		wind = new Vector3(0.0f, 0.0f, 0.0f);
		integrationMethod = Integration.SymplecticWithImplicitCollisions;
	}

	/// <summary>
	/// Integration method.
	/// </summary>
	public enum Integration {
		Explicit = 0,
		// Semi-Implicit Euler Integration Explicit in Velocity and Implicit in Position
		Symplectic = 1,
		SymplecticWithImplicitCollisions = 2,
		DirectImplicitCollisions = 3,
		// Implicit Euler Integration (Unknown forces to compute new velocities and positions) - predictions-based
		Implicit = 4
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
    private int _actualWindAnimationIteration;
    private int windUp;

    private class Edge {
	    private int vertexA;
	    private int vertexB;
	    private int vertexOther;
	    
	    public Edge(int vA, int vB, int vO) {
		    if (vA <= vB) {  // The smallest vertex index is always in the first position
			    vertexA = vA;
			    vertexB = vB;
		    }
		    else {
			    vertexA = vB;
			    vertexB = vA;
		    }

		    vertexOther = vO;
	    }

	    public int GetVertexA() {
		    return vertexA;
	    }
	    
	    public int GetVertexB() {
		    return vertexB;
	    }

	    public int GetVertexOther() {
		    return vertexOther;
	    }
	    
	    public override bool Equals(object obj) {
		    // If the passed object is null
		    if (obj == null)
			    return false;
		    if (!(obj is Edge))
			    return false;
		    return (vertexA == ((Edge)obj).vertexA)
		           && (vertexB == ((Edge)obj).vertexB);
	    }
	    
	    public override int GetHashCode() {
		    return (vertexA, vertexB, vertexOther).GetHashCode();
	    }
    }
    
    #endregion

    #region MonoBehaviour

    public void Start() {
	    // Retrieve the mesh filter component
	    _mesh = GetComponent<MeshFilter>().mesh;
	    _actualWindAnimationIteration = 0;
	    // timeStep /= substeps;  // This allows setting more substeps while keep adjusting the speed of the simulation (timeStep normalization)
	    Debug.Log("New substep: " + timeStep);
	    
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
		    
		    ///////////////////////// Should work both ways /////////////////////////////
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
	    if (_triangles.Length % 3 == 0) {  // Check if the number of indices is exact for a number of triangles
		    int topologyIdx = 1;
		    for (int i = 0; i < _triangles.Length / 3; ++i) {  // Initialize edges vertex _triangles indices
			    // POSSIBLE OPTIMIZATION FOR MILLION VERTEX MESHES: CREATE NODES IN THIS LOOP!!! (Triangle iteration)
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
		    
		    for (int i = 0; i < edges.Count; ++i) {
			    if (i != edges.Count - 1 && edges[i].Equals(edges[i + 1])) {  // Duplicated edge is found
				    // ADD TRACTION SPRING FROM VERTEX_A TO VERTEX_B
				    // Initialize the actual processed edge as GameObjects
				    GameObject tractionSpringGameObject = new GameObject();
				    tractionSpringGameObject.name = "Traction Spring A_" + edges[i].GetVertexA() + " B_" + edges[i].GetVertexB();
				    tractionSpringGameObject.AddComponent<Spring>(); // Assign a spring component into the new GameObject
				    
				    // Retrieve the actual springs
				    Spring tractionSpring = tractionSpringGameObject.GetComponent<Spring>();
				    
				    tractionSpring.GetComponent<Transform>().parent = transform;  // Assign MassSpringCloth parent transform
				    
				    // Access the node idx and populate the spring components nodes
				    tractionSpring.nodeA = nodes[edges[i].GetVertexA()];
				    tractionSpring.nodeB = nodes[edges[i].GetVertexB()];
				    tractionSpring.SubstepStartLengtUpdate();
				    tractionSpring.stiffness = 500.0f; // Add a stiffness value to the traction spring
				    
				    // Finally add the spring to the springs list
				    springs.Add(tractionSpring);
				    
				    // FINALLY ADD FLEXION SPRING FROM VERTEX_A TO VERTEX_B
				    // Initialize the actual processed edge as GameObjects
				    GameObject flexionSpringGameObject = new GameObject();
				    flexionSpringGameObject.name = "Flexion Spring A_" + edges[i].GetVertexOther() + " B_" + edges[i + 1].GetVertexOther();
				    flexionSpringGameObject.AddComponent<Spring>(); // Assign a spring component into the new GameObject
				    
				    // Retrieve the actual springs
				    Spring flexionSpring = flexionSpringGameObject.GetComponent<Spring>();
				    
				    flexionSpring.GetComponent<Transform>().parent = transform;  // Assign MassSpringCloth parent transform
				    
				    // Access the node idx and populate the spring components nodes
				    flexionSpring.nodeA = nodes[edges[i].GetVertexOther()];
				    flexionSpring.nodeB = nodes[edges[i + 1].GetVertexOther()];
				    flexionSpring.SubstepStartLengtUpdate();
				    flexionSpring.stiffness = 50.0f; // Add a stiffness value to the flexion spring way less (<<) than a traction one
				    
				    // Finally add the spring to the springs list
				    springs.Add(flexionSpring);

				    // Skip the flexion spring index on the edges list and continue iterating
				    i++;
			    }
			    else {  // Actual edge is not duplicated
				    // IN THIS CASE ALWAYS ADD TRACTION SPRING FROM VERTEX_A TO VERTEX_B
				    // Initialize the actual processed edge as GameObjects
				    GameObject tractionSpringGameObject = new GameObject();
				    tractionSpringGameObject.name = "Traction Spring A_" + edges[i].GetVertexA() + " B_" + edges[i].GetVertexB();
				    tractionSpringGameObject.AddComponent<Spring>(); // Assign a spring component into the new GameObject
				    
				    // Retrieve the actual springs
				    Spring tractionSpring = tractionSpringGameObject.GetComponent<Spring>();
				    
				    tractionSpring.GetComponent<Transform>().parent = transform;  // Assign MassSpringCloth parent transform
				    
				    // Access the node idx and populate the spring components nodes
				    tractionSpring.nodeA = nodes[edges[i].GetVertexA()];
				    tractionSpring.nodeB = nodes[edges[i].GetVertexB()];
				    tractionSpring.SubstepStartLengtUpdate();
				    tractionSpring.stiffness = 500.0f; // Add a stiffness value to the traction spring
				    
				    // Finally add the spring to the springs list
				    springs.Add(tractionSpring);
			    }
		    }
	    }
	    
	    Debug.Log("Nodes and Springs initialization finished successfully!!!!");
    }

    public void Update() {
		if (Input.GetKeyUp (KeyCode.P))
			paused = !paused;
		
		// Iterate through every vertex of the mesh and assign it's new position value, previously computed in nodes list
		for (int i = 0; i < nodes.Count; i++) _vertices[i] = transform.InverseTransformPoint(nodes[i].pos);
		_mesh.vertices = _vertices;  // Update mesh vertices position (Local positions)
		// For the springs it is enough to perform this node assignation in the FixedUpdate as long as
		// springs are only used to compute the elastic forces and apply them to the nodes, so only using nodes will work
	}

    public void FixedUpdate() {
        if (paused)
            return; // Not simulating
        
        // Substeps simulation
        for (int i = 0; i < substeps; ++i)
        {
	        // Wind simulation implemented
	        /***
	        windUp++;
	        if (windUp == 1000)
	        {
		        windUp = 0;
		        _actualWindAnimationIteration++;  // Variable to update the actual wind iteration
		        wind = new Vector3(0.0f, 0.0f, -6.0f);
	        }
	        if (_actualWindAnimationIteration % 2 == 0)
	        {
		        if (windUp == 300)
		        {
			        wind -= new Vector3(0.0f, 0.0f, 5.0f);
		        }
		        else if (windUp == 600)
		        {
			        wind -= new Vector3(0.0f, 0.0f, 2.5f);
		        }
	        }
	        /**/
	        
	        // Select integration method
	        switch (integrationMethod)
	        {
		        case Integration.Explicit:
			        StepExplicit();
			        break;
		        case Integration.Symplectic:
			        StepSymplectic();
			        break;
		        case Integration.SymplecticWithImplicitCollisions:
			        StepSymplecticWithImplicitCollisions();
			        break;
		        case Integration.DirectImplicitCollisions:
			        StepDirectImplicitCollisionsComputation();
			        break;
		        case Integration.Implicit:
			        StepImplicit();
			        break;
		        default:
			        throw new System.Exception("[ERROR] Should never happen!");
	        }
        }
    }
    
    #endregion

    /// <summary>
    /// Performs a simulation step in 1D using Explicit integration.
    /// </summary>
    private void StepExplicit() {
		// Update forces for each node of the mesh
		foreach (Node node in nodes) {
			node.force = Vector3.zero;
			node.ComputeForces();
		}

		// Update forces for each spring of the mesh
		foreach (Spring spring in springs) {
			spring.ComputeForces();
		}

		// Update explicit position and explicit velocity
		foreach (Node node in nodes) {
			if (!node.isFixed) {
				node.pos += timeStep * node.vel;
				node.vel += timeStep / node.mass * node.force;
			}
		}
	
		// Update the length of each spring after this step
		foreach (Spring spring in springs) {
			spring.UpdateLength();
		}
	}

	/// <summary>
	/// Performs a simulation step in 1D using Symplectic integration.
	/// </summary>
	private void StepSymplectic() {
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

	/// <summary>
	/// Performs a simulation step in 1D using Symplectic integration and Implicit collisions computation.
	/// </summary>
	private void StepSymplecticWithImplicitCollisions() {
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
			if (!node.isFixed && !node.inCollision)
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
	
	/// <summary>
	/// Performs a simulation step in 1D using Collision Implicit integration.
	/// </summary>
	private void StepDirectImplicitCollisionsComputation()
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
				MatrixXD C = new DenseMatrixXD(3);
				if (node.pos.y < -1.0000f)
				{
					// Contact normal and contact stiffness initialization when collision
					float contactStiffness = 10000.0f;
					MatrixXD I = DenseMatrixXD.CreateIdentity(3);
					VectorXD n = new DenseVectorXD(3);
					n[0] = 0;
					n[1] = 1;
					n[2] = 0;
				
					// Transpose computation
					//MatrixXD nt = n.OuterProduct();
				
					// Normal matrix multiplication
					MatrixXD nMat = n.OuterProduct(n); // OuterProduct performs N.Transpose()
					
					// Penalty force differential with respect to node position
					MatrixXD penaltyDiff = - contactStiffness * nMat;
					
					// Penalty force computation
					Vector3 penetration = node.pos -  new Vector3(0.0f, -1.0000f, 0.0f);
					float delta = Vector3.Dot(penetration, new Vector3(0.0f, 1.0f, 0.0f));
					VectorXD penaltyForceDense = penaltyDiff * (delta * n);
					Vector3 penaltyForce = new Vector3((float) penaltyForceDense[0], (float) penaltyForceDense[1], (float) penaltyForceDense[2]);
					
					// node.force is a prediction of the future forces calculated by approximation
					// node.force = F(0) + F(h) (collisions implicit penalty forces)
					node.force += penaltyForce;
					
					// Collision implicit operand calculation
					C = I - timeStep * timeStep / node.mass * penaltyDiff;
				}
				else
				{
					// If there is no ground collision dF/dx is 0 so we just need the identity
					C = DenseMatrixXD.CreateIdentity(3);
				}

				Vector3 antF = node.vel + timeStep / node.mass * node.force;
				VectorXD antFDense = new DenseVectorXD(3);
				antFDense[0] = antF[0];
				antFDense[1] = antF[1];
				antFDense[2] = antF[2];
				
				VectorXD newVelDense = C.Inverse() * antFDense;
				Vector3 newVel = new Vector3((float) newVelDense[0], (float) newVelDense[1], (float) newVelDense[2]);
				
				// New velocity implicit assignment
				node.vel = newVel;
				node.pos += timeStep * node.vel;
			}
		}

		// Update the length of each spring after this step
		foreach (Spring spring in springs)
		{
			spring.UpdateLength();
		}
	}
	
	/// <summary>
	/// Performs a simulation step in 1D using General Implicit integration.
	/// </summary>
	private void StepImplicit() {
		// Evaluate forces
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
		
		// Derivatives matrices computation
		
		// Linear system solving A * v(t+h) = b => v(t+h) = A^-1 * b
		// A = massMat + timeStep * DampingMat + timeStep*timeStep * StiffnessMat
		// b = (massMat + timeStep * DampingMat) * node.vel + timeStep * node.force

		// Velocity and position implicit integration
		foreach (Node node in nodes)
		{
			if (!node.isFixed)
			{
				// I need to find forceDiffPosition and forceDiffVelocity ...
				float forceDiffPosition = 1.0f;
				float forceDiffVelocity = 1.0f;
				node.vel = ((node.mass - timeStep * forceDiffVelocity) * node.vel + timeStep * node.force) / 
				           (node.mass - timeStep * forceDiffVelocity - timeStep*timeStep * forceDiffPosition);
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
