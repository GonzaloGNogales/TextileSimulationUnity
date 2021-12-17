using MathNet.Numerics.LinearAlgebra;
using UnityEngine;
using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

public class Collideable : MonoBehaviour {
    private MassSpringCloth[] _cloths;
    private Mesh _mesh;
    private Vector3[] _collideableNormals;
    private MatrixXD _N;
    private float _height;
    
    // Start is called before the first frame update
    void Start() {
        /***
        // Math.Numerics Working Test
        Vector3 v3 = new Vector3(1.0f, 2.0f, 3.0f);
        VectorXD vlarge = new DenseVectorXD(3);
        vlarge[0] = v3[0];
        vlarge[1] = v3[1];
        vlarge[2] = v3[2];
        Debug.Log("v3 is => [" + v3.x + " " + v3.y + " " + v3.z + "]");
        Debug.Log("vlarge is => [" + vlarge);

        MatrixXD mlarge = new DenseMatrixXD(3);
        mlarge = DenseMatrixXD.CreateIdentity(3);
        Debug.Log("mlarge is => " + mlarge);

        VectorXD ularge = mlarge.Inverse() * vlarge;
        Vector3 u3 = new Vector3((float) ularge[0], (float) ularge[1], (float) ularge[2]);
        Debug.Log("u3 is => [" + u3.x + " " + u3.y + " " + u3.z + "]");
        /**/
        // Get the mesh filter of the collideable object and allocate it's normals and vertices
        _mesh = GetComponent<MeshFilter>().mesh;
        _collideableNormals = _mesh.normals;
        _height = transform.position.y;
        Debug.Log("Ground is at height => " + _height);
        
        /*for (int i = 0; i < _collideableNormals.Length; ++i)
        {
            VectorXD row = new DenseVectorXD(3);
            row[0] = _collideableNormals[i][0];
            row[1] = _collideableNormals[i][1];
            row[2] = _collideableNormals[i][2];
            N.SetRow(i, row);
        }*/
        MatrixXD N = new DenseMatrixXD(3, 1);
        N[0, 0] = _collideableNormals[0][0];
        N[1, 0] = _collideableNormals[0][1];
        N[2, 0] = _collideableNormals[0][2];
        
        // Transpose computation
        MatrixXD NT = N.Transpose();
        
        // Normal matrix multiplication
        _N = N.Multiply(NT);
        
        // Get all the MassSpringCloths of the scene
        _cloths = FindObjectsOfType<MassSpringCloth>();
        Debug.Log(_cloths.Length + " collideable cloths found in the scene...");
    }

    // Update is called once per frame
    void FixedUpdate() {
        // Add implicit penalty force for modeling collisions
        // For achieving that, it is necessary to loop through every collision possible GameObject (Colliders)

        /***  NOT IMPLICIT METHOD
        foreach (MassSpringCloth cloth in _cloths) {
            foreach (Node node in cloth.nodes) {
                for (int w = 0; w < _collideableVertices.Length; ++w) {
                    Vector3 u = _collideableVertices[w] - node.pos;
                    float delta = Vector3.Dot(_collideableNormals[w], u);
                    float contactStiffness = 300.0f;
                    if (delta > 0) {
                        Debug.Log("Collision Detected! => Penalty Force Computation");
                        node.force += _collideableNormals[w] * delta * contactStiffness;
                    }
                }
            }
        }
        /**/
        
        // Implicit Method
        foreach (MassSpringCloth cloth in _cloths)
        {
            foreach (Node node in cloth.nodes)
            {
                float delta = _height - node.pos.y;
                if (delta > 0)
                {
                    float contactStiffness = 300.0f;
                    MatrixXD penaltyDiff = -contactStiffness * _N;
                    MatrixXD I = DenseMatrixXD.CreateIdentity(3);
                    MatrixXD C = I - Mathf.Pow(0.01f, 2)/node.mass * penaltyDiff;
                    VectorXD F_0 = new DenseVectorXD(3);
                    F_0[0] = node.force[0];
                    F_0[1] = node.force[1];
                    F_0[2] = node.force[2];
                    VectorXD penaltyForce = C.Inverse() * F_0;
                    Vector3 force = new Vector3((float) penaltyForce[0], (float) penaltyForce[1], (float) penaltyForce[2]);
                    node.force = force;
                    Debug.Log(node.force.y);
                }
            }
        }
    }
}
