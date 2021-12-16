using UnityEngine;

public class Collideable : MonoBehaviour {
    private MassSpringCloth[] _cloths;
    private Mesh _mesh;
    private Vector3[] _collideableNormals;
    private Vector3[] _collideableVertices;
    
    // Start is called before the first frame update
    void Start() {
        // Get the mesh filter of the collideable object and allocate it's normals and vertices
        _mesh = GetComponent<MeshFilter>().mesh;
        _collideableNormals = _mesh.normals;
        _collideableVertices = _mesh.vertices;

        // Get all the MassSpringCloths of the scene
        _cloths = FindObjectsOfType<MassSpringCloth>();
        Debug.Log(_cloths.Length + " collideable cloths found in the scene...");
    }

    // Update is called once per frame
    void Update() {
        // Add penalty force for modeling collisions
        // For achieving that, it is necessary to loop through every collision possible GameObject (Colliders)
        // Just multiply => Contact Normal: n * Delta Penetration Quantity: Vector3.Dot(Contact Normal, (Collider Collision Point - Node Position)) * Contact Stiffness: Relative to Mass?

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
    }
}
