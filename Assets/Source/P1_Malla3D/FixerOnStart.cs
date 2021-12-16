using System.Collections.Generic;
using UnityEngine;

public class FixerOnStart : MonoBehaviour {
    private BoxCollider _fixerCollider;
    private Vector3 _initialFixerPosition;
    private List<Node> _onStartFixedNodes;
    
    // Start is called before the first frame update
    void Start() {
        // Get the BoxCollider component of every fixer in the scene
        _fixerCollider = GetComponent<BoxCollider>();
        _initialFixerPosition = transform.position;  // Save the initial fixer position to compute movement delta
        Debug.Log("Fixer in position " + _initialFixerPosition + " initialized!!!");
        
        // Get all the MassSpringCloths of the scene
        MassSpringCloth[] cloths = FindObjectsOfType<MassSpringCloth>();
        Debug.Log(cloths.Length + " cloths found in the scene...");

        // Initialize the fixed nodes list and check if nodes are contained inside the fixer collider bounds
        _onStartFixedNodes = new List<Node>();
        foreach (MassSpringCloth cloth in cloths) {
            List<Node> clothNodes = cloth.nodes;
            foreach (Node node in clothNodes) {
                // Transform the node position from local coordinates to global with the fixer transform
                //Vector3 nodeLocalPosition = transform.InverseTransformPoint(node.pos);  // Local position of the node with respect to the fixer
                //Vector3 nodeGlobalPosition = transform.TransformPoint(nodeLocalPosition);  // Global position of the node with respect to the fixer
                if (_fixerCollider.bounds.Contains(node.pos)) {            
                    node.isFixed = true; 
                    _onStartFixedNodes.Add(node);
                }
            }
        }
        
        Debug.Log("Nodes have been fixed on start!!!");
    }

    // Update is called once per frame
    void Update() {
        // Get the fixer position in global coordinates
        Vector3 finalFixerPosition = transform.position;
        
        // Iterate only through the initially fixed nodes to update their positions, or unfix in case that the collision ends
        foreach (Node node in _onStartFixedNodes) {
            // Transform the node position from local coordinates to global with the fixer transform
            // Vector3 nodeLocalPosition = transform.InverseTransformPoint(node.pos);  // Local position of the node with respect to the fixer
            // Vector3 nodeGlobalPosition = transform.TransformPoint(nodeLocalPosition);  // Global position of the node with respect to the fixer
            if (_fixerCollider.bounds.Contains(node.pos)) {            
                Vector3 movementQuantity = finalFixerPosition - _initialFixerPosition;  // Measure the fixer movement in xyz axis
                node.pos += movementQuantity;  // Add the movement quantity measured to the fixed nodes position
            }
            else {
                node.isFixed = false;
            }
        }

        _initialFixerPosition = finalFixerPosition; // Assign the final position of the fixer
    }
}
