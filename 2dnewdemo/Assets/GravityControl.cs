using UnityEngine;
using System.Collections;

public class GravityControl : MonoBehaviour {

	// Use this for initialization
	void Start () {
	
	}
	
	// Update is called once per frame
	void Update () 
    {
        Debug.Log(Input.acceleration.x);
        Debug.Log(Input.acceleration.y);
  
	}
}
