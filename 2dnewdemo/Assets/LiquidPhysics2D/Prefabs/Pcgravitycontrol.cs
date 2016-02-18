using UnityEngine;
using System.Collections;

public class Pcgravitycontrol : MonoBehaviour {
    [Tooltip("The world with that gravity")]
    public LPManager world;
	// Use this for initialization
	void Start () 
    {
	
	}
	
	// Update is called once per frame
	void Update () 
    {
        LPAPIWorld.SetWorldGravity(world.GetPtr(), Input.acceleration.x*9,Input.acceleration.y*9);
	}
}
