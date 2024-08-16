using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f; // time step
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity of the whole rigid body
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity of the whole rigid body
	// the other two states are position and orientation
	//x: transform.position, q: transform.rotation

	Vector3 max_point 	= new Vector3(-999 , -999, -999);	// the maximum point of the mesh(original frame)
	Vector3 min_point 	= new Vector3(999, 999, 999);	// the minimum point of the mesh

	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				

	//serialize and could be modified in the editor
	[Range(0.0f, 1.0f)]
	public float restitution 	= 0.5f;					// for collision
	[Range(0.0f, 1.0f)]
	public float friction		= 0.5f;					// for collision

	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];

			max_point.x = Mathf.Max(max_point.x, vertices[i].x);
			max_point.y = Mathf.Max(max_point.y, vertices[i].y);
			max_point.z = Mathf.Max(max_point.z, vertices[i].z);
			min_point.x = Mathf.Min(min_point.x, vertices[i].x);
			min_point.y = Mathf.Min(min_point.y, vertices[i].y);
			min_point.z = Mathf.Min(min_point.z, vertices[i].z);
		}
		I_ref [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}
	Matrix4x4 SubMatrices(Matrix4x4 a, Matrix4x4 b)
    {
        Matrix4x4 result = new Matrix4x4();

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                result[i, j] = a[i, j] - b[i, j];
            }
        }
        return result;
    }

	Matrix4x4 MultiplyMatrixByScalar(Matrix4x4 matrix, float scalar){
        Matrix4x4 result = new Matrix4x4();

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                result[i, j] = matrix[i, j] * scalar;
            }
        }
        return result;
    }

	// In this function, update v and w by the impulse due to the collision 
	// the plane is defined by P and N, where P is a point on the plane, and N is the normal of the plane
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		// check if the collision happens, use all the vertices of the mesh (can use bounding box for acceleration)
		Vector3 v_average = new Vector3(0, 0, 0);
		Vector3 P_average = new Vector3(0, 0, 0);
		int count = 0;
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;
		for (int i=0; i<vertices.Length; i++)
		{
			// the position of vetices should multiply the rotation matrix
			Vector3 P_i = transform.rotation * vertices[i] + transform.position;
			// check if the i-th vertex is inside the collision plane
			if(Vector3.Dot(P_i - P, N) >= 0) continue;
			// the radius vector from the center of mass to the point
			
			Vector3 Rr_i = P_i - transform.position;
			Vector3 v_i = v + Vector3.Cross(w, Rr_i);
			// if(float.IsNaN(v_i.x) || float.IsNaN(v_i.y) || float.IsNaN(v_i.z)){
			// 	print(v+" "+w+" "+Rr_i+" "+P_i+" "+transform.position+" "+vertices[i]);
			// }
			// if NaN appeared in v_i, skip this vertex
			// the relative velocity of the i-th vertex is away from the collision plane
			if(Vector3.Dot(v_i, N) >= 0) continue;

			// use the average point of the vertices inside the collision plane
			P_average += P_i;
			v_average += v_i;
			count ++;
		}
		if(count == 0) return;

		Matrix4x4 rotation_matrix = Matrix4x4.Rotate(transform.rotation);
		Matrix4x4 I_inv = rotation_matrix * I_ref.inverse * rotation_matrix.transpose;

		P_average /= count;
		v_average /= count;

		// split the velocity into normal and tangential components
		Vector3 v_n_i = Vector3.Dot(v_average, N) * N;
		Vector3 v_t_i = v_average - v_n_i;

		Vector3 Rr_average = P_average - transform.position;

		// calculate friction coefficient
		float a = 0;
		if(v_t_i.magnitude > 0.001) a = 1 - friction*(1 + restitution)*v_n_i.magnitude/v_t_i.magnitude;
		if(a < 0) a = 0;

		Vector3 v_i_new = a*v_t_i - restitution*v_n_i;
		Matrix4x4 Rr_i_dual = Get_Cross_Matrix(Rr_average);

		Matrix4x4 K = SubMatrices(
			MultiplyMatrixByScalar(Matrix4x4.identity,1.0f/mass) , 
			Rr_i_dual*I_inv*Rr_i_dual);
		Vector3 impulse = K.inverse.MultiplyVector(v_i_new - v_average);

		v += impulse/mass; // update linear 
		w += I_inv.MultiplyVector(Vector3.Cross(Rr_average,impulse)); // update angular velocity

		// print(a + " " + v_t_i + " " + v_n_i.magnitude + " " + v_t_i.magnitude);
	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			transform.rotation = new Quaternion (0, 0, 0, 1);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			w = new Vector3 (0, 0, 0);
			launched=true;
		}
		if(!launched){
			return;
		}
		// using leapfrog integration, first velocity, then position

		// Part I: Update velocities
		v *= linear_decay;
		w *= angular_decay;

		v += new Vector3(0, -9.8f, 0) * dt; // always apply gravity, from a global perspective

		// Part II: Collision Impulse

		Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0)); // ground

		Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0)); // wall

		// Part III: Update position & orientation
		//Update linear status
		Vector3    x = transform.position;
		//Update angular status
		Quaternion q = transform.rotation;
		// print(v);
		x += v * dt;
		// using quaternion to update orientation
		Quaternion dq = (new Quaternion(w.x*dt/2, w.y*dt/2, w.z*dt/2, 0)) * q;
		q.x += dq.x; q.y += dq.y; q.z += dq.z; q.w += dq.w;
		// Part IV: Assign to the object
		transform.position = x;
		transform.rotation = q;
	}
}
