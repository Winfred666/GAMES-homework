using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class implicit_model : MonoBehaviour
{
	float 		t 		= 0.0333f;
	float 		mass	= 1;
	float		damping	= 0.99f;

	// this is the update rate of the simulation
	[Range(0.0f, 1.0f)]
	public float 		rho		= 0.995f;
	
	float 		spring_k = 8000;
	int[] 		E; // all non-repeated edges, used for spring force calculation
	float[] 	L; // rest length of each edge
	Vector3[] 	V; // velocity of each vertex

	// make the iteration number serializable
	[SerializeField]
	public int iteration_num = 32;

	[SerializeField]
	public float shpere_radius = 2.7f;

	[SerializeField]
	public int n = 21;

    // Start is called before the first frame update
    void Start()
    {
		Mesh mesh = GetComponent<MeshFilter> ().mesh;

		//Resize the mesh.
		Vector3[] X  	= new Vector3[n*n];
		Vector2[] UV 	= new Vector2[n*n];
		int[] triangles	= new int[(n-1)*(n-1)*6];
		for(int j=0; j<n; j++)
		for(int i=0; i<n; i++)
		{
			X[j*n+i] =new Vector3(5-10.0f*i/(n-1), 0, 5-10.0f*j/(n-1));
			UV[j*n+i]=new Vector3(i/(n-1.0f), j/(n-1.0f));
		}
		int t=0;
		for(int j=0; j<n-1; j++)
		for(int i=0; i<n-1; i++)	
		{
			triangles[t*6+0]=j*n+i;
			triangles[t*6+1]=j*n+i+1;
			triangles[t*6+2]=(j+1)*n+i+1;
			triangles[t*6+3]=j*n+i;
			triangles[t*6+4]=(j+1)*n+i+1;
			triangles[t*6+5]=(j+1)*n+i;
			t++;
		}
		mesh.vertices=X;
		mesh.triangles=triangles;
		mesh.uv = UV;
		mesh.RecalculateNormals ();


		//Construct the original E
		int[] _E = new int[triangles.Length*2];
		for (int i=0; i<triangles.Length; i+=3) 
		{
			_E[i*2+0]=triangles[i+0];
			_E[i*2+1]=triangles[i+1];
			_E[i*2+2]=triangles[i+1];
			_E[i*2+3]=triangles[i+2];
			_E[i*2+4]=triangles[i+2];
			_E[i*2+5]=triangles[i+0];
		}
		//Reorder the original edge list
		for (int i=0; i<_E.Length; i+=2)
			if(_E[i] > _E[i + 1]) 
				Swap(ref _E[i], ref _E[i+1]);
		//Sort the original edge list using quicksort
		Quick_Sort (ref _E, 0, _E.Length/2-1);

		int e_number = 0;
		for (int i=0; i<_E.Length; i+=2)
			if (i == 0 || _E [i + 0] != _E [i - 2] || _E [i + 1] != _E [i - 1]) 
					e_number++;

		E = new int[e_number * 2];
		for (int i=0, e=0; i<_E.Length; i+=2)
			if (i == 0 || _E [i + 0] != _E [i - 2] || _E [i + 1] != _E [i - 1]) 
			{
				E[e*2+0]=_E [i + 0]; // edge start
				E[e*2+1]=_E [i + 1]; // edge end
				e++;
			}

		L = new float[E.Length/2];
		for (int e=0; e<E.Length/2; e++) 
		{
			int v0 = E[e*2+0];
			int v1 = E[e*2+1];
			L[e]=(X[v0]-X[v1]).magnitude;
		}

		V = new Vector3[X.Length];
		for (int i=0; i<V.Length; i++)
			V[i] = new Vector3 (0, 0, 0);
    }

    void Quick_Sort(ref int[] a, int l, int r)
	{
		int j;
		if(l<r)
		{
			j=Quick_Sort_Partition(ref a, l, r);
			Quick_Sort (ref a, l, j-1);
			Quick_Sort (ref a, j+1, r);
		}
	}

	int  Quick_Sort_Partition(ref int[] a, int l, int r)
	{
		int pivot_0, pivot_1, i, j;
		pivot_0 = a [l * 2 + 0];
		pivot_1 = a [l * 2 + 1];
		i = l;
		j = r + 1;
		while (true) 
		{
			do ++i; while( i<=r && (a[i*2]<pivot_0 || a[i*2]==pivot_0 && a[i*2+1]<=pivot_1));
			do --j; while(  a[j*2]>pivot_0 || a[j*2]==pivot_0 && a[j*2+1]> pivot_1);
			if(i>=j)	break;
			Swap(ref a[i*2], ref a[j*2]);
			Swap(ref a[i*2+1], ref a[j*2+1]);
		}
		Swap (ref a [l * 2 + 0], ref a [j * 2 + 0]);
		Swap (ref a [l * 2 + 1], ref a [j * 2 + 1]);
		return j;
	}

	void Swap(ref int a, ref int b)
	{
		int temp = a;
		a = b;
		b = temp;
	}

	void Collision_Handling()
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;
		// find the sphere object
		GameObject sphere = GameObject.Find("Sphere");
		//Handle colllision.
		for(int i=0; i<X.Length; i++)
		{
			// calculate the distance to the shpere center
			Vector3 dir_i = (X[i] - sphere.transform.position);
			float d = dir_i.magnitude;
			if(d <= shpere_radius){
				dir_i.Normalize();
				V[i] += 1/t*(sphere.transform.position + shpere_radius*dir_i - X[i]);
				X[i] = sphere.transform.position + shpere_radius*dir_i;
			}
		}
		mesh.vertices = X;
	}

	//G is the output gradient list, a 3N vector.s 
	void Get_Gradient(Vector3[] X, Vector3[] X_hat, float t_inv2, Vector3[] G)
	{
		//Momentum and Gravity.
		// add gravity to G.
		for(int i=0;i<X.Length;i++){
			G[i] = new Vector3(0, 9.8f, 0); // gravity of each vertex (because force term is negative, so gravity term is positive)
			G[i] += (X[i] - X_hat[i]) * t_inv2 * mass; // momentum of each vertex
		}
		//Spring Force.
		for(int i=0;i<L.Length;i++){
			int v0 = E[i*2+0];
			int v1 = E[i*2+1]; // get vertex index
			Vector3 d = X[v0]-X[v1]; // get the current distance of spring
			G[v0] += spring_k * (1-L[i]/d.magnitude) * d; // add spring force to vertex 0
			G[v1] -= spring_k * (1-L[i]/d.magnitude) * d; // add spring force to vertex 1
		}
	}

    // Update is called once per frame
	void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X 		= mesh.vertices; // current position
		Vector3[] X_hat 	= new Vector3[X.Length]; // predicted position
		Vector3[] last_X 	= new Vector3[X.Length]; // last position, used for chebychev accelerate of non-linear solvinga iteration.
		Vector3[] G 		= new Vector3[X.Length];

		//Initial Setup
		for(int i=0; i<X.Length; i++){
			V[i] *= damping; // apply dumping to velocity
			X_hat[i] = X[i] + V[i] * t; // this is the initial guessed position, update using old velocity
			// which is not implicit update which use future velocity
			last_X[i] = X[i] = X_hat[i]; // set x to initial guess, which i
		}

		// Iterative Solve
		float t_inv2 = 1.0f / (t * t);
		float omega = 1;
		for(int k=0; k<iteration_num; k++)
		{
			Get_Gradient(X, X_hat, t_inv2, G);
			if (k == 1) omega = 1/(1 - 0.5f*rho*rho);
			if (k > 1) omega = 1/(1 - 0.25f*rho*rho*omega);
			//Update X by gradient, because hessian matrix is hard to calculate, so use diagonal matrix as hessian.
			for(int i=0; i<X.Length; i++){
				// for the two fix points, we don't update them
				if(i == 0 || i == n-1) continue;
				// the bottom fix points
				// if(i == (n-1)*n || i == n*n-1) continue;
				Vector3 old_X = X[i];
				X[i] -= (1/(t_inv2*mass + 4*spring_k))*G[i]; // update position(magic formula)
				X[i] = X[i] * omega + last_X[i] * (1-omega); // chebychev accelerate
			}
			last_X = X;
		}
		//Finishing.
		for(int i=0; i<X.Length; i++){
			V[i] += (X[i] - X_hat[i]) / t; // update velocity
		}
		mesh.vertices = X;

		Collision_Handling ();
		mesh.RecalculateNormals ();
	}
}
