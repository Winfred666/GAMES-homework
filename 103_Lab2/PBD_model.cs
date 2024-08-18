using UnityEngine;
using System.Collections;

public class PBD_model: MonoBehaviour {

	float 		t= 0.0333f;
	float		damping= 0.99f;
	int[] 		E;
	float[] 	L;
	Vector3[] 	V;

	[SerializeField]
	public int iteration_num = 32;

	[SerializeField]
	public float shpere_radius = 2.7f;

	// Use this for initialization
	void Start () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;

		//Resize the mesh.
		int n=21;
		Vector3[] X  	= new Vector3[n*n];
		Vector2[] UV 	= new Vector2[n*n];
		int[] T	= new int[(n-1)*(n-1)*6];
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
			T[t*6+0]=j*n+i;
			T[t*6+1]=j*n+i+1;
			T[t*6+2]=(j+1)*n+i+1;
			T[t*6+3]=j*n+i;
			T[t*6+4]=(j+1)*n+i+1;
			T[t*6+5]=(j+1)*n+i;
			t++;
		}
		mesh.vertices	= X;
		mesh.triangles	= T;
		mesh.uv 		= UV;
		mesh.RecalculateNormals ();

		//Construct the original edge list
		int[] _E = new int[T.Length*2];
		for (int i=0; i<T.Length; i+=3) 
		{
			_E[i*2+0]=T[i+0];
			_E[i*2+1]=T[i+1];
			_E[i*2+2]=T[i+1];
			_E[i*2+3]=T[i+2];
			_E[i*2+4]=T[i+2];
			_E[i*2+5]=T[i+0];
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
				E[e*2+0]=_E [i + 0];
				E[e*2+1]=_E [i + 1];
				e++;
			}

		L = new float[E.Length/2];
		for (int e=0; e<E.Length/2; e++) 
		{
			int i = E[e*2+0];
			int j = E[e*2+1];
			L[e]=(X[i]-X[j]).magnitude;
		}

		V = new Vector3[X.Length];
		for (int i=0; i<X.Length; i++)
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

	bool Is_Fixed(int i)
	{
		return (i==0 || i==20);
	}

	void Strain_Limiting()
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] vertices = mesh.vertices;

		//Apply PBD here.
		Vector3[] sum_x = new Vector3[vertices.Length]; // store the sum of every x_i update
		int[] sum_num = new int[vertices.Length]; // store the number of spring x_i connected
		for(int i=0; i<vertices.Length; i++){
			sum_x[i] = new Vector3(0, 0, 0); // initialize sum_x
			sum_num[i] = 0; // initialize sum_num
		}
		for(int i=0; i<L.Length; i++){
			int v0 = E[i*2+0];
			int v1 = E[i*2+1];
			Vector3 delta_p = vertices[v0] - vertices[v1];// the vector from v1 to v0
			// update the position(every new value is a position,take average)
			// of v0 and v1 to make them satisfy the constraint
			// print(vertices[v0] + " " + vertices[v1] + " " + L[i] + " " + delta_p.normalized);
			sum_x[v0] += 1.0f/2*(vertices[v0] + vertices[v1] + L[i]*(delta_p.normalized));
			sum_x[v1] += 1.0f/2*(vertices[v0] + vertices[v1] - L[i]*(delta_p.normalized));

			sum_num[v0]++;
			sum_num[v1]++;
		}
		for(int i=0; i<vertices.Length; i++){
			if(Is_Fixed(i))	continue; // fix top left and right vertices
			if(sum_num[i] != 0){
				Vector3 old_X = vertices[i];
				// print(sum_num[i]+" "+sum_x[i]);
				vertices[i] = (0.2f*old_X + sum_x[i])/(0.2f + sum_num[i]);
				V[i] += (1/t)*(vertices[i] - old_X); // here 0.2f is the damping factor, make update step smaller.
			}
		}
		mesh.vertices = vertices;
	}

	void Collision_Handling()
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;
		
		//For every vertex, detect collision and apply impulse if needed.
		GameObject sphere = GameObject.Find("Sphere");
		for(int i=0; i<X.Length; i++){
			if(Is_Fixed(i))	continue; // fix top left and right vertices
			// calculate the distance to the shpere center
			Vector3 dir_i = (X[i] - sphere.transform.position);
			float d = dir_i.magnitude;
			if(d <= shpere_radius){ // if the vertex is inside the sphere, correct V and X to make it outside the sphere
				dir_i.Normalize();
				V[i] += 1/t*(sphere.transform.position + shpere_radius*dir_i - X[i]);
				X[i] = sphere.transform.position + shpere_radius*dir_i;
			}
		}
		mesh.vertices = X;
	}

	// Update is called once per frame
	void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;

		for(int i=0; i<X.Length; i++)
		{
			if(Is_Fixed(i))	continue; // fix top left and right vertices
			//Initial Setup
			//damp velocity, update v
			V[i] += new Vector3(0, -9.8f, 0) * t; // particle mass = 1
			V[i] *= damping;
			X[i] += V[i] * t;
		}
		mesh.vertices = X;

		for(int l=0; l<iteration_num; l++)
			Strain_Limiting ();

		Collision_Handling ();

		mesh.RecalculateNormals ();

	}


}

