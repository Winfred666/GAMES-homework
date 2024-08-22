using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

public class FVM : MonoBehaviour
{
	float dt 			= 0.003f;
    float mass 			= 1;
	float stiffness_0	= 20000.0f;
    float stiffness_1 	= 5000.0f;
    float damp			= 0.999f;

	int[] 		Tet; // store the tetrahedra, each element is the index of a vertex.
	int tet_number;			//The number of tetrahedra, which is the number of faces in the model.

	Vector3[] 	Force;
	Vector3[] 	V;
	Vector3[] 	X;			// store the current vertices position of the model.
	int number;				//The number of vertices

	Matrix4x4[] inv_Dm;		// constant, store the inv of original edge matrix of the tetrahedra.

	//For Laplacian smoothing.
	Vector3[]   V_sum; // store the sum of vertices that a vertex is connected to.
	int[]		V_num; // store the number of vertices that a vertex is connected to.

	SVD svd = new SVD();

	[Range(0.0f, 3.0f)]
	public float restitution 	= 1f;					// for collision
	[Range(0.0f, 1.0f)]
	public float friction 	= 0.4f;					// for collision

	[Range(0.0f, 3f)]
	public float predict_dist = 0.9f;					// for collision

	[Range(0.0f, 1.0f)]
	public float laplace_smooth = 0.1f;					// for laplacian smoothing

    // Start is called before the first frame update
    void Start()
    {
    	// FILO IO: Read the house model from files.
    	// The model is from Jonathan Schewchuk's Stellar lib.
    	{
			// set the index of the vertices of the tetrahedra.
    		string fileContent = File.ReadAllText("Assets/bounce/house2.ele");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		
    		tet_number=int.Parse(Strings[0]);
        	Tet = new int[tet_number*4];

    		for(int tet=0; tet<tet_number; tet++)
    		{
				Tet[tet*4+0]=int.Parse(Strings[tet*5+4])-1;
				Tet[tet*4+1]=int.Parse(Strings[tet*5+5])-1;
				Tet[tet*4+2]=int.Parse(Strings[tet*5+6])-1;
				Tet[tet*4+3]=int.Parse(Strings[tet*5+7])-1;
			}
    	}

    	{
			// fold the flat mesh into a 3D model
			string fileContent = File.ReadAllText("Assets/bounce/house2.node");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		number = int.Parse(Strings[0]);
    		X = new Vector3[number];
			// set the original position of the vertices the house model
       		for(int i=0; i<number; i++)
       		{
       			X[i].x=float.Parse(Strings[i*5+5])*0.4f;
       			X[i].y=float.Parse(Strings[i*5+6])*0.4f;
       			X[i].z=float.Parse(Strings[i*5+7])*0.4f;
       		}
    		//Centralize the model.
	    	Vector3 center=Vector3.zero;
	    	for(int i=0; i<number; i++)		center+=X[i];
	    	center=center/number;
	    	for(int i=0; i<number; i++)
	    	{
	    		X[i]-=center;
	    		float temp=X[i].y; // because unity is left handed.
	    		X[i].y=X[i].z;
	    		X[i].z=temp;
	    	}
		}
        
		// {
		// 	tet_number=1;
		// 	// this create one tetrahedron, for testing.
		// 	Tet = new int[tet_number*4];
		// 	Tet[0]=0;
		// 	Tet[1]=1;
		// 	Tet[2]=2;
		// 	Tet[3]=3;

		// 	number=4;
		// 	X = new Vector3[number];
		// 	V = new Vector3[number];
		// 	Force = new Vector3[number];
		// 	X[0]= new Vector3(0, 0, 0);
		// 	X[1]= new Vector3(2, 0, 0);
		// 	X[2]= new Vector3(0, 3, 0);
		// 	X[3]= new Vector3(0, 0, 2);
		// }

        //Create triangle mesh according to the tetrahedron set

       	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];

        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }

        int[] triangles = new int[tet_number*12];
        for(int t=0; t<tet_number*4; t++)
        {
        	triangles[t*3+0]=t*3+0;
        	triangles[t*3+1]=t*3+1;
        	triangles[t*3+2]=t*3+2;
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.triangles = triangles;
		mesh.RecalculateNormals ();

		V 	  = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];
		//set V and F to zero.
		for(int i=0; i<number; i++)
		{
			V[i]=Vector3.zero;
			Force[i]=Vector3.zero;
			V_sum[i] = Vector3.zero;
			V_num[i] = 0;
		}

		//TODO: Need to allocate and assign inv_Dm,
		// remember only use first 3x3 matrix of the 4x4 matrix.
		inv_Dm = new Matrix4x4[tet_number];
		for(int tet=0; tet<tet_number; tet++){
			inv_Dm[tet] = Build_Edge_Matrix(tet).inverse;
		}
    }

	// use p0 as the end point
    Matrix4x4 Build_Edge_Matrix(int tet)
    {
    	//TODO: Need to build edge matrix here.
    	Matrix4x4 ret=Matrix4x4.zero;
		//get tetrahedron vertices
		Vector3 p0 = X[Tet[tet*4+0]];
		Vector3 p1 = X[Tet[tet*4+1]];
		Vector3 p2 = X[Tet[tet*4+2]];
		Vector3 p3 = X[Tet[tet*4+3]];
		ret.SetColumn(0, p1-p0);
		ret.SetColumn(1, p2-p0);
		ret.SetColumn(2, p3-p0);
		ret.SetColumn(3, new Vector4(0, 0, 0, 1)); // the last column is not used.
		return ret;
    }

	Matrix4x4 AddMatrix(Matrix4x4 a, Matrix4x4 b){
        Matrix4x4 result = Matrix4x4.zero;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                result[i, j] = a[i, j] + b[i, j];
            }
        }
		result[3,3] = 1; // the last element is not used.
        return result;
    }

	Matrix4x4 MultiplyMatrixByScalar(Matrix4x4 a, float b){
        Matrix4x4 result = Matrix4x4.zero;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                result[i, j] = b * a[i, j];
            }
        }
		result[3,3] = 1; // the last element is not used.
        return result;
    }


    void _Update()
    {
    	// Jump up.
		if(Input.GetKeyDown(KeyCode.Space))
    	{
    		for(int i=0; i<number; i++)
    			V[i].y+=0.5f;
    	}

    	for(int i=0 ;i<number; i++)
    	{
    		//TODO: Add gravity to Force.
			Force[i] = new Vector3(0, -9.8f, 0) * mass;
    	}

    	for(int tet=0; tet<tet_number; tet++)
    	{
    		//TODO: Deformation Gradient, F
			// first get the original edge matrix
			Matrix4x4 F = Build_Edge_Matrix(tet) * inv_Dm[tet];
    		//TODO: Green Strain
			Matrix4x4 G = MultiplyMatrixByScalar(
				AddMatrix(F.transpose * F, MultiplyMatrixByScalar(Matrix4x4.identity,-1f)),
				0.5f);
			
    		//TODO: Second PK Stress
			// for different materials, we have different energy density W, and its gradient-to-G S.
			
			Matrix4x4 S = AddMatrix(
			MultiplyMatrixByScalar(G,2.0f*stiffness_1),
			MultiplyMatrixByScalar(Matrix4x4.identity , stiffness_0*(G[0,0] + G[1,1] + G[2,2]))); // remember that only the first 3x3 matrix is used.
			Matrix4x4 P = F * S;
			
			// the above process of P calculation can be replaced using SVD

			// Matrix4x4 U=Matrix4x4.identity, Scale=Matrix4x4.zero, V=Matrix4x4.identity;
			// svd.svd(F, ref U, ref Scale, ref V);
			// //1. linear isotropic elasticity(StVK is a special case)
			// float I_c = stiffness_0*((Scale[0,0]*Scale[0,0] + Scale[1,1]*Scale[1,1] + Scale[2,2]*Scale[2,2])- 3)/2;
			// Matrix4x4 P_lambda = Matrix4x4.Scale(new Vector3(
			// 	I_c*Scale[0,0] + stiffness_1*(Scale[0,0]*Scale[0,0]*Scale[0,0] - Scale[0,0]),
			// 	I_c*Scale[1,1] + stiffness_1*(Scale[1,1]*Scale[1,1]*Scale[1,1] - Scale[1,1]),
			// 	I_c*Scale[2,2] + stiffness_1*(Scale[2,2]*Scale[2,2]*Scale[2,2] - Scale[2,2])
			// ));
			// Matrix4x4 P = U * P_lambda * V.transpose;

			
    		//TODO: Elastic Force
			Matrix4x4 F_123 = MultiplyMatrixByScalar(P * inv_Dm[tet].transpose, 
			- 1.0f / (6.0f * inv_Dm[tet].determinant));

			// set force to the vertices(Force is the total force, and here only focus on deformative force.)
			Vector3 f_1 = new Vector3(F_123[0, 0], F_123[1, 0], F_123[2, 0]); // the first column of F_123
			Vector3 f_2 = new Vector3(F_123[0, 1], F_123[1, 1], F_123[2, 1]); // the second column of F_123
			Vector3 f_3 = new Vector3(F_123[0, 2], F_123[1, 2], F_123[2, 2]); // the third column of F_123
			// for linear deformation, the force on the last vertex is the negative sum of the other three vertices.
			Force[Tet[tet*4+1]] += f_1;
			Force[Tet[tet*4+2]] += f_2;
			Force[Tet[tet*4+3]] += f_3;
			Force[Tet[tet*4+0]] += -f_1 - f_2 - f_3;
			// print(Force[Tet[tet*4+0]] + " " + Force[Tet[tet*4+1]] + " " + Force[Tet[tet*4+2]] + " " + Force[Tet[tet*4+3]]);
    	}


		for(int tet=0; tet<tet_number; tet++){
			// calculate for Laplacian smoothing.
			int i0=Tet[tet*4+0], i1=Tet[tet*4+1], i2=Tet[tet*4+2], i3=Tet[tet*4+3];
			V_sum[i0] += V[i1] + V[i2] + V[i3];
			V_num[i0] += 3;
			V_sum[i1] += V[i0] + V[i2] + V[i3];
			V_num[i1] += 3;
			V_sum[i2] += V[i0] + V[i1] + V[i3];
			V_num[i2] += 3;
			V_sum[i3] += V[i0] + V[i1] + V[i2];
			V_num[i3] += 3;
		}

		// for simplicity, we use explicit Euler integration.
		for(int i=0; i<number; i++){
			// smooth velocity(before apply force, or force will be inaccurate.)
			// print(V[i] + " " + V_sum[i] + " " + V_num[i]);
			if(V_num[i]>0)
				V[i] = V[i]*(1.0f - laplace_smooth) + (V_sum[i]/V_num[i]) * laplace_smooth;
			V_sum[i] = Vector3.zero;
			V_num[i] = 0;

			//TODO: Update X and V here.
			V[i]+=Force[i]/mass*dt;
			V[i]*=damp;
			X[i]+=V[i]*dt;

			// print(V[i]);

			//TODO: (Particle) collision with floor,
			// just like what we do in shape matching.
			float Ydist = (X[i].y + 3) + predict_dist;
			if(Ydist < 0){
				Vector3 N = new Vector3(0, 1, 0); // normal vector of the floor.
				Vector3 v_n =  N*Vector3.Dot(V[i], N); // normal velocity
				Vector3 v_t = V[i] - v_n; // tangential velocity
				// use impulse to update the velocity
				X[i] = X[i] - N * Ydist;
				float a = 0; // calculate the friction of the collision
				if(v_t.magnitude > 0.001) a = 1 - friction*(1 + restitution)*v_n.magnitude/v_t.magnitude;
				if(a < 0) a = 0;
				V[i] = v_t * a - v_n * restitution;
			}
    	}
    }

    // Update is called once per frame
    void Update()
    {
    	for(int l=0; l<10; l++)
    		 _Update();

    	// Dump the vertex array for rendering.
    	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.RecalculateNormals ();
    }

}
