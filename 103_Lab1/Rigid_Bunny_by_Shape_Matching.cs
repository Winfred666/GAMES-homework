using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Rigid_Bunny_by_Shape_Matching : MonoBehaviour
{
	float dt = 0.015f;
	public bool launched = false;
	Vector3[] X; // X is the current mesh vertices position.
	Vector3[] Q; // Q is the original mesh vertices position, centerized at the origin, so it is r_i
	Vector3[] V; // V is the velocity of each particle.
	Matrix4x4 QQt = Matrix4x4.zero;

	[Range(0.0f, 3.0f)]
	public float restitution 	= 2f;					// for collision

	[Range(0.0f, 1.0f)]
	public float friction 	= 0.4f;					// for collision

	[Range(0.0f, 0.2f)]
	public float predict_dist = 0.09f;					// for collision

    // Start is called before the first frame update
    void Start()
    {
    	Mesh mesh = GetComponent<MeshFilter>().mesh;
        V = new Vector3[mesh.vertices.Length];
        X = mesh.vertices;
        Q = mesh.vertices;

        //Centerizing Q.
        Vector3 c=Vector3.zero;
        for(int i=0; i<Q.Length; i++)
        	c+=Q[i];
        c/=Q.Length;
        for(int i=0; i<Q.Length; i++)
        	Q[i]-=c;

        //Get QQ^t ready.
		for(int i=0; i<Q.Length; i++)
		{
			QQt[0, 0]+=Q[i][0]*Q[i][0];
			QQt[0, 1]+=Q[i][0]*Q[i][1];
			QQt[0, 2]+=Q[i][0]*Q[i][2];
			QQt[1, 0]+=Q[i][1]*Q[i][0];
			QQt[1, 1]+=Q[i][1]*Q[i][1];
			QQt[1, 2]+=Q[i][1]*Q[i][2];
			QQt[2, 0]+=Q[i][2]*Q[i][0];
			QQt[2, 1]+=Q[i][2]*Q[i][1];
			QQt[2, 2]+=Q[i][2]*Q[i][2];
		}
		QQt[3, 3]=1;

		for(int i=0; i<X.Length; i++)
			V[i][0]=4.0f;

		Update_Mesh(transform.position, Matrix4x4.Rotate(transform.rotation), 0);
		transform.position=Vector3.zero;
		transform.rotation=Quaternion.identity;
   	}

	Matrix4x4 AddMatrix(Matrix4x4 a, Matrix4x4 b)
    {
        Matrix4x4 result = new Matrix4x4();

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                result[i, j] = a[i, j] + b[i, j];
            }
        }
        return result;
    }
	Matrix4x4 OuterProduct(Vector3 A, Vector3 B){
		// Create each column of the matrix
		Vector4 column1 = new Vector4(A.x * B.x, A.y * B.x, A.z * B.x, 0);
		Vector4 column2 = new Vector4(A.x * B.y, A.y * B.y, A.z * B.y, 0);
		Vector4 column3 = new Vector4(A.x * B.z, A.y * B.z, A.z * B.z, 0);
		Vector4 column4 = new Vector4(0, 0, 0, 1);

		// Construct the matrix using column vectors
		Matrix4x4 result = new Matrix4x4(column1, column2, column3, column4);

		return result;
	}

   	// Polar Decomposition that returns the rotation from F.
   	Matrix4x4 Get_Rotation(Matrix4x4 F)
	{
		Matrix4x4 C = Matrix4x4.zero;
	    for(int ii=0; ii<3; ii++)
	    for(int jj=0; jj<3; jj++)
	    for(int kk=0; kk<3; kk++)
	        C[ii,jj]+=F[kk,ii]*F[kk,jj];
	   
	   	Matrix4x4 C2 = Matrix4x4.zero;
		for(int ii=0; ii<3; ii++)
	    for(int jj=0; jj<3; jj++)
	    for(int kk=0; kk<3; kk++)
	        C2[ii,jj]+=C[ii,kk]*C[jj,kk];
	    
	    float det    =  F[0,0]*F[1,1]*F[2,2]+
	                    F[0,1]*F[1,2]*F[2,0]+
	                    F[1,0]*F[2,1]*F[0,2]-
	                    F[0,2]*F[1,1]*F[2,0]-
	                    F[0,1]*F[1,0]*F[2,2]-
	                    F[0,0]*F[1,2]*F[2,1];
	    
	    float I_c    =   C[0,0]+C[1,1]+C[2,2];
	    float I_c2   =   I_c*I_c;
	    float II_c   =   0.5f*(I_c2-C2[0,0]-C2[1,1]-C2[2,2]);
	    float III_c  =   det*det;
	    float k      =   I_c2-3*II_c;
	    
	    Matrix4x4 inv_U = Matrix4x4.zero;
	    if(k<1e-10f)
	    {
	        float inv_lambda=1/Mathf.Sqrt(I_c/3);
	        inv_U[0,0]=inv_lambda;
	        inv_U[1,1]=inv_lambda;
	        inv_U[2,2]=inv_lambda;
	    }
	    else
	    {
	        float l = I_c*(I_c*I_c-4.5f*II_c)+13.5f*III_c;
	        float k_root = Mathf.Sqrt(k);
	        float value=l/(k*k_root);
	        if(value<-1.0f) value=-1.0f;
	        if(value> 1.0f) value= 1.0f;
	        float phi = Mathf.Acos(value);
	        float lambda2=(I_c+2*k_root*Mathf.Cos(phi/3))/3.0f;
	        float lambda=Mathf.Sqrt(lambda2);
	        
	        float III_u = Mathf.Sqrt(III_c);
	        if(det<0)   III_u=-III_u;
	        float I_u = lambda + Mathf.Sqrt(-lambda2 + I_c + 2*III_u/lambda);
	        float II_u=(I_u*I_u-I_c)*0.5f;
	        
	        
	        float inv_rate, factor;
	        inv_rate=1/(I_u*II_u-III_u);
	        factor=I_u*III_u*inv_rate;
	        
	       	Matrix4x4 U = Matrix4x4.zero;
			U[0,0]=factor;
	        U[1,1]=factor;
	        U[2,2]=factor;
	        
	        factor=(I_u*I_u-II_u)*inv_rate;
	        for(int i=0; i<3; i++)
	        for(int j=0; j<3; j++)
	            U[i,j]+=factor*C[i,j]-inv_rate*C2[i,j];
	        
	        inv_rate=1/III_u;
	        factor=II_u*inv_rate;
	        inv_U[0,0]=factor;
	        inv_U[1,1]=factor;
	        inv_U[2,2]=factor;
	        
	        factor=-I_u*inv_rate;
	        for(int i=0; i<3; i++)
	        for(int j=0; j<3; j++)
	            inv_U[i,j]+=factor*U[i,j]+inv_rate*C[i,j];
	    }
	    
	    Matrix4x4 R=Matrix4x4.zero;
	    for(int ii=0; ii<3; ii++)
	    for(int jj=0; jj<3; jj++)
	    for(int kk=0; kk<3; kk++)
	        R[ii,jj]+=F[ii,kk]*inv_U[kk,jj];
	    R[3,3]=1;
	    return R;
	}

	// Update the mesh vertices according to translation c and rotation R.
	// It also updates the velocity.
	void Update_Mesh(Vector3 c, Matrix4x4 R, float inv_dt)
   	{
   		for(int i=0; i<Q.Length; i++)
		{
			Vector3 x=(Vector3)(R*Q[i])+c;

			V[i]+=(x-X[i])*inv_dt;
			X[i]=x;
		}	
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		mesh.vertices=X;
   	}

	// Collision handling.
	void Collision(float inv_dt, Vector3 P, Vector3 N)
	{
		// check every particle, if it is inside the plane, project it out.
		for(int i=0; i<V.Length; i++){
			float dist = Vector3.Dot(X[i]-P, N) - predict_dist;
			if(dist < 0 && Vector3.Dot(V[i], N) < 0){
				Vector3 v_n =  N*Vector3.Dot(V[i], N);
				Vector3 v_t = V[i] - v_n;
				// use impulse to update the velocity
				X[i] = X[i] - N * dist;

				float a = 0;
				if(v_t.magnitude > 0.001) a = 1 - friction*(1 + restitution)*v_n.magnitude/v_t.magnitude;
				if(a < 0) a = 0;

				V[i] = v_t * a - v_n * restitution;
			}
		}
	}

    // Update is called once per frame
    void Update()
    {
		//Game Control
		if(Input.GetKey("r"))
		{
			for(int i=0; i<X.Length; i++){
				V[i]=new Vector3 (0, 0, 0);
				X[i]=Q[i] + new Vector3(0, 0.6f, 0);
			}
			Mesh mesh = GetComponent<MeshFilter>().mesh;
			mesh.vertices=X;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			for(int i=0; i<X.Length; i++)
				V[i]=new Vector3 (5, 2, 0);
			launched=true;
		}
		if(!launched){
			return;
		}

  		//Step 1: run a simple particle system.
        for(int i=0; i<V.Length; i++)
        {
			V[i]*=0.99f; // damping
			X[i]+=V[i]*dt; // simple Euler integration
        }

        //Step 2: Perform simple particle collision.
		for(int i=0; i<V.Length; i++)
			V[i]+=new Vector3(0, -9.8f, 0)*dt; // gravity, this should do before collision position correction
		
		Collision(1/dt,new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
		Collision(1/dt,new Vector3(2, 0, 0), new Vector3(-1, 0, 0));
		
		// Step 3: Use shape matching to get new translation c and 
		// new rotation R. Update the mesh by c and R.
        //Shape Matching (translation)
		Vector3 c=Vector3.zero;
		for(int i=0; i<X.Length; i++) c+=X[i];
		c/=X.Length;
		//Shape Matching (rotation)
		Matrix4x4 y_c_r = Matrix4x4.zero;
		Matrix4x4 r_rT = Matrix4x4.zero;
		for(int i=0; i<X.Length; i++){
			y_c_r = AddMatrix(y_c_r, OuterProduct(X[i]-c, Q[i]) );
			r_rT = AddMatrix(r_rT, OuterProduct(Q[i], Q[i]));
		}
		Matrix4x4 A = y_c_r * r_rT.inverse;
		Matrix4x4 R = Get_Rotation(A);
		Update_Mesh(c, R, 1/dt);
    }
}
