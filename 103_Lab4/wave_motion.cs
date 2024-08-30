using UnityEngine;
using System.Collections;

public class wave_motion : MonoBehaviour 
{
	int size 		= 100; // size is the resolution of the water.
	
	[SerializeField]
	float  rate 	= 0.008f; // rate is the viscosity of the water, to smooth wave

	[SerializeField]
	bool boundary_connected = true;

	[SerializeField]
	float gamma		= 0.008f; // gamma is used to stablize one-way coupling with virtual height, because it is explicit integration.

	[SerializeField]
	float damping 	= 0.997f; // damping is used to diminish the virtual height of the water.

	[SerializeField]
	float manual_wave_height = 0.4f;

	[SerializeField]
	float density = 1f;

	GameObject [] cubes; // blocks stores the rigid bodies in the scene.
	Cube_rigid_body [] cube_rigids; // cubes stores the floating objects in the scene.

	float[,] 	old_h; // old_h is used to store the height of the water at the previous time step.
	float[,]	low_h; // low_h is used to store the height of the water at the current time step.
	float[,]	vh; // vh, store the virtual height of the water (add to pressure term to create interact effect).
	float[,]	b;  // b is used to store the right hand side of the Poisson equation.

	bool [,]	cg_mask; // cg_mask help to solve poisson equation using matrix,
	// because water with no virtual height (no interaction with rigid body) should not be considered.
	float[,]	cg_p;
	float[,]	cg_r;
	float[,]	cg_Ap;
	// bool 	tag=true; // tag is used to switch between block 1 and block 2.

	Vector3 	cube_v = Vector3.zero;
	Vector3 	cube_w = Vector3.zero;


	// Use this for initialization
	void Start () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.Clear ();

		Vector3[] X=new Vector3[size*size]; // X stores the vertex positions of the water.

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			X[i*size+j].x=i*0.1f-size*0.05f;
			X[i*size+j].y=0; // set the water height to 0.
			X[i*size+j].z=j*0.1f-size*0.05f;
		}

		int[] T = new int[(size - 1) * (size - 1) * 6]; // 6 indices per triangle
		int index = 0;
		for (int i=0; i<size-1; i++)
		for (int j=0; j<size-1; j++)
		{
			T[index*6+0]=(i+0)*size+(j+0);
			T[index*6+1]=(i+0)*size+(j+1);
			T[index*6+2]=(i+1)*size+(j+1);
			T[index*6+3]=(i+0)*size+(j+0);
			T[index*6+4]=(i+1)*size+(j+1);
			T[index*6+5]=(i+1)*size+(j+0);
			index++;
		}
		mesh.vertices  = X;
		mesh.triangles = T;
		mesh.RecalculateNormals ();

		low_h 	= new float[size,size];
		old_h 	= new float[size,size];
		vh 	  	= new float[size,size];
		b 	  	= new float[size,size];

		cg_mask	= new bool [size,size];
		cg_p 	= new float[size,size];
		cg_r 	= new float[size,size];
		cg_Ap 	= new float[size,size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			low_h[i,j]=99999;
			old_h[i,j]=0;
			vh[i,j]=0;
		}

		cubes = new GameObject[2];
		cube_rigids = new Cube_rigid_body[2];
		cubes[0] = GameObject.Find("Block");
		cube_rigids[0] = cubes[0].GetComponent<Cube_rigid_body>();
		cubes[1] = GameObject.Find("Cube");
		cube_rigids[1] = cubes[1].GetComponent<Cube_rigid_body>();
	}

	// calculate minus of 4 direct laplace smoother (act like a low pass filter)
	void A_Times(float[,] x, float[,] Ax, int li, int ui, int lj, int uj, bool[,] mask = null)
	{
		for(int i=li; i<=ui; i++){ // ui is the upper bound of i
			for(int j=lj; j<=uj; j++){
				if(i>=0 && j>=0 && i<size && j<size && (mask==null || mask[i,j]))
				{
					Ax[i,j]=0; // safely set the value to 0.
					if(boundary_connected){
						// usen neumann boundary condition, so there are no pressure gradient at the boundary.
						if(i!=0)		Ax[i,j]-=x[i-1,j]-x[i,j];
						if(i!=size-1)	Ax[i,j]-=x[i+1,j]-x[i,j];
						if(j!=0)		Ax[i,j]-=x[i,j-1]-x[i,j];
						if(j!=size-1)	Ax[i,j]-=x[i,j+1]-x[i,j];
					}else{
						// connect the pressure to another side.
						Ax[i,j] -= x[(i-1+size)%size,j] + x[(i+1)%size,j]
							+x[i,(j-1+size)%size] + x[i,(j+1)%size] - 4*x[i,j];
					}
				}
			}
		}
	}

	// calculate the dot product of two matrix.
	float Dot(bool[,] mask, float[,] x, float[,] y, int li, int ui, int lj, int uj)
	{
		float ret=0;
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			ret+=x[i,j]*y[i,j];
		}
		return ret;
	}
	
	// calculate the Poisson equation, one-way coupling with the water.
	void Conjugate_Gradient(bool[,] mask, float[,] b, float[,] x, int li, int ui, int lj, int uj)
	{
		//calculate the Laplacian smoother by in a cg_r, which is current A*x (x is initial guess)
		A_Times(x, cg_r, li, ui, lj, uj,mask);

		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			// now cg_p is the residual of the Poisson equation, b-A*x
			cg_p[i,j]=cg_r[i,j]=b[i,j]-cg_r[i,j];
		}

		float rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj); // calculate the norm of the residual.

		for(int k=0; k<128; k++) // using 128 iteration to solve the Poisson equation using (conjugate gradient).
		{
			if(rk_norm<1e-10f)	break;
			A_Times(cg_p, cg_Ap, li, ui, lj, uj,mask); // now cg_Ap is the A*p = A*(b-A*x)
			// calculate the step size of the conjugate gradient, which is |b-A*x|/|A*(b-A*x)|
			float alpha=rk_norm/Dot(mask, cg_p, cg_Ap, li, ui, lj, uj);

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				// update the x using the step size and residual.
				x[i,j]   +=alpha*cg_p[i,j];
				// update the residual using the step size and A*p.
				cg_r[i,j]-=alpha*cg_Ap[i,j];
			}

			// calculate the new norm of the residual.
			float _rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);
			// calculate the beta, which is the step size of the residual.
			float beta=_rk_norm/rk_norm;
			rk_norm=_rk_norm;

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				cg_p[i,j]=cg_r[i,j]+beta*cg_p[i,j]; // update the residual.
			}
		}
	}

	// calculate collision detection between AABB and ray, if no collision, return null.
	// return: entry point, is_collide, exit point, is_positive_collide
	(Vector3,bool,Vector3,bool) AABB_Collide(Vector3 min_pos, Vector3 max_pos, Vector3 ori, Vector3 dir){
		for (int i = 0; i < 3; i++){
			if (dir[i] < 0){
				float temp = min_pos[i];
				min_pos[i] = max_pos[i];
				max_pos[i] = temp;
			}
		}
		Vector3 tmin = new Vector3();
    	Vector3 tmax = new Vector3();
		// Calculate tmin and tmax for each axis, handling the case where dir[i] == 0
		for (int i = 0; i < 3; i++){
			if (dir[i] != 0){ // normal case
				tmin[i] = (min_pos[i] - ori[i]) / dir[i];
				tmax[i] = (max_pos[i] - ori[i]) / dir[i];
			}else{
				if(ori[i] < min_pos[i] || ori[i] > max_pos[i]){ // Ray is parallel and outside the slab
					// Ray is parallel and outside the slab
					return (Vector3.zero, false, Vector3.zero,false);
				} else { // just compared to position.
					// Ray is parallel and inside the slab; set to extreme values that will never be reached
					tmin[i] = float.NegativeInfinity;
					tmax[i] = float.PositiveInfinity;
				}
			}
		}
		float t_enter = Mathf.Max(Mathf.Max(tmin.x, tmin.y), tmin.z);
		float t_exit = Mathf.Min(Mathf.Min(tmax.x, tmax.y), tmax.z);
		if(t_enter < t_exit){
			// if the box is sinking, still need to consider the exit point.
			return (ori + t_enter*dir, true, ori + t_exit*dir, t_exit > 0);
		}
		return (Vector3.zero, false, Vector3.zero, false);
	}

	float getXZ(int index){
		return (index*0.1f-size*0.05f);
	}

	// calculate b = (new_h - low_h)/rate for a block, b should set to zero at first.
	void calc_cube_b(GameObject block , float[,] b, bool[,] mask, float[,] new_h, float[,] exit_h, float[,] enter_h){
		Bounds aabbWorld = block.GetComponent<Renderer>().bounds;
		// AABB in world space
		Vector3 worldMin = aabbWorld.min;
		Vector3 worldMax = aabbWorld.max;
		int min_i = (int)((worldMin.x+size*0.05f)/0.1f);
		if(min_i < 0) min_i = 0;
		int max_i = (int)((worldMax.x+size*0.05f)/0.1f) + 1;
		if(max_i >= size) max_i = size-1;
		int min_j = (int)((worldMin.z+size*0.05f)/0.1f);
		if(min_j < 0) min_j = 0;
		int max_j = (int)((worldMax.z+size*0.05f)/0.1f) + 1;
		if(max_j >= size) max_j = size-1;
		
		// Vector3 cube_pos = block.transform.position;  // in local frame, the position is (0,0,0)
		Vector3 cube_scale = block.transform.localScale;
		Vector3 min_pos = - cube_scale/2;
		Vector3 max_pos = cube_scale/2;
		
		// project the cube orthographically onto y=0 plane.
		// but for convenience, for every ray shooting along y axis, we can first transform the ray to the cube's local space.
		Vector3 dir = new Vector3(0,1,0);
		dir = block.transform.InverseTransformDirection(dir);

		// then we can calculate the intersection point with the cube.
		for(int i=min_i; i<=max_i; i++){
			for(int j=min_j; j<max_j; j++){
				// for column at (i,j), the position is (i*0.1f-size*0.05f,0,j*0.1f-size*0.05f)
				Vector3 origin = new Vector3(getXZ(i), 0, getXZ(j));
				origin = block.transform.InverseTransformPoint(origin);
				
				// now we can calculate the intersection point with the cube(now an AABB).
				(Vector3, bool, Vector3,bool) ret = AABB_Collide(min_pos, max_pos, origin, dir);
				// print(origin + " " + min_pos + " " + max_pos);
				if(!ret.Item2) continue;
				// finally, we can transform the entry intersection point back to the world space.
				float enter_y = block.transform.TransformPoint(ret.Item1).y;
				// now we can calculate the height of the water at the intersection point.
				// add pressure term, only if water is higher than the contact point.
				if(enter_y < new_h[i,j]){
					// print(i + " " + j + " "  + b[i,j]);
					if(ret.Item4) // if the block is floating, the pressure is applied to the water.
						b[i,j] += (new_h[i,j] - enter_y)/rate; // this is also the distance between the water and the block.
					mask[i,j] = true;
					exit_h[i,j] = block.transform.TransformPoint(ret.Item3).y;
					enter_h[i,j] = enter_y;
				}
			}
		}

	}

	// private float shallow_wave_dt = 0.001875f;

	// Diminish the virtual height of the water.
	void Shallow_Wave(float[,] old_h, float[,] h, float [,] new_h)
	{		
		//Step 1:
		//TODO: Compute new_h based on the shallow wave model.
		A_Times(h, cg_r, 0, size-1, 0, size-1); // calculate the Laplacian smoother for all column.
		
		// init the mask and exit height for each block.
		bool[][,] cg_masks = new bool[cubes.Length][,];
		float[][,] exit_hs = new float[cubes.Length][,];
		float[][,] enter_hs = new float[cubes.Length][,];
		for(int q=0; q<cubes.Length; q++){
			cg_masks[q] = new bool[size,size];
			exit_hs[q] = new float[size,size];
			enter_hs[q] = new float[size,size];
			for(int i=0; i<size; i++){
				for(int j=0; j<size; j++){
					cg_masks[q][i,j] = false;
					exit_hs[q][i,j] = 0;
					enter_hs[q][i,j] = 0;
				}
			}
		}

		for(int i=0; i<size; i++){
			for(int j=0; j<size; j++){
				new_h[i,j] = h[i,j] + damping * (h[i,j] - old_h[i,j]) - rate*cg_r[i,j];
				b[i,j] = 0; // set b to zero.
				vh[i,j] = 0f;
			}
		}

		//Step 2: Block->Water coupling

		// OPTIMISE: If concatenate block_1 and block_2 vh together, the result is almost the same(small step update)

		//TODO: for block 1, calculate low_h.
		//TODO: then set up b and cg_mask for conjugate gradient.
		for(int q=0; q<cubes.Length; q++){
			calc_cube_b(cubes[q], b ,cg_masks[q] ,new_h,exit_hs[q],enter_hs[q]);
			// the mask of all blocks is a union of all masks.
			for(int i=0; i<size; i++){
				for(int j=0; j<size; j++){
					cg_mask[i,j] = cg_mask[i,j] || cg_masks[q][i,j];
				}
			}
		}
		//TODO: Solve the Poisson equation to obtain vh (virtual height).

		Conjugate_Gradient(cg_mask, b, vh, 0, size-1, 0, size-1);

		//TODO: Diminish vh.		
		for(int i=0; i<size; i++){
			for(int j=0; j<size; j++){
				vh[i,j] *= gamma;
			}
		}
		//TODO: Update new_h by vh (actually an axtra term in pressure term).
		float [,] delta_h = new float[size,size];
		// actually for sparse floating object, could union all mask and do "expansion filter" to create new mask.
		A_Times(vh, delta_h, 0, size-1, 0, size-1);

		//Step 3
		//TODO: old_h <- h; h <- new_h;
		for(int i=0; i<size; i++){
			for(int j=0; j<size; j++){
				new_h[i,j] -= delta_h[i,j] * rate;
				old_h[i,j] = h[i,j];
				h[i,j] = new_h[i,j];
			}
		}

		//Step 4: Water->Block coupling.
		//calculate the pressure (density * g * vh * A) applied to the block.
		// if the pressure is applied to cube, still need to add pressure when water is higher than the block,
		
		for(int q=0;q<cubes.Length;q++){
			for(int i=0; i<size; i++){
				for(int j=0; j<size; j++){
					// positive virtual height means block is apply pressure to water.
					// the amount of pressure is the same as the "virtual rising water height",
					// but this case is only adaptive when block is floating, that is, for every ray aligned with y axis, the entry point is below the water and exit point above.
					if(cg_masks[q][i,j]){
						// this is the expression of floating force.
						float buoyancy = density * 0.098f * 
							(exit_hs[q][i,j] < h[i,j] ? (exit_hs[q][i,j]-enter_hs[q][i,j]):vh[i,j]);
						cube_rigids[q].apply_new_force(
							new Vector3(0, buoyancy,0),
							new Vector3(getXZ(i), enter_hs[q][i,j], getXZ(j)));
					}
				}
			}
			cube_rigids[q].update_force();
		}
		//More TODO here.
	}
	

	// Update is called once per frame
	void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X    = mesh.vertices; // X stores the vertex positions of the water.
		float[,] new_h = new float[size, size];
		float[,] h     = new float[size, size];

		//TODO: Load X.y into h.
		for(int i=0; i<size; i++){
			for(int j=0; j<size; j++){
				h[i,j] = X[i*size+j].y;
			}
		}

		if (Input.GetKeyDown ("r")) 
		{
			//TODO: Add random water.
			int rand_i = Random.Range(0, size);
			int rand_j = Random.Range(0, size);

			// for imcompressible fluid, need to reduce the water height at the surround column.
			h[rand_i, rand_j] += manual_wave_height;
			// this may cross the boundary to another side, instead of surrounding column. 
			h[(rand_i+size-1)%size, rand_j] -= manual_wave_height/4;
			h[(rand_i+1)%size, rand_j] -= manual_wave_height/4;
			h[rand_i, (rand_j+size-1)%size] -= manual_wave_height/4;
			h[rand_i, (rand_j+1)%size] -= manual_wave_height/4;
		}
	
		for(int l=0; l<8; l++) // 8 times simulation
		{
			Shallow_Wave(old_h, h, new_h);
		}

		//TODO: Store h back into X.y and recalculate normal.
		for(int i=0; i<size; i++){
			for(int j=0; j<size; j++){
				X[i*size+j].y = h[i,j];
			}
		}

		mesh.vertices = X;
		mesh.RecalculateNormals();
	}
}
