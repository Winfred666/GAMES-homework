using UnityEngine;

// here implement a rigid body dynamic for the cube
public class Cube_rigid_body :MonoBehaviour
{
    Vector3 offset;
    IInteract_cube Icube;
    
    [SerializeField]
    float mass = 0.5f;

    Vector3 v = Vector3.zero; // linear velocity
    Vector3 w = Vector3.zero; // angular velocity

    Vector3 outer_force = Vector3.zero; // outer force applied to the cube
    Vector3 outer_torque = Vector3.zero; // outer torque applied to the cube

    Vector3 Rr_average = Vector3.zero; // average arm stregth of torque that applied to the cube
    Vector3 delta_v_average = Vector3.zero; // average velocity change
    int v_num = 0;

    float dt 			= 0.015f; // time step

    [SerializeField]
    float side_length = 1.0f;


    // [Range(0.0f, 1.0f),Tooltip("should be smaller, because water drag is not considered")]
    float angular_damping = 0.98f;
    
    float linear_damping = 0.99f;


    Matrix4x4 rotation_matrix = Matrix4x4.identity;

    Matrix4x4 I_ref_inv = Matrix4x4.identity;


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

    void Start(){
        // Retrieve the component that implements IInteractable
        Icube = GetComponent<IInteract_cube>();
        if(Icube == null){
            Debug.LogError("IInteract_cube not found in the node");
        }

        // get the moment of inertia of cube:
        I_ref_inv = MultiplyMatrixByScalar(Matrix4x4.identity,1.0f/6.0f *mass*side_length*side_length).inverse;
    }

    // to sync the cube with the physics engine, called by wave.
    public void update_force(){
        if(Icube.is_drag_by_mouse())return; // if the cube is dragging by mouse, no physics is applied

        Vector3 impulse = Vector3.zero; // impulse
        
        if(v_num > 0){
            delta_v_average /= v_num;
            Rr_average /= v_num;
        }

        // apply gravity and outer force
        impulse += (new Vector3(0, -9.8f, 0) * mass + outer_force) * dt;

        //apply the force and torque
        Matrix4x4 I_inv = rotation_matrix * I_ref_inv * rotation_matrix.transpose;
        Vector3 impulse_j = Vector3.zero;
        if(delta_v_average.magnitude > 0.001){
            Matrix4x4 Rr_i_dual = Get_Cross_Matrix(Rr_average);
            Matrix4x4 K = SubMatrices(MultiplyMatrixByScalar(Matrix4x4.identity,1.0f/mass),
                Rr_i_dual*I_inv*Rr_i_dual);
            impulse_j = (K.inverse).MultiplyVector(delta_v_average);
        }
        // update w
        w += I_inv.MultiplyVector( outer_torque*dt + Vector3.Cross(Rr_average,impulse_j) );
        w *= angular_damping;
        // update v
        v += (impulse + impulse_j)/mass;
        v *= linear_damping;


        //clear state

        // first way of applying force: setting velocity of certain point
        delta_v_average = Vector3.zero;
        Rr_average = Vector3.zero;
        v_num = 0;
        // second way of applying force: setting force and torque
        outer_force = Vector3.zero;
        outer_torque = Vector3.zero;
    }

    public void update_position(){
        if(Icube.is_drag_by_mouse()){
            // also need to reset impulse, or the water impulse will add up,
            // and the cube will fly away when the mouse is released
            outer_force = Vector3.zero;
            w = Vector3.zero;
            v = Vector3.zero;
            return;
        }
        // update the position and rotation
        Quaternion dq = new Quaternion(dt/2*w.x, dt/2*w.y, dt/2*w.z, 0) * transform.rotation;
        transform.rotation = new Quaternion(
            transform.rotation.x + dq.x, 
            transform.rotation.y + dq.y, 
            transform.rotation.z + dq.z, 
            transform.rotation.w + dq.w);
        
        transform.position += v * dt;

        rotation_matrix =  Matrix4x4.Rotate(transform.rotation);
    }

    void Update(){
        update_position();
    }

    // this apply new_v to specifit point of the cube, calculate the force and torque by it
    //and dt is controled overall. position is relative to world.
    public void apply_new_vi(Vector3 new_v_i, Vector3 position){
        Vector3 Rr_i =  rotation_matrix.MultiplyVector(position - transform.position);
        Vector3 old_v_i = v + Vector3.Cross(w, Rr_i);
        delta_v_average += new_v_i - old_v_i;
        Rr_average += Rr_i;
        v_num ++;
    }

    // this is called by wave, to apply bouyancy to the cube
    public void apply_new_force(Vector3 force, Vector3 position){
        if(Icube.is_drag_by_mouse())return;
        Vector3 Rr_i =  rotation_matrix.MultiplyVector(position - transform.position);
        outer_force += force;
        outer_torque += Vector3.Cross(Rr_i, force);
    }

    public float get_fy(){
        // return v.y*dt + y_acc/2.0f*dt*dt;
        return outer_force.y - 9.8f*mass;
    }
}