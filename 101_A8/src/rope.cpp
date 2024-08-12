#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        // for each node, create a mass and connect it to the previous one with a spring.
        for(int q=0; q<num_nodes; q++){
            Mass *m = new Mass(start + 1.0f*q/(num_nodes-1.0f)*(end-start), node_mass, false); // node_mass is the mass of each node
            masses.push_back(m);
            if(q == 0) continue;
            Spring *s = new Spring(masses[q-1], masses[q], k); // k is the spring constant
            springs.push_back(s);
        }
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D displacement = s->m1->position - s->m2->position; // from m2 to m1
            Vector2D force2_1 = - (s->k) * (displacement.norm() - s->rest_length) * displacement.unit();
            s->m1->forces += force2_1;
            s->m2->forces += -force2_1;

        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += m->mass * gravity;
                // TODO (Part 2): Add global damping, this is not damp of spring, just air resistance
                m->forces -= 0.01 * m->velocity; // damping directly on velocity
                // then compute the new velocity and position, using Euler's method/ Modified Euler 
                Vector2D new_v = m->velocity + delta_t * (m->forces / m->mass);
                m->position += delta_t/2 * (m->velocity + new_v);
                m->velocity = new_v;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D displacement = s->m1->position - s->m2->position; // from m2 to m1
            float dist = displacement.norm();
            Vector2D correction = (dist - s->rest_length) / dist * displacement;
            if(!s->m1->pinned) s->m1->position -= correction / 2;
            if(!s->m2->pinned) s->m2->position += correction / 2; // directly to the position,  the correction is divided by 2
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass (do not use force here)
                m->position = m->position + (m->position - m->last_position) + (gravity-0.008*m->velocity/m->mass) * delta_t * delta_t;
                // TODO (Part 4): Add global Verlet damping
                m->velocity = (m->position - m->last_position) / delta_t;
                m->last_position = temp_position;
            }
        }
    }
}
