#pragma once

#include <cgv/render/drawable.h>
#include <cgv/gui/provider.h>
#include <cgv_gl/arrow_renderer.h>
#include <cgv_gl/box_renderer.h>
#include <cgv_gl/sphere_renderer.h>
#include <random>
#include <vector>

#include <util.h>
#include <rbody.h>

#include "lib_begin.h"

/* TODO: check description comments for each member */

namespace physics {

	/** provides simulation of rigid body dynamics on oriented bounding boxes */
	class CGV_API rigid_body : public cgv::render::drawable, public cgv::gui::provider {
	protected:
		/**@name simulation world management*/
		//@{
		/// the gravitational force to be used
		vec3 gravity;
		/// world size
		float world_sphere_size;
		/// list of simulation bodies
		std::vector<rbody> bodies;
	public:
		/// adds another rigid body to the simulation
		void add_body(rbody b);
		/// returns a reference to the list of bodies
		std::vector<rbody>& get_bodies() { return bodies; }
		/// removes all bodies from the world
		void reset();
		//@}
	protected:
		/**@name rendering */
		//@{
		/// pointer reference to the current view
		//cgv::render::view* view_ptr;
		/// render style for the sphere representation of contact points
		cgv::render::sphere_render_style sphere_style;
		/// render style for the arrow representation of collision normals
		cgv::render::arrow_render_style arrow_style;
		/// attribute manager for box rendering
		cgv::render::attribute_array_manager b_manager;
		//@}

		/**@name simulation*/
		//@{
		/// if gravitational forces should be appied
		bool use_gravity;
		/// if friction forces should be applied
		bool use_friction;
		/// whether to use accumulate impulses optimization for sequential impulse calculation
		bool use_accumulate_impulses;
		/// how many iterations should be used for the sequential impulses solver
		unsigned solver_iterations;
		/// time step used for explicit integration
		//float time_step;
		/// simulation time
		float simulation_time;
		/// list of all collisions
		std::vector<collision> collisions;
		/// test for colliding objects
		void check_collisions();
		/// perform collision handling to alter object velocities
		void handle_collisions(float dt);
	public:
		/// perform one simulation step
		void step_simulation(float dt);
		//@}
	public:
		/**@name main viewer interface implementation*/
		//@{
		/// construct with default parameters
		rigid_body();
		/// just configures background color
		bool init(cgv::render::context& ctx);
		/// ensure that framebuffer object is created in sampling resolution, that shader programs are built, and perform simulation step if necessary
		void init_frame(cgv::render::context& ctx);
		/// ensure the scene is created with selected number of bodies, and draw either texture in 2D or scene 3D mode
		void draw(cgv::render::context& ctx);
		/// clean up
		void clear(cgv::render::context& ctx);
		/// create the gui
		void create_gui() {}
		void create_gui(cgv::base::base* bp, cgv::gui::provider& p);
		//@}
	};
}

#include <cgv/config/lib_end.h>
