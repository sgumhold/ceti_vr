#include "rigid_body.h"
#include <cgv/base/find_action.h>
#include <cgv/math/ftransform.h>
#include <cgv/gui/trigger.h>
#include <cgv/gui/key_event.h>
#include <cgv/gui/mouse_event.h>
#include <cgv/reflect/reflect_enum.h>
#include <cgv/signal/rebind.h>
#include <cgv_gl/gl/gl.h>
#include <cgv_gl/gl/gl_tools.h>

namespace cgv {
	namespace reflect {
	}
}

namespace physics {

	/************* simulation world management ****************************************/

	void rigid_body::add_body(rbody b) {

		bodies.push_back(b);
	}

	void rigid_body::reset() {

		simulation_time = 0.0f;
		update_member(&simulation_time);

		bodies.clear();
		collisions.clear();
	}

	/************* simulation ****************************************/

	void rigid_body::step_simulation(float dt) {

		simulation_time += dt;
		update_member(&simulation_time);

		for(unsigned i = 0; i < bodies.size(); ++i) {
			rbody &b = bodies[i];

			vec3 force = b.force; //vec3(0.0f);
			vec3 torque = b.torque; //vec3(0.0f);

			if(b.inv_mass == 0.0f)
				continue;

			force *= b.inv_mass;
			if(use_gravity)
				force += gravity;

			// use semi-implicit Euler integration to calculate the new velocity
			b.velocity += dt * force;

			mat3 orientation_mat;
			b.orientation.put_matrix(orientation_mat);

			mat3 inertia_world = orientation_mat * b.inv_inertia;

			orientation_mat.transpose();
			inertia_world *= orientation_mat;

			b.angular_velocity += dt * inertia_world * torque;
		}

		check_collisions();
		handle_collisions(dt);

		std::vector<rbody> next_bodies;
		next_bodies.reserve(bodies.size());

		for(unsigned i = 0; i < bodies.size(); ++i) {
			rbody &b = bodies[i];

			b.position += dt * b.velocity;
			b.force = vec3(0.0f);
			b.torque = vec3(0.0f);

			vec3 axis = b.angular_velocity;
			float av = axis.length();
			if(av >= std::numeric_limits<float>::epsilon()) {
				quat rotation = quat(axis / av, dt * av);
				b.orientation = rotation * b.orientation;
			}

			if(b.position.length() < world_sphere_size) {
				next_bodies.push_back(b);
			}
		}

		bodies = next_bodies;
	}

	void rigid_body::check_collisions() {
		collisions.clear();

		for(unsigned i = 0; i < bodies.size(); ++i) {
			rbody& b1 = bodies[i];

			for(unsigned j = i + 1; j < bodies.size(); ++j) {
				bool intersection = true;

				rbody& b2 = bodies[j];

				// if both bodies are static we dont need to do a collision check
				if(b1.inv_mass == 0.0f && b2.inv_mass == 0.0f)
					continue;

				std::vector<rbody::test_case> test_cases = b1.get_test_cases(b2);

				rbody::test_case min_case;
				float min_dist = std::numeric_limits<float>::max();
				vec3 mtv(0.0f); // minimum translation vector

				for(unsigned k = 0; k < test_cases.size(); ++k) {
					rbody::test_case& tc = test_cases[k];
					vec3 axis = tc.axis;

					if(fabsf(axis.sqr_length()) <= std::numeric_limits<float>::epsilon()) continue;

					axis.normalize();

					std::vector<float> projected0 = b1.project_corners(axis);
					std::vector<float> projected1 = b2.project_corners(axis);

					float min0 = std::numeric_limits<float>::max();
					float max0 = -std::numeric_limits<float>::max();
					float min1 = std::numeric_limits<float>::max();
					float max1 = -std::numeric_limits<float>::max();

					for(unsigned p = 0; p < 8; ++p) {
						float l0 = projected0[p];
						float l1 = projected1[p];

						min0 = std::min(min0, l0);
						max0 = std::max(max0, l0);

						min1 = std::min(min1, l1);
						max1 = std::max(max1, l1);
					}

					float center0 = 0.5f * (min0 + max0);
					float center1 = 0.5f * (min1 + max1);

					float invert = -1.0f;
					if(center0 > center1) {
						std::swap(min0, min1);
						std::swap(max0, max1);
						invert = 1.0f;
					}

					float dist = min1 - max0;

					if(dist > 0.0f) {
						intersection = false;
					} else {
						// TODO: remove edge check?
						bool is_edge = tc.feature == rbody::test_feature::EDGE;

						if(-dist < min_dist && ((is_edge && min_dist + dist > 0.01f) || !is_edge)) {
							min_dist = -dist;
							mtv = invert * dist * axis;
							min_case = tc;
						}
					}
				}

				if(intersection) {
					std::vector<collision> pair_collisions = b1.get_collisions(&b2, min_case, min_dist);

					for(unsigned k = 0; k < pair_collisions.size(); ++k)
						collisions.push_back(pair_collisions[k]);
				}
			}
		}
	}

	void rigid_body::handle_collisions(float dt) {
		float allowed_penetration = 0.0001f;
		float bias_factor = 0.2f;

		mat3 I;
		I.identity();

		// precalculate collision attributes
		for(unsigned i = 0; i < collisions.size(); ++i) {
			collision& c = collisions[i];

			vec3 r1 = c.position - c.body1->position;
			vec3 r2 = c.position - c.body2->position;
			c.r1 = r1;
			c.r2 = r2;

			mat3 m1 = c.body1->inv_inertia*(dot(r1, r1) * I - dyad(r1, r1));
			mat3 m2 = c.body2->inv_inertia*(dot(r2, r2) * I - dyad(r2, r2));

			float inv_mass_sum = c.body1->inv_mass + c.body2->inv_mass;
			c.normal_mass = 1.0f / (inv_mass_sum + dot(c.normal, m1 * c.normal) + dot(c.normal, m2 * c.normal));

			if(dt < std::numeric_limits<float>::epsilon())
				c.bias = 0.0f;
			else
				c.bias = -bias_factor * (1.0f / dt) * std::min(0.0f, c.separation + allowed_penetration);

			c.friction = sqrtf(c.body1->friction * c.body2->friction);

			vec3 v1 = c.body1->velocity + cross(c.body1->angular_velocity, r1);
			vec3 v2 = c.body2->velocity + cross(c.body2->angular_velocity, r2);

			vec3 v_rel = v2 - v1;

			if(v_rel.length() > 0.0f) {
				vec3 bitangent = cross(c.normal, normalize(v_rel));
				vec3 tangent = cross(c.normal, bitangent);
				if(tangent.length() > 0.0f) {
					tangent.normalize();
					c.tangent = tangent;

					c.tangent_mass = 1.0f / (inv_mass_sum + dot(tangent, m1 * tangent) + dot(tangent, m2 * tangent));
				} else {
					c.tangent = vec3(0.0f);
				}
			}
		}

		for(unsigned i = 0; i < solver_iterations; ++i) {
			for(unsigned j = 0; j < collisions.size(); ++j) {
				collision& c = collisions[j];

				vec3 v1 = c.body1->velocity + cross(c.body1->angular_velocity, c.r1);
				vec3 v2 = c.body2->velocity + cross(c.body2->angular_velocity, c.r2);

				vec3 v_rel = v2 - v1;
				float v_rel_n = dot(v_rel, c.normal);

				float dPn = (-v_rel_n + c.bias) * c.normal_mass;

				if(use_accumulate_impulses) {
					float Pn0 = c.Pn;
					c.Pn = std::max(Pn0 + dPn, 0.0f);
					dPn = c.Pn - Pn0;
				} else {
					dPn = std::max(dPn, 0.0f);
				}

				vec3 Pn = dPn * c.normal;

				c.body1->velocity -= c.body1->inv_mass * Pn;
				c.body1->angular_velocity -= c.body1->inv_inertia * cross(c.r1, Pn);

				c.body2->velocity += c.body2->inv_mass * Pn;
				c.body2->angular_velocity += c.body2->inv_inertia * cross(c.r2, Pn);

				if(use_friction) {
					float v_rel_t = dot(v_rel, c.tangent);

					float dPt = -v_rel_t * c.tangent_mass;

					if(use_accumulate_impulses) {
						float maxPt = c.friction * c.Pn;

						float Pt0 = c.Pt;
						c.Pt = cgv::math::clamp(Pt0 + dPt, -maxPt, maxPt);
						dPt = c.Pt - Pt0;
					} else {
						float maxPt = fabsf(c.friction * dPn);
						dPt = cgv::math::clamp(dPt, -maxPt, maxPt);
					}

					vec3 Pt = dPt * c.tangent;

					c.body1->velocity -= c.body1->inv_mass * Pt;
					c.body1->angular_velocity -= c.body1->inv_inertia * cross(c.r1, Pt);

					c.body2->velocity += c.body2->inv_mass * Pt;
					c.body2->angular_velocity += c.body2->inv_inertia * cross(c.r2, Pt);
				}
			}
		}
	}

	/************* main interface ****************************************/

	rigid_body::rigid_body() {
		
		gravity = vec3(0.0f, -9.81f, 0.0f);
		bodies.clear();

		use_gravity = true;
		use_friction = true;
		use_accumulate_impulses = true;

		solver_iterations = 30u;

		world_sphere_size = 5.0f;

		//time_step = 0.01f;
		simulation_time = 0;
	}
 
	bool rigid_body::init(cgv::render::context& ctx) {
		if(!b_manager.init(ctx))
			return false;

		cgv::render::ref_arrow_renderer(ctx, 1);
		cgv::render::ref_box_renderer(ctx, 1);
		cgv::render::ref_sphere_renderer(ctx, 1);

		arrow_style.length_scale = 0.05f;
		arrow_style.radius_relative_to_length = 0.05f;

		sphere_style.blend_width_in_pixel = 0.0f;

		ctx.set_bg_clr_idx(3);

		return true;
	}

	//
	void rigid_body::init_frame(cgv::render::context& ctx) {
		
	}

	//
	void rigid_body::draw(cgv::render::context& ctx) {
		
		if(collisions.size() > 0) {
			std::vector<vec3> points;
			std::vector<vec4> colors;
			std::vector<float> sizes;

			std::vector<vec3> normals;

			for(unsigned i = 0; i < collisions.size(); ++i) {
				points.push_back(collisions[i].position);
				colors.push_back(vec4(1.0f));
				sizes.push_back(0.005f);
				normals.push_back(collisions[i].normal);
			}

			cgv::render::sphere_renderer& s_renderer = cgv::render::ref_sphere_renderer(ctx);
			s_renderer.set_render_style(sphere_style);

			s_renderer.set_position_array(ctx, points);
			s_renderer.set_color_array(ctx, colors);
			s_renderer.set_radius_array(ctx, sizes);

			if(s_renderer.validate_and_enable(ctx)) {
				glDrawArrays(GL_POINTS, 0, GLsizei(collisions.size()));
				s_renderer.disable(ctx);
			}

			cgv::render::arrow_renderer& a_renderer = cgv::render::ref_arrow_renderer(ctx);
			a_renderer.set_render_style(arrow_style);
			a_renderer.set_position_array(ctx, points);
			a_renderer.set_color_array(ctx, colors);
			a_renderer.set_direction_array(ctx, normals);

			if(a_renderer.validate_and_enable(ctx)) {
				glDrawArraysInstanced(GL_POINTS, 0, GLsizei(points.size()), arrow_style.nr_subdivisions);
				a_renderer.disable(ctx);
			}
		}

		// render boxes
		if(bodies.size() > 0) {
			std::vector<box3> boxes;
			std::vector<quat> rotations;
			std::vector<vec3> translations;
			std::vector<rgb> colors;

			std::vector<vec3> pos, ext;

			for(unsigned i = 0; i < bodies.size(); ++i) {
				boxes.push_back(bodies[i].box);
				rotations.push_back(bodies[i].orientation);
				translations.push_back(bodies[i].position);

				colors.push_back(bodies[i].color);
			}

			cgv::render::box_render_style box_style;
			box_style.culling_mode = cgv::render::CullingMode::CM_BACKFACE;

			cgv::render::box_renderer& b_renderer = cgv::render::ref_box_renderer(ctx);
			b_renderer.set_render_style(box_style);
			b_renderer.set_attribute_array_manager(ctx, &b_manager);

			b_renderer.set_box_array(ctx, boxes);

			b_renderer.set_translation_array(ctx, translations);
			b_renderer.set_rotation_array(ctx, rotations);
			b_renderer.set_color_array(ctx, colors);

			if(b_renderer.validate_and_enable(ctx)) {
				glDrawArrays(GL_POINTS, 0, GLsizei(bodies.size()));
			}

			b_renderer.disable(ctx);
			glFlush();
		}
	}

	void rigid_body::clear(cgv::render::context& ctx) {
		bodies.clear();
		collisions.clear();

		b_manager.destruct(ctx);

		cgv::render::ref_arrow_renderer(ctx, -1);
		cgv::render::ref_box_renderer(ctx, -1);
		cgv::render::ref_sphere_renderer(ctx, -1);
	}

	void rigid_body::create_gui(cgv::base::base* bp, cgv::gui::provider& p) {
		if(p.begin_tree_node("Rigid Body", use_gravity, false, "level=3")) {
			p.align("\a");
			p.add_member_control(bp, "enable gravity", use_gravity, "check");
			p.add_member_control(bp, "enable friction", use_friction, "check");
			p.add_member_control(bp, "accumulate impulses", use_accumulate_impulses, "check");
			p.add_member_control(bp, "solver iterations", solver_iterations, "value_slider", "min=1;max=100;ticks=true");
			p.add_member_control(bp, "world sphere size", world_sphere_size, "value_slider", "min=1.0;max=100.0;ticks=true");
			//p.add_member_control(bp, "time_step", time_step, "value_slider", "min=0.001;step=0.001;max=10;ticks=true;log=true");
			p.add_view("simulation_time", simulation_time, "value");
			p.align("\b");
			p.end_tree_node(use_gravity);
		}
		/*if (begin_tree_node("Rendering", show_contact_points, false, "level=3")) {
			align("\a");
			add_member_control(this, "contact points", show_contact_points, "check");
			align("\b");
			end_tree_node(show_contact_points);
		}*
		if(begin_tree_node("Simulation", animate, false, "level=3")) {
			align("\a");
			add_member_control(this, "accumulate impulses", accumulate_impulses, "check");
			add_member_control(this, "use friction", use_friction, "check");
			add_member_control(this, "animate", animate, "toggle");
			add_member_control(this, "single_step", single_step, "toggle");
			add_member_control(this, "time_step", time_step, "value_slider", "min=0.001;step=0.001;max=10;ticks=true;log=true");
			add_view("simulation_time", simulation_time, "value");
			align("\b");
			end_tree_node(animate);
		}*/
	}
}
