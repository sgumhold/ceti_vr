#include "rigid_body_viewer.h"
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

/************* scene management ****************************************/

void generate_ground_plane(std::vector<rbody>& bodies) {
	rbody b(box3(vec3(-1.0f, -0.05f, -1.0f), vec3(1.0f, 0.05f, 1.0f)));
	b.position = vec3(0.0f, -0.05f, 0.0f);
	b.make_static();
	b.color = cgv::media::color<float, cgv::media::HLS>(0.0f, 0.3f, 0.0f);
	bodies.push_back(b);
}

void generate_scene_1(std::vector<rbody>& bodies) {
	{
		rbody b(box3(vec3(-0.05f, -0.2f, -1.0f), vec3(0.05f, 0.2f, 1.0f)));
		b.position = vec3(0.95f, 0.2f, 0.0f);
		b.make_static();
		b.color = cgv::media::color<float, cgv::media::HLS>(0.0f, 0.3f, 0.0f);
		bodies.push_back(b);
	}
	{
		rbody b(box3(-0.1f, 0.1f));
		b.position = vec3(-1.0f, 0.2f, 0.0f);
		b.orientation = quat(vec3(0.0f, 1.0f, 0.0f), cgv::math::deg2rad(45.0f));
		b.orientation.normalize();
		b.velocity = vec3(4.0f, 0.0f, 0.0f);
		b.color = cgv::media::color<float, cgv::media::HLS>(0.5f, 0.4f, 1.0f);
		bodies.push_back(b);
	}
}

void generate_scene_2(std::vector<rbody>& bodies) {

	for(unsigned i = 0; i < 5; ++i) {
		float x = static_cast<float>(i) * 0.2f;

		rbody b(box3(-0.1f, 0.1f));
		b.position = vec3(-0.2f + x, 0.11f, 0.0f);
		b.friction = 0.3f;

		b.color = cgv::media::color<float, cgv::media::HLS>(0.5f, 0.4f, 1.0f);
		bodies.push_back(b);
	}

	rbody b(box3(-0.1f, 0.1f));
	b.position = vec3(-1.0f, 0.13f, 0.0f);
	b.velocity = vec3(10.0f, 0.0f, 0.0f);
	b.friction = 0.3f;
	b.calculate_mass_and_inertia(15.0f);

	b.color = cgv::media::color<float, cgv::media::HLS>(0.0f, 0.4f, 1.0f);
	bodies.push_back(b);
}

void generate_scene_3(std::vector<rbody>& bodies) {

	unsigned count = 6u;

	float size = 0.15f;
	float offset = static_cast<float>(count - 1u);
	offset = 0.5f * offset * size;

	for(unsigned y = 0; y < count; ++y) {
		float yf = static_cast<float>(y) * size;

		for(unsigned x = y; x < count; ++x) {
			float xf = static_cast<float>(x) * size;

			rbody b(box3(-0.5f * size, 0.5f * size));
			b.position = vec3(xf - offset - 0.5f*yf, yf + 0.5f * size, 0.0f);
			b.friction = 0.3f;

			b.color = cgv::media::color<float, cgv::media::HLS>(0.2f, 0.4f, 1.0f);
			bodies.push_back(b);
		}
	}
}

void rigid_body_viewer::generate_scene() {

	rb.reset();

	std::vector<rbody> bodies;

	generate_ground_plane(bodies);

	switch(current_scene) {
	case S_SLIDING:
		generate_scene_1(bodies);
		break;
	case S_IMPACT:
		generate_scene_2(bodies);
		break;
	case S_STACKED:
		generate_scene_3(bodies);
		break;
	case S_PLANE:
	default:
		break;
	}

	for(unsigned i = 0; i < bodies.size(); ++i) {
		rb.add_body(bodies[i]);
	}
}


void rigid_body_viewer::generate_body() {
	std::uniform_real_distribution<float> uni_dist(0.0, 1.0);

	vec3 size = 0.2f * vec3(uni_dist(rand_gen), uni_dist(rand_gen), uni_dist(rand_gen)) + 0.05f;
	vec3 offset = vec3(0.2f * uni_dist(rand_gen) - 0.1f, 0.2f * uni_dist(rand_gen) - 0.1f + 0.8f, 0.2f * uni_dist(rand_gen) - 0.1f);

	rbody b(box3(-0.5f * size, 0.5f * size));
	b.position = offset;
	b.color = cgv::media::color<float, cgv::media::HLS>(uni_dist(rand_gen), 0.4f, 1.0f);
	rb.add_body(b);
}

void rigid_body_viewer::shoot_body() {
	std::uniform_real_distribution<float> uni_dist(0.0, 1.0);

	auto view_ptr = find_view_as_node();
	vec3 eye = view_ptr->get_eye();

	vec3 size = 0.1f * vec3(uni_dist(rand_gen), uni_dist(rand_gen), uni_dist(rand_gen)) + 0.05f;

	rbody b(box3(-0.5f * size, 0.5f * size));
	b.position = eye;
	b.velocity = 20.0f * normalize(vec3(mouse_target) - eye);
	b.angular_velocity = 4.0f * (2.0f * vec3(uni_dist(rand_gen), uni_dist(rand_gen), uni_dist(rand_gen)) - 1.0f);
	b.calculate_mass_and_inertia(20.0f);
	b.color = cgv::media::color<float, cgv::media::HLS>(uni_dist(rand_gen), 0.4f, 1.0f);
	rb.add_body(b);
}

void rigid_body_viewer::timer_event(double t, double dt)
{
	// TODO: use dt to step simulation here 
	if(animate || single_step) {
		rb.step_simulation(float(dt));
		single_step = false;
		update_member(&single_step);
		
		post_redraw();
	}
}

/************* main interface ****************************************/

rigid_body_viewer::rigid_body_viewer()
{
	set_name("Rigid Body Viewer");

	current_scene = S_PLANE;

	do_reset = false;
	do_generate_body = false;
	do_shoot_body = false;
	do_detonate_bomb = false;

	animate = false;
	single_step = false;

	generate_scene();

	cgv::signal::connect(cgv::gui::get_animation_trigger().shoot, this, &rigid_body_viewer::timer_event);
}

//
bool rigid_body_viewer::self_reflect(cgv::reflect::reflection_handler& _rh)
{
	return false;
}

// overload and implement this method to handle events
bool rigid_body_viewer::handle(cgv::gui::event& e)
{
	if(e.get_kind() == cgv::gui::EID_KEY) {
		cgv::gui::key_event& ke = (cgv::gui::key_event&) e;
		if(ke.get_action() != cgv::gui::KA_PRESS)
			return false;

		switch(ke.get_key()) {
		case 'A':
			animate = !animate;
			on_set(&animate);
			break;
		case 'B':
			do_detonate_bomb = true;
			post_redraw();
			break;
		case 'G':
			do_generate_body = true;
			post_redraw();
			break;
		case 'R':
			do_reset = true;
			post_redraw();
			break;
		case 'S':
			single_step = true;
			post_redraw();
			break;
		case cgv::gui::Keys::KEY_Space:
			do_shoot_body = true;
			post_redraw();
			break;
		default:
			return false;
		}
	} else if(e.get_kind() == cgv::gui::EID_MOUSE) {
		cgv::gui::mouse_event me = (cgv::gui::mouse_event&) e;

		if(me.get_action() == cgv::gui::MA_MOVE) {
			unsigned x = me.get_x();
			unsigned y = me.get_y();
			get_world_location(x, y, *view_ptr, mouse_target);

			post_redraw();
		}
		return false;
	}

	return true;
}

// overload to stream help information to the given output stream
void rigid_body_viewer::stream_help(std::ostream& os)
{
	os << "rigid_body_viewer: scene      ... <R>eset, <G>enerate body, <Space> shoot box, detonate <B>omb\n";
	os << "				      simulation ... <A>nimate, <S>ingle step" << std::endl;
}

// stream statistical information
void rigid_body_viewer::stream_stats(std::ostream& os)
{
	os << "3D <";
	os << ">";
}

//
void rigid_body_viewer::on_set(void* member_ptr)
{
	if(member_ptr == &current_scene) {
		do_reset = true;
	}

	update_member(member_ptr);
	post_redraw();
}

// 
bool rigid_body_viewer::init(cgv::render::context& ctx)
{
	if(!(view_ptr = find_view_as_node())) {
		return false;
	}

	return rb.init(ctx);
}

//
void rigid_body_viewer::init_frame(cgv::render::context& ctx)
{
	if(do_generate_body) {
		generate_body();

		do_generate_body = false;
		update_member(&do_generate_body);
	}

	if(do_shoot_body) {
		shoot_body();

		do_shoot_body = false;
		update_member(&do_shoot_body);
	}

	if(do_reset) {
		generate_scene();

		do_reset = false;
		animate = false;
		update_member(&do_reset);
		update_member(&animate);
	}

	if(do_detonate_bomb) {
		std::vector<rbody>& bodies = rb.get_bodies();

		for(unsigned i = 0; i < bodies.size(); ++i) {
			rbody& body = bodies[i];
			vec3 direction = body.position - vec3(0.0f, -0.2f, 0.0f);
			float distance = direction.length();
			direction.normalize();

			vec3 helper = util::ortho_vec(direction);
			vec3 axis = normalize(cross(direction, helper));

			float amount = 10000.0f * 1.0f / (1.0f + distance * distance);
			body.force = amount * direction;
			body.torque = 0.01f * amount * axis;
		}

		do_detonate_bomb = false;
		update_member(&do_detonate_bomb);
	}

	/*if(animate || single_step) {
		rb.step_simulation();
		single_step = false;
		update_member(&single_step);
	}*/

	rb.init_frame(ctx);
}

//
void rigid_body_viewer::draw(cgv::render::context& ctx)
{
	rb.draw(ctx);
}

// 
void rigid_body_viewer::create_gui()
{
	add_decorator("Rigid Body Viewer", "heading", "level=2");
	align("\a");
	add_member_control(this, "scene", current_scene, "dropdown", "enums='plane,sliding,impact,stacked'");
	add_member_control(this, "reset", do_reset, "toggle");
	add_member_control(this, "generate_body", do_generate_body, "toggle");
	add_member_control(this, "animate", animate, "toggle");
	add_member_control(this, "single_step", single_step, "toggle");
	align("\b");
	
	rb.create_gui(this, *this);
}

#include "lib_begin.h"
#include <cgv/base/register.h>

extern CGV_API cgv::base::object_registration<rigid_body_viewer> rigid_body_viewer_reg("rigid body viewer");
