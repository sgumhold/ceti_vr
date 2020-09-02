#include "vr_scene.h"
#include <cgv/defines/quote.h>
#include <cgv/base/register.h>
#include <cgv/math/ftransform.h>
#include <cgv/math/pose.h>
#include <cgv/utils/dir.h>
#include <cgv/utils/file.h>
#include <cg_vr/vr_events.h>
#include <random>
#include <fstream>

void vr_scene::construct_table(float tw, float td, float th, float tW, float tO, rgb table_clr, rgb leg_clr) 
{
	float x0 = -0.5f * tw;
	float x1 = -0.5f * tw + tO;
	float x2 = -0.5f * tw + tO + tW;
	float x3 = 0.5f * tw - tO - tW;
	float x4 = 0.5f * tw - tO;
	float x5 = 0.5f * tw;
	float y0 = 0;
	float y1 = th - tW;
	float y2 = th;
	float z0 = -0.5f * td;
	float z1 = -0.5f * td + tO;
	float z2 = -0.5f * td + tO + tW;
	float z3 =  0.5f * td - tO - tW;
	float z4 =  0.5f * td - tO;
	float z5 =  0.5f * td;
	boxes.push_back(box3(vec3(x0, y1, z0), vec3(x5, y2, z5))); box_colors.push_back(table_clr);

	boxes.push_back(box3(vec3(x1, y0, z1), vec3(x2, y1, z2))); box_colors.push_back(leg_clr);
	boxes.push_back(box3(vec3(x3, y0, z1), vec3(x4, y1, z2))); box_colors.push_back(leg_clr);
	boxes.push_back(box3(vec3(x3, y0, z3), vec3(x4, y1, z4))); box_colors.push_back(leg_clr);
	boxes.push_back(box3(vec3(x1, y0, z3), vec3(x2, y1, z4))); box_colors.push_back(leg_clr);
}

void vr_scene::construct_room(float w, float d, float h, float W, bool walls, bool ceiling) {
	// construct floor
	boxes.push_back(box3(vec3(-0.5f * w, -W, -0.5f * d), vec3(0.5f * w, 0, 0.5f * d)));
	box_colors.push_back(rgb(0.2f, 0.2f, 0.2f));

	if (walls) {
		// construct walls
		boxes.push_back(box3(vec3(-0.5f * w, -W, -0.5f * d - W), vec3(0.5f * w, h, -0.5f * d)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));
		boxes.push_back(box3(vec3(-0.5f * w, -W, 0.5f * d), vec3(0.5f * w, h, 0.5f * d + W)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));

		boxes.push_back(box3(vec3(0.5f * w, -W, -0.5f * d - W), vec3(0.5f * w + W, h, 0.5f * d + W)));
		box_colors.push_back(rgb(0.5f, 0.8f, 0.5f));
	}
	if (ceiling) {
		// construct ceiling
		boxes.push_back(box3(vec3(-0.5f * w - W, h, -0.5f * d - W), vec3(0.5f * w + W, h + W, 0.5f * d + W)));
		box_colors.push_back(rgb(0.5f, 0.5f, 0.8f));
	}
}

void vr_scene::construct_environment(float s, float ew, float ed, float w, float d, float h) {
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	unsigned n = unsigned(ew / s);
	unsigned m = unsigned(ed / s);
	float ox = 0.5f * float(n) * s;
	float oz = 0.5f * float(m) * s;
	for (unsigned i = 0; i < n; ++i) {
		float x = i * s - ox;
		for (unsigned j = 0; j < m; ++j) {
			float z = j * s - oz;
			if (fabsf(x) < 0.5f * w && fabsf(x + s) < 0.5f * w && fabsf(z) < 0.5f * d && fabsf(z + s) < 0.5f * d)
				continue;
			float h = 0.2f * (std::max(abs(x) - 0.5f * w, 0.0f) + std::max(abs(z) - 0.5f * d, 0.0f)) * distribution(generator) + 0.1f;
			boxes.push_back(box3(vec3(x, 0.0f, z), vec3(x + s, h, z + s)));
			rgb color = cgv::media::color<float, cgv::media::HLS>(distribution(generator), 0.1f * distribution(generator) + 0.15f, 0.3f);
			box_colors.push_back(color);
		}
	}
}

void vr_scene::build_scene(float w, float d, float h, float W)
{
	construct_room(w, d, h, W, false, false);
	construct_environment(0.3f, 3 * w, 3 * d, w, d, h);
	construct_table(table_width, table_depth, table_height, leg_width, leg_offset, table_color, leg_color);
}

vr_scene::vr_scene()
{
	set_name("vr_scene");
	current_scene_idx = 0;
	nr_vertices = 0;
	nr_edges = 0;
	vr_view_ptr = 0;
	scene_file_path = QUOTE_SYMBOL_VALUE(INPUT_DIR) "/../data";
	on_set(&scene_file_path);

	table_color = rgb(0.3f, 0.2f, 0.0f);
	table_width = 1.6f;
	table_depth = 0.8f;
	table_height = 0.7f;
	leg_color = rgb(0.2f, 0.1f, 0.1f);
	leg_width = 0.04f;
	leg_offset = 0.0f;

	build_scene(5, 7, 3, 0.2f);

	pixel_scale = 0.001f;

	on_set(&table_width);
}

std::string vr_scene::get_new_scene_file_name() const
{
	int i = (int)scene_file_names.size() -1;
	std::string file_name;
	do {
		++i;
		file_name = "scene_";
		std::string s = cgv::utils::to_string(i);
		while (s.length() < 5)
			s = std::string("0") + s;
		file_name += s + ".scn";
	} while (cgv::utils::file::exists(scene_file_path + "/" + file_name));
	return file_name;
}

bool vr_scene::self_reflect(cgv::reflect::reflection_handler& rh)
{
	return 		
		rh.reflect_member("table_color", table_color) &&
		rh.reflect_member("table_width", table_width) &&
		rh.reflect_member("table_depth", table_depth) &&
		rh.reflect_member("table_height", table_height) &&
		rh.reflect_member("table_leg color", leg_color) &&
		rh.reflect_member("table_legs", leg_width) &&
		rh.reflect_member("table_offset", leg_offset);

}

void vr_scene::on_set(void* member_ptr)
{
	if (member_ptr == &scene_file_path) {
		cgv::utils::dir::glob(scene_file_path, scene_file_names, "scene_*.scn", false, true);
		if (scene_file_names.empty())
			current_scene_idx = 0;
		else
			current_scene_idx = (int)scene_file_names.size()-1;
		on_set(&current_scene_idx);
	}
	if (member_ptr == &current_scene_idx) {
		if (current_scene_idx < scene_file_names.size())
			read_scene(scene_file_path + "/" + scene_file_names[current_scene_idx]);
	}
	if (member_ptr >= &table_width && member_ptr < &leg_color + 1) {
		boxes.resize(boxes.size() - 5);
		box_colors.resize(box_colors.size() - 5);
		construct_table(table_width, table_depth, table_height, leg_width, leg_offset, table_color, leg_color);
	}
	update_member(member_ptr);
	post_redraw();
}

bool vr_scene::init(cgv::render::context& ctx)
{
	cgv::render::ref_sphere_renderer(ctx, 1);
	cgv::render::ref_box_renderer(ctx, 1);
	cgv::render::ref_rounded_cone_renderer(ctx, 1);
	cgv::gui::connect_vr_server(true);

	lm.init(ctx);
	cgv::media::font::font_ptr f = cgv::media::font::find_font("Courier New");
	lm.set_font_face(f->get_font_face(cgv::media::font::FFA_BOLD));
	lm.set_font_size(36.0f);
	lm.set_text_color(rgba(0, 0, 0, 1));

	for (int ci = 0; ci < 2; ++ci) {
		li_help[ci] = add_label("DPAD_Right .. next/new scene\nDPAD_Left  .. prev scene\nDPAD_Down  .. save scene",
			rgba(ci == 0 ? 0.8f : 0.4f, 0.4f, ci == 1 ? 0.8f : 0.4f, 1.0f));
		fix_label_size(li_help[ci]);		
		place_label(li_help[ci], vec3(ci==0?-0.05f:0.05f, 0.0f, 0.0f), quat(vec3(1, 0, 0), -1.5f), 
			ci == 0 ? CS_LEFT_CONTROLLER : CS_RIGHT_CONTROLLER, ci == 0 ? LA_RIGHT:LA_LEFT, 0.2f);
		hide_label(li_help[ci]);
	}

	li_stats = add_label(
		"scene index: 000000\n"
		"nr vertices: 000000\n"
		"nr edges:    000000", rgba(0.8f, 0.6f, 0.0f, 1.0f));
	fix_label_size(li_stats);
	place_label(li_stats, vec3(0.0f, 0.01f, 0.0f), quat(vec3(1, 0, 0), -1.5f), CS_TABLE);

	auto view_ptr = find_view_as_node();
	if (view_ptr) {
		// if the view points to a vr_view_interactor
		vr_view_ptr = dynamic_cast<vr_view_interactor*>(view_ptr);
		if (vr_view_ptr) {
			// configure vr event processing
			vr_view_ptr->set_event_type_flags(
				cgv::gui::VREventTypeFlags(
					cgv::gui::VRE_KEY +
					cgv::gui::VRE_ONE_AXIS +
					cgv::gui::VRE_ONE_AXIS_GENERATES_KEY +
					cgv::gui::VRE_TWO_AXES +
					cgv::gui::VRE_TWO_AXES_GENERATES_DPAD +
					cgv::gui::VRE_POSE
				));
			// vr_view_ptr->enable_vr_event_debugging(false);
			// configure vr rendering
			// vr_view_ptr->draw_action_zone(false);
			vr_view_ptr->draw_vr_kits(true);
			// vr_view_ptr->enable_blit_vr_views(true);
			// vr_view_ptr->set_blit_vr_view_width(200);
		}
	}
	return true;
}

void vr_scene::init_frame(cgv::render::context& ctx)
{
	bool repack = lm.is_packing_outofdate();
	std::stringstream ss;
	ss << "scene index: " << current_scene_idx << "\n"
		<< "nr vertices: " << nr_vertices << "\n"
		<< "nr edges:    " << nr_edges;
	ss.flush();
	update_label_text(li_stats, ss.str());
	lm.ensure_texture_uptodate(ctx);
	if (repack) {
		for (uint32_t li = 0; li < label_texture_ranges.size(); ++li)
			label_texture_ranges[li] = lm.get_texcoord_range(li);
	}

	// update visibility of visibility changing labels
	if (vr_view_ptr && vr_view_ptr->get_current_vr_state()) {
		vec3 view_dir = -reinterpret_cast<const vec3&>(vr_view_ptr->get_current_vr_state()->hmd.pose[6]);
		vec3 view_pos = reinterpret_cast<const vec3&>(vr_view_ptr->get_current_vr_state()->hmd.pose[9]);
		for (int ci = 0; ci < 2; ++ci) {
			vec3 controller_pos = reinterpret_cast<const vec3&>(vr_view_ptr->get_current_vr_state()->controller[ci].pose[9]);
			float controller_depth = dot(view_dir, controller_pos - view_pos);
			float controller_dist = (view_pos + controller_depth * view_dir - controller_pos).length();
			if (view_dir.y() < -0.5f && controller_depth / controller_dist > 5.0f)
				show_label(li_help[ci]);
			else
				hide_label(li_help[ci]);
		}
	}
}

void vr_scene::clear(cgv::render::context& ctx)
{
	cgv::render::ref_sphere_renderer(ctx,-1);
	cgv::render::ref_box_renderer(ctx,-1);
	cgv::render::ref_rounded_cone_renderer(ctx,-1);
	lm.destruct(ctx);
}

void vr_scene::draw(cgv::render::context& ctx)
{
	// activate render styles
	auto& sr = cgv::render::ref_sphere_renderer(ctx);
	sr.set_render_style(srs);
	auto& br = cgv::render::ref_box_renderer(ctx);
	br.set_render_style(style);
	auto& rcr = cgv::render::ref_rounded_cone_renderer(ctx);
	rcr.set_render_style(rcrs);
	auto& rr = cgv::render::ref_rectangle_renderer(ctx);
	rr.set_render_style(prs);

	// draw static part
	br.set_box_array(ctx, boxes);
	br.set_color_array(ctx, box_colors);
	br.render(ctx, 0, boxes.size());

	// draw vertex edge graph
	if (!vertices.empty()) {
		sr.set_position_array(ctx, &vertices.front().position, vertices.size(), sizeof(vertex));
		sr.set_radius_array(ctx, &vertices.front().radius, vertices.size(), sizeof(vertex));
		sr.set_color_array(ctx, &vertices.front().color, vertices.size(), sizeof(vertex));
		sr.render(ctx, 0, (GLsizei)vertices.size());
	}
	if (!edges.empty()) {
		rcr.set_position_array(ctx, &vertices.front().position, vertices.size(), sizeof(vertex));
		rcr.set_radius_array(ctx, &vertices.front().radius, vertices.size(), sizeof(vertex));
		rcr.set_color_array(ctx, &vertices.front().color, vertices.size(), sizeof(vertex));
		rcr.set_indices(ctx, &edges.front().origin_vi, 2 * edges.size());
		rcr.render(ctx, 0, (GLsizei)(2 * edges.size()));
	}

	// compute label poses in lab coordinate system
	std::vector<vec3> P;
	std::vector<quat> Q;
	std::vector<vec2> E;
	std::vector<vec4> T;
	mat34 pose[5];
	bool valid[5];
	valid[CS_LAB] = valid[CS_TABLE] = true;
	pose[CS_LAB].identity();
	pose[CS_TABLE].identity();
	cgv::math::pose_position(pose[CS_TABLE]) = vec3(0.0f, table_height, 0.0f);
	valid[CS_HEAD] = vr_view_ptr->get_current_vr_state() && vr_view_ptr->get_current_vr_state()->hmd.status == vr::VRS_TRACKED;
	if (valid[CS_HEAD])
		pose[CS_HEAD] = reinterpret_cast<const mat34&>(vr_view_ptr->get_current_vr_state()->hmd.pose[0]);
	valid[CS_LEFT_CONTROLLER] = vr_view_ptr->get_current_vr_state() && vr_view_ptr->get_current_vr_state()->controller[0].status == vr::VRS_TRACKED;
	if (valid[CS_LEFT_CONTROLLER])
		pose[CS_LEFT_CONTROLLER] = reinterpret_cast<const mat34&>(vr_view_ptr->get_current_vr_state()->controller[0].pose[0]);
	valid[CS_RIGHT_CONTROLLER] = vr_view_ptr->get_current_vr_state() && vr_view_ptr->get_current_vr_state()->controller[1].status == vr::VRS_TRACKED;
	if (valid[CS_RIGHT_CONTROLLER])
		pose[CS_RIGHT_CONTROLLER] = reinterpret_cast<const mat34&>(vr_view_ptr->get_current_vr_state()->controller[1].pose[0]);
	for (uint32_t li = 0; li < label_coord_systems.size(); ++li) {
		if (!label_visibilities[li] || !valid[label_coord_systems[li]])
			continue;
		mat34 label_pose = cgv::math::pose_construct(label_orientations[li], label_positions[li]);
		cgv::math::pose_transform(pose[label_coord_systems[li]], label_pose);
		P.push_back(cgv::math::pose_position(label_pose));
		Q.push_back(quat(cgv::math::pose_orientation(label_pose)));
		E.push_back(label_extents[li]);
		T.push_back(label_texture_ranges[li]);
	}
	// draw labels
	rr.set_position_array(ctx, P);
	rr.set_rotation_array(ctx, Q);
	rr.set_extent_array(ctx, E);
	rr.set_texcoord_array(ctx, T);
	lm.get_texture()->enable(ctx);
	rr.render(ctx, 0, P.size());
	lm.get_texture()->disable(ctx);
}

void vr_scene::clear_scene()
{
	vertices.clear();
	edges.clear();
	nr_vertices = 0;
	nr_edges = 0;
	update_member(&nr_vertices);
	update_member(&nr_edges);
}

void vr_scene::stream_help(std::ostream& os)
{
	os << "vr_scene: navigate scenes with direction pad left and right and save with down" << std::endl;
}

bool vr_scene::handle(cgv::gui::event& e)
{
	if ((e.get_flags() && cgv::gui::EF_VR) == 0)
		return false;
	if (e.get_kind() != cgv::gui::EID_KEY)
		return false;

	auto& ke = static_cast<cgv::gui::vr_key_event&>(e);
	if (ke.get_action() == cgv::gui::KA_RELEASE)
		return false;
	switch (ke.get_key()) {
	case vr::VR_DPAD_RIGHT:
		if (vertices.empty())
			return false;
		if (current_scene_idx >= scene_file_names.size())
			scene_file_names.push_back(get_new_scene_file_name());
		if (vertices.size() > 0)
			write_scene(scene_file_path + "/" + scene_file_names[current_scene_idx]);
		++current_scene_idx;
		if (current_scene_idx >= scene_file_names.size())
			clear_scene();
		else
			read_scene(scene_file_path + "/" + scene_file_names[current_scene_idx]);
		return true;
	case vr::VR_DPAD_LEFT:
		if (current_scene_idx > 0) {
			if (vertices.size() > 0) {
				if (current_scene_idx >= scene_file_names.size())
					scene_file_names.push_back(get_new_scene_file_name());
				write_scene(scene_file_path + "/" + scene_file_names[current_scene_idx]);
			}
			--current_scene_idx;
			read_scene(scene_file_path + "/" + scene_file_names[current_scene_idx]);
		}
		return true;
	case vr::VR_DPAD_DOWN:
		if (current_scene_idx >= scene_file_names.size())
			scene_file_names.push_back(get_new_scene_file_name());
		write_scene(scene_file_path+"/"+scene_file_names[current_scene_idx]);
		return true;
	}
	return false;
}

bool vr_scene::read_scene(const std::string& scene_file_name)
{
	std::ifstream is(scene_file_name);
	if (is.fail())
		return false;
	clear_scene();

	vertex V;
	edge E;
	while (!is.eof()) {
		char buffer[2048];
		is.getline(&buffer[0], 2048);
		std::string line(buffer);
		if (line.empty())
			continue;
		std::stringstream ss(line, std::ios_base::in);
		char c;
		ss.get(c);
		switch (toupper(c)) {
		case 'V':
			ss >> V.position >> V.radius >> V.color;
			if (!ss.fail())
				vertices.push_back(V);
			break;
		case 'E':
			ss >> E.origin_vi >> E.target_vi;
			if (!ss.fail())
				edges.push_back(E);
			break;
		}
	}

	nr_vertices = (uint32_t)vertices.size();
	nr_edges = (uint32_t)edges.size();
	update_member(&nr_vertices);
	update_member(&nr_edges);
	return !is.fail();
}

bool vr_scene::write_scene(const std::string& scene_file_name) const
{
	std::ofstream os(scene_file_name);
	if (os.fail())
		return false;
	for (size_t vi = 0; vi < vertices.size(); ++vi)
		os << "v " << vertices[vi].position << " " << vertices[vi].radius << " " << vertices[vi].color << std::endl;
	for (size_t ei = 0; ei < edges.size(); ++ei)
		os << "e " << edges[ei].origin_vi << " " << edges[ei].target_vi << std::endl;
	return true;
}

void vr_scene::create_gui()
{
	add_decorator("vr_scene", "heading");
	add_view("nr_vertices", nr_vertices);
	add_view("nr_edges", nr_edges);
	add_member_control(this, "scene_file_path", scene_file_path, "directory");
	//		"open=true;open_title='open scene file';filter='scene (scn):*.scn|all files:*.*';"
	//"save=true;save_title='save scene file';w=140");
	if (begin_tree_node("table", table_width)) {
		align("\a");
		add_member_control(this, "color", table_color);
		add_member_control(this, "width", table_width, "value_slider", "min=0.1;max=3.0;ticks=true");
		add_member_control(this, "depth", table_depth, "value_slider", "min=0.1;max=3.0;ticks=true");
		add_member_control(this, "height", table_height, "value_slider", "min=0.1;max=3.0;ticks=true");
		add_member_control(this, "leg color", leg_color);
		add_member_control(this, "legs", leg_width, "value_slider", "min=0.0;max=0.3;ticks=true");
		add_member_control(this, "offset", leg_offset, "value_slider", "min=0.0;max=0.5;ticks=true");
		align("\b");
		end_tree_node(table_width);
	}
	if (begin_tree_node("rendering", srs.material)) {
		align("\a");
		if (begin_tree_node("spheres", srs)) {
			align("\a");
			add_gui("spheres", srs);
			align("\b");
			end_tree_node(srs);
		}
		if (begin_tree_node("cones", rcrs)) {
			align("\a");
			add_gui("cones", rcrs);
			align("\b");
			end_tree_node(rcrs);
		}
		align("\b");
		end_tree_node(srs.material);
	}
}

#include <cgv/base/register.h>

cgv::base::object_registration<vr_scene> vr_scene_reg("vr_scene");
