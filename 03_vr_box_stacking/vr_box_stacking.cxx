#include "vr_box_stacking.h"
#include <cgv/defines/quote.h>
#include <cgv/signal/rebind.h>
#include <cgv/base/register.h>
#include <cgv/math/ftransform.h>
#include <cgv/utils/scan.h>
#include <cgv/utils/options.h>
#include <cgv/utils/file.h>
#include <cgv/gui/dialog.h>
#include <cgv/gui/file_dialog.h>
#include <cgv/render/attribute_array_binding.h>
#include <cgv_gl/sphere_renderer.h>
#include <cgv/media/mesh/simple_mesh.h>
#include <cg_vr/vr_events.h>
#include <cgv_gl/box_wire_renderer.h>

#include <random>
#include <sstream>

#include "intersection.h"

template <typename T>
double time_stamp(const T& t_start) {
	auto now = std::chrono::steady_clock::now();
	return static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(now - t_start).count()) / 1e6;
}

bool hasEnding(std::string const &str, std::string const &ending) {
	if (str.length() >= ending.length()) {
		return str.compare(str.length() - ending.length(), ending.length(), ending) == 0;
	}
	return false;
}

template <typename T>
T snapToGrid(T p, const float grid) {
	for (int i = 0; i < T::size(); ++i) {
		p[i] = round(p[i] / grid)*grid;
	}
	return p;
}

template <>
float snapToGrid<float>(float p, const float grid) {
	return round(p / grid)*grid;
}

cgv::render::render_types::vec4 quatToAxisAngle(const cgv::render::render_types::quat q) {
	typedef  cgv::render::render_types::vec4 vec4;
	float div = sqrt(1.f - q.w() * q.w());
	if (div == 0.f) {
		return vec4(1, 0, 0, 0);
	}
	return vec4(
		q.x() / div,
		q.y() / div,
		q.z() / div,
		2.f*acos(q.w())
	);
}

cgv::render::render_types::quat axisAngleToQuat(const cgv::render::render_types::vec4 aa) {
	typedef  cgv::render::render_types::quat quat;
	float s = sin(aa.w() * 0.5f);
	return quat(
		cos(aa.w() * 0.5f),
		aa.x() * s,
		aa.y() * s,
		aa.z() * s
	);
}


void vr_box_stacking::change_box_extents(Axis axis,int ci)
{
	for (size_t i = 0; i < intersection_points.size(); ++i) {
		if (intersection_controller_indices[i] != ci)
			continue;
		// extract box index
		int bi = intersection_box_indices[i];
		box3 b = movable_boxes[bi];
		float extent = b.get_max_pnt()[axis] - b.get_min_pnt()[axis];
		float center = b.get_center()[axis];

		const float size_limit = 0.5;
		float new_extent = std::max(std::fmod(extent + edit_box_step, size_limit), edit_box_step);
		movable_boxes[bi].ref_max_pnt()[axis] = center + new_extent * 0.5f;
		movable_boxes[bi].ref_min_pnt()[axis] = center - new_extent * 0.5f;
		// update intersection points
		//intersection_points[i] = rotation * (intersection_points[i] - last_pos) + pos;
		break; //change only the first box
	}
}

void vr_box_stacking::delete_box(int bi)
{
	movable_boxes.erase(movable_boxes.begin() + bi);
	movable_box_colors.erase(movable_box_colors.begin() + bi);
	movable_box_translations.erase(movable_box_translations.begin() + bi);
	movable_box_rotations.erase(movable_box_rotations.begin() + bi);
}

size_t vr_box_stacking::clear_intersections(int ci)
{
	size_t i = 0;
	while (i < intersection_points.size()) {
		if (intersection_controller_indices[i] == ci) {
			intersection_points.erase(intersection_points.begin() + i);
			intersection_colors.erase(intersection_colors.begin() + i);
			intersection_box_indices.erase(intersection_box_indices.begin() + i);
			intersection_controller_indices.erase(intersection_controller_indices.begin() + i);
		}
		else
			++i;
	}
	return i;
}

size_t vr_box_stacking::clear_highlighted_boxes(int ci)
{
	size_t i = 0;
	while (i < highlighted_boxes.size()) {
		if (highlighted_box_controler[i] == ci) {
			highlighted_boxes.erase(highlighted_boxes.begin() + i);
			highlighted_box_colors.erase(highlighted_box_colors.begin() + i);
			highlighted_box_translations.erase(highlighted_box_translations.begin() + i);
			highlighted_box_rotations.erase(highlighted_box_rotations.begin() + i);
			highlighted_box_controler.erase(highlighted_box_controler.begin() + i);
			highlighted_box_id.erase(highlighted_box_id.begin() + i);
		}
		else
			++i;
	}
	return i;
}

size_t vr_box_stacking::clear_highlighted_boxes(int ci, int bi)
{
	size_t i = 0;
	while (i < highlighted_boxes.size()) {
		if (highlighted_box_controler[i] == ci && highlighted_box_id[i] == bi) {
			highlighted_boxes.erase(highlighted_boxes.begin() + i);
			highlighted_box_colors.erase(highlighted_box_colors.begin() + i);
			highlighted_box_translations.erase(highlighted_box_translations.begin() + i);
			highlighted_box_rotations.erase(highlighted_box_rotations.begin() + i);
			highlighted_box_controler.erase(highlighted_box_controler.begin() + i);
			highlighted_box_id.erase(highlighted_box_id.begin() + i);
		}
		else
			++i;
	}
	return i;
}

float vr_box_stacking::quadratic_error(const box3 & a, const box3 & b, const vec3 & translation_a, 
		const vec3 & translation_b, const quat & rot_a, const quat & rot_b)
{
	std::vector<vec3> pnts_a(8), pnts_b(8);
	box_corners(a, translation_a, rot_a, pnts_a.data());
	box_corners(b, translation_b, rot_b, pnts_b.data());

	//find closest points
	float error = 0.f;
	for (int i = 0; i < 8; ++i) {
		float e = std::numeric_limits<float>::infinity();
		for (int j = 0; j < 8; ++j) {
			vec3 d = pnts_a[i] - pnts_b[j];
			e = std::min(d.sqr_length(), e);
		}
		error += e;
	}
	return error;
}

void vr_box_stacking::box_corners(const box3 & b, const vec3 & translation, const quat & rotation, vec3 * points)
{
	vec3& min_pnt = points[0] = b.get_min_pnt();
	vec3& max_pnt = points[7] = b.get_max_pnt();
	vec3 extent = b.get_extent();
	points[1] = vec3(max_pnt.x(), min_pnt.y(), min_pnt.z());
	points[2] = vec3(min_pnt.x(), max_pnt.y(), min_pnt.z());
	points[3] = vec3(max_pnt.x(), max_pnt.y(), min_pnt.z());
	points[4] = vec3(min_pnt.x(), min_pnt.y(), max_pnt.z());
	points[5] = vec3(max_pnt.x(), min_pnt.y(), max_pnt.z());
	points[6] = vec3(min_pnt.x(), max_pnt.y(), max_pnt.z());
	//transform points to global
	for (int i = 0; i < 8; ++i) {
		points[i] = rotation.get_rotated(points[i]) + translation;
	}
}

int vr_box_stacking::nearest_box_center(const vec3& p, const box3 * boxes, const int boxes_size, const int exclude)
{
	int nearest_box = -1;

	for (int i = 0; i < boxes_size;++i) {
		if (i == exclude) { continue; }
		else if (nearest_box == -1) {
			nearest_box = 1;
		}
		if ((boxes[i].get_center() - p).sqr_length() < (boxes[nearest_box].get_center() - p).sqr_length()) {
			nearest_box = i;
		}
	}
	return nearest_box;
}

/// compute intersection points of controller ray with movable boxes
void vr_box_stacking::compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color)
{
	for (size_t i = 0; i < movable_boxes.size(); ++i) {
		vec3 origin_box_i = origin - movable_box_translations[i];
		movable_box_rotations[i].inverse_rotate(origin_box_i);
		vec3 direction_box_i = direction;
		movable_box_rotations[i].inverse_rotate(direction_box_i);
		float t_result;
		vec3  p_result;
		vec3  n_result;
		if (cgv::media::ray_axis_aligned_box_intersection(
			origin_box_i, direction_box_i,
			movable_boxes[i],
			t_result, p_result, n_result, 0.000001f)) {

			// transform result back to world coordinates
			movable_box_rotations[i].rotate(p_result);
			p_result += movable_box_translations[i];
			movable_box_rotations[i].rotate(n_result);

			// store intersection information
			intersection_points.push_back(p_result);
			intersection_colors.push_back(color);
			intersection_box_indices.push_back((int)i);
			intersection_controller_indices.push_back(ci);
		}
	}
}

/// register on device change events
void vr_box_stacking::on_device_change(void* kit_handle, bool attach)
{
	if (attach) {
		if (last_kit_handle == 0) {
			vr::vr_kit* kit_ptr = vr::get_vr_kit(kit_handle);
			if (kit_ptr) {
				last_kit_handle = kit_handle;
				post_recreate_gui();
			}
		}
	}
	else {
		if (kit_handle == last_kit_handle) {
			last_kit_handle = 0;
			post_recreate_gui();
		}
	}
}

/// construct boxes that represent a table of dimensions tw,td,th and leg width tW
void vr_box_stacking::construct_table(float tw, float td, float th, float tW) {
	// construct table
	rgb table_clr(0.3f, 0.2f, 0.0f);
	boxes.push_back(box3(
		vec3(-0.5f*tw - 2 * tW, th - tW, -0.5f*td - 2 * tW),
		vec3(0.5f*tw + 2 * tW, th, 0.5f*td + 2 * tW)));
	box_colors.push_back(table_clr);

	boxes.push_back(box3(vec3(-0.5f*tw, 0, -0.5f*td), vec3(-0.5f*tw - tW, th - tW, -0.5f*td - tW)));
	boxes.push_back(box3(vec3(-0.5f*tw, 0, 0.5f*td), vec3(-0.5f*tw - tW, th - tW, 0.5f*td + tW)));
	boxes.push_back(box3(vec3(0.5f*tw, 0, -0.5f*td), vec3(0.5f*tw + tW, th - tW, -0.5f*td - tW)));
	boxes.push_back(box3(vec3(0.5f*tw, 0, 0.5f*td), vec3(0.5f*tw + tW, th - tW, 0.5f*td + tW)));
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
}

/// construct boxes that represent a room of dimensions w,d,h and wall width W
void vr_box_stacking::construct_room(float w, float d, float h, float W, bool walls, bool ceiling) {
	// construct floor
	boxes.push_back(box3(vec3(-0.5f*w, -W, -0.5f*d), vec3(0.5f*w, 0, 0.5f*d)));
	box_colors.push_back(rgb(0.2f, 0.2f, 0.2f));

	if(walls) {
		// construct walls
		boxes.push_back(box3(vec3(-0.5f*w, -W, -0.5f*d - W), vec3(0.5f*w, h, -0.5f*d)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));
		boxes.push_back(box3(vec3(-0.5f*w, -W, 0.5f*d), vec3(0.5f*w, h, 0.5f*d + W)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));

		boxes.push_back(box3(vec3(0.5f*w, -W, -0.5f*d - W), vec3(0.5f*w + W, h, 0.5f*d + W)));
		box_colors.push_back(rgb(0.5f, 0.8f, 0.5f));
	}
	if(ceiling) {
		// construct ceiling
		boxes.push_back(box3(vec3(-0.5f*w - W, h, -0.5f*d - W), vec3(0.5f*w + W, h + W, 0.5f*d + W)));
		box_colors.push_back(rgb(0.5f, 0.5f, 0.8f));
	}
}

/// construct boxes for environment
void vr_box_stacking::construct_environment(float s, float ew, float ed, float w, float d, float h) {
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	unsigned n = unsigned(ew / s);
	unsigned m = unsigned(ed / s);
	float ox = 0.5f*float(n)*s;
	float oz = 0.5f*float(m)*s;
	for(unsigned i = 0; i < n; ++i) {
		float x = i * s - ox;
		for(unsigned j = 0; j < m; ++j) {
			float z = j * s - oz;
			if(fabsf(x) < 0.5f*w && fabsf(x + s) < 0.5f*w && fabsf(z) < 0.5f*d && fabsf(z + s) < 0.5f*d)
				continue;
			float h = 0.2f*(std::max(abs(x) - 0.5f*w, 0.0f) + std::max(abs(z) - 0.5f*d, 0.0f))*distribution(generator) + 0.1f;
			boxes.push_back(box3(vec3(x, 0.0f, z), vec3(x + s, h, z + s)));
			rgb color = cgv::media::color<float, cgv::media::HLS>(distribution(generator), 0.1f*distribution(generator) + 0.15f, 0.3f);
			box_colors.push_back(color);
			/*box_colors.push_back(
				rgb(0.3f*distribution(generator) + 0.3f,
					0.3f*distribution(generator) + 0.2f,
					0.2f*distribution(generator) + 0.1f));*/
		}
	}
}

/// construct boxes that can be moved around
void vr_box_stacking::construct_movable_boxes(float tw, float td, float th, float tW, size_t nr) {
	/*
	vec3 extent(0.75f, 0.5f, 0.05f);
	movable_boxes.push_back(box3(-0.5f * extent, 0.5f * extent));
	movable_box_colors.push_back(rgb(0, 0, 0));
	movable_box_translations.push_back(vec3(0, 1.2f, 0));
	movable_box_rotations.push_back(quat(1, 0, 0, 0));
	*/
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	std::uniform_real_distribution<float> signed_distribution(-1, 1);
	for(size_t i = 0; i < nr; ++i) {
		float x = distribution(generator);
		float y = distribution(generator);
		vec3 extent(distribution(generator), distribution(generator), distribution(generator));
		extent += 0.01f;
		extent *= std::min(tw, td)*0.1f;

		vec3 center(-0.5f*tw + x * tw, th + tW, -0.5f*td + y * td);
		movable_boxes.push_back(box3(-0.5f*extent, 0.5f*extent));
		movable_box_colors.push_back(rgb(distribution(generator), distribution(generator), distribution(generator)));
		movable_box_translations.push_back(center);
		quat rot(signed_distribution(generator), signed_distribution(generator), signed_distribution(generator), signed_distribution(generator));
		rot.normalize();
		movable_box_rotations.push_back(rot);
	}
}

/// construct a scene with a table
void vr_box_stacking::build_scene(float w, float d, float h, float W, float tw, float td, float th, float tW)
{
	construct_room(w, d, h, W, false, false);
	construct_table(tw, td, th, tW);
	construct_environment(0.3f, 3 * w, 3 * d, w, d, h);
	//construct_environment(0.4f, 0.5f, 1u, w, d, h);
	construct_movable_boxes(tw, td, th, tW, 20);
}

vr_box_stacking::vr_box_stacking() 
{
	set_name("vr_box_stacking");
	build_scene(5, 7, 3, 0.2f, 1.6f, 0.8f, 0.7f, 0.03f);
	vr_view_ptr = 0;
	ray_length = 2;
	last_kit_handle = 0;
	connect(cgv::gui::ref_vr_server().on_device_change, this, &vr_box_stacking::on_device_change);

	srs.radius = 0.005f;

	vr_events_stream = nullptr;
	box_trajectory_stream = nullptr;
	controller_trajectory_stream = nullptr;
	box_edit_mode = true;
	for (auto& a : grab_number) a = 0;
	new_box = box3(vec3(-0.05f), vec3(0.05f));
	new_box_color = rgb(88.f / 255.f, 24.f / 255.f, 69.f / 255.f);
	edit_box_selected_axis = 0;
	edit_box_step = 0.025f;
	edit_box_max_size = 0.2f;
	new_box_distance = 0.35f;
	snap_box_to_grid = false;
	grid_step = 0.025f;
	rotation_grid_step = 0.5;
	show_message_t = false;

	label_outofdate = true;
	label_text = "Info Board";
	label_font_idx = 0;
	label_upright = true;
	label_face_type = cgv::media::font::FFA_BOLD;
	label_resolution = 256;
	label_size = 20.0f;
	label_color = rgb(1, 1, 1);

	cgv::media::font::enumerate_font_names(font_names);
	font_enum_decl = "enums='";
	for (unsigned i = 0; i < font_names.size(); ++i) {
		if (i>0)
			font_enum_decl += ";";
		std::string fn(font_names[i]);
		if (cgv::utils::to_lower(fn) == "calibri") {
			label_font_face = cgv::media::font::find_font(fn)->get_font_face(label_face_type);
			label_font_idx = i;
		}
		font_enum_decl += std::string(fn);
	}
	font_enum_decl += "'";
	state[0] = state[1] = state[2] = state[3] = IS_NONE;

	read_configuration(1);
}
	
void vr_box_stacking::stream_help(std::ostream& os) 
{
	os << "vr_box_stacking: no shortcuts defined" << std::endl;
}
	
void vr_box_stacking::on_set(void* member_ptr)
{
	if (member_ptr == &label_face_type || member_ptr == &label_font_idx) {
		label_font_face = cgv::media::font::find_font(font_names[label_font_idx])->get_font_face(label_face_type);
		label_outofdate = true;
	}
	if ((member_ptr >= &label_color && member_ptr < &label_color + 1) ||
		member_ptr == &label_size || member_ptr == &label_text) {
		label_outofdate = true;
	}
	if (member_ptr == &log_vr_events && vr_events_stream) {
		if (!log_vr_events) { //close file
			vr_events_stream->close();
			box_trajectory_stream->close();
			controller_trajectory_stream->close();
			vr_events_stream = nullptr;
			box_trajectory_stream = nullptr;
			controller_trajectory_stream = nullptr;
			vr_events_record_path = "";
		}
		else { //start timer
			vrr_t_start = std::chrono::steady_clock::now();
		}
	}
	update_member(member_ptr);
	post_redraw();
}
	
bool vr_box_stacking::handle(cgv::gui::event& e)
{
	// check if vr event flag is not set and don't process events in this case
	if ((e.get_flags() & cgv::gui::EF_VR) == 0)
		return false;

	// record controller events
	if (log_vr_events && vr_events_stream) {
		auto now = std::chrono::steady_clock::now();
		double t = time_stamp(vrr_t_start);
		//double t = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(now - vrr_t_start).count()) / 1e6;
		*vr_events_stream << t << " \""; //timestamp
		e.stream_out(*vr_events_stream); //raw data
		*vr_events_stream << "\"\n";
	}

	// check event id
	switch (e.get_kind()) {
	case cgv::gui::EID_KEY:
	{
		cgv::gui::vr_key_event& vrke = static_cast<cgv::gui::vr_key_event&>(e);
		if (vrke.get_action() != cgv::gui::KA_RELEASE) {
			switch (vrke.get_key()) {
			case vr::VR_GRIP:
			{
				//set grabed box as template for new boxes
				int ci = vrke.get_controller_index();
				if (box_edit_mode && state[ci] == IS_OVER || state[ci] == IS_GRAB) {
					// iterate intersection points of current controller
					for (size_t i = 0; i < intersection_points.size(); ++i) {
						if (intersection_controller_indices[i] != ci)
							continue;
						// extract box index
						int bi = intersection_box_indices[i];
						int axis = edit_box_selected_axis;
						new_box = movable_boxes[bi];
						new_box_color = movable_box_colors[bi];
						label_outofdate = true;
						break; //get only the first box
					}
					post_redraw();
				}
				return true;
			}
			}
		}
		break;
	}
	case cgv::gui::EID_THROTTLE:
	{
		cgv::gui::vr_throttle_event& vrte = static_cast<cgv::gui::vr_throttle_event&>(e);
		std::cout << "throttle " << vrte.get_throttle_index() << " of controller " << vrte.get_controller_index()
			<< " adjusted from " << vrte.get_last_value() << " to " << vrte.get_value() << std::endl;
		if (vrte.get_value() == 1.0f) {
			if (box_edit_mode) {
				//create a new box
				int ci = vrte.get_controller_index();
				if (state[ci] == IS_NONE || state[ci] == IS_OVER) {
					const vec3 up = vec3(0.f, 1.f, 0.f);
					vec3 origin, direction;
					vrte.get_state().controller[ci].put_ray(&origin(0), &direction(0));
					movable_box_colors.emplace_back(new_box_color);
					movable_box_rotations.emplace_back(quat(cross(up, direction), acos(dot(up, direction))));
					movable_box_translations.emplace_back(origin + direction * new_box_distance);
					movable_boxes.emplace_back(new_box);
					post_redraw();
				}
				else if (state[ci] == IS_GRAB) {
					edit_box_selected_axis = (edit_box_selected_axis + 1) % 3;
				}
				return true;
			}
		}
		return true;
	}
	case cgv::gui::EID_STICK:
	{
		cgv::gui::vr_stick_event& vrse = static_cast<cgv::gui::vr_stick_event&>(e);
		switch (vrse.get_action()) {
		case cgv::gui::SA_TOUCH:
			if (state[vrse.get_controller_index()] == IS_OVER)
				state[vrse.get_controller_index()] = IS_GRAB;
			break;
		case cgv::gui::SA_RELEASE:
			if (state[vrse.get_controller_index()] == IS_GRAB) {
				state[vrse.get_controller_index()] = IS_OVER;
				++grab_number[vrse.get_controller_index()];
			}
			break;
		case cgv::gui::SA_PRESS:
		{
			int ci = vrse.get_controller_index();
			if (box_edit_mode && vrse.get_x() < -0.5f && state[ci] == IS_GRAB) {
				//change box extents
				change_box_extents(AXIS_X, ci);
				post_redraw();
				return true;
			}
			if (box_edit_mode && vrse.get_y() < -0.5f && state[ci] == IS_GRAB) {
				//change box extents
				change_box_extents(AXIS_Y, ci);
				post_redraw();
				return true;
			}
			if (box_edit_mode && vrse.get_x() > 0.5f && state[ci] == IS_GRAB) {
				//change box extents
				change_box_extents(AXIS_Z, ci);
				post_redraw();
				return true;
			}
			if (box_edit_mode && vrse.get_y() > 0.5f && state[ci] == IS_GRAB) {
				//delete box
				for (size_t i = 0; i < intersection_points.size(); ++i) {
					if (intersection_controller_indices[i] != ci)
						continue;
					// extract box index
					delete_box(intersection_box_indices[i]);
					break; //delete only the first box
				}
				// clear intersections of current controller 
				size_t i = clear_intersections(ci);
				// compute intersections
				vec3 origin, direction;
				vrse.get_state().controller[ci].put_ray(&origin(0), &direction(0));
				compute_intersections(origin, direction, ci, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
				label_outofdate = true;


				// update state based on whether we have found at least 
				// one intersection with controller ray
				if (intersection_points.size() == i)
					state[ci] = IS_NONE;
				else
					if (state[ci] == IS_NONE)
						state[ci] = IS_OVER;

				post_redraw();
				return true;
			}
		}
		}
		return true;
	}
	case cgv::gui::EID_POSE:
		cgv::gui::vr_pose_event& vrpe = static_cast<cgv::gui::vr_pose_event&>(e);
		// check for controller pose events
		int ci = vrpe.get_trackable_index();
		if (ci != -1) {
			if (state[ci] == IS_GRAB) {
				// in grab mode apply relative transformation to grabbed boxes

				// get previous and current controller position
				vec3 last_pos = vrpe.get_last_position();
				vec3 pos = vrpe.get_position();
				// get rotation from previous to current orientation
				// this is the current orientation matrix times the
				// inverse (or transpose) of last orientation matrix:
				// vrpe.get_orientation()*transpose(vrpe.get_last_orientation())
				mat3 rotation = vrpe.get_rotation_matrix();
				// iterate intersection points of current controller
				for (size_t i = 0; i < intersection_points.size(); ++i) {
					if (intersection_controller_indices[i] != ci)
						continue;
					// extract box index
					unsigned bi = intersection_box_indices[i];

					if (snap_box_to_grid) {
						if (intersection_grab_initialized[i] == 0) {
							intersection_grab_translations[i] = movable_box_translations[bi];
							intersection_grab_rotations[i] = movable_box_rotations[bi];
							intersection_grab_initialized[i] = 1;
						}
						intersection_grab_translations[i] = rotation * (intersection_grab_translations[i] - last_pos) + pos;
						movable_box_translations[bi] = snapToGrid(intersection_grab_translations[i], grid_step);
						intersection_grab_rotations[i] = quat(rotation) * intersection_grab_rotations[i];
						vec4 r = quatToAxisAngle(intersection_grab_rotations[i]);
						vec3 ax = snapToGrid(vec3(r.x(), r.y(), r.z()), rotation_grid_step);
						float angle = snapToGrid(r.w(), M_PI / 8.f); //TODO add slider to for this
						movable_box_rotations[bi] = axisAngleToQuat(vec4(ax.x(), ax.y(), ax.z(), angle));
						movable_box_rotations[bi].normalize();
					}
					else {
						// update translation with position change and rotation
						movable_box_translations[bi] = rotation * (movable_box_translations[bi] - last_pos) + pos;
						// update orientation with rotation, note that quaternions
						// need to be multiplied in oposite order. In case of matrices
						// one would write box_orientation_matrix *= rotation
						movable_box_rotations[bi] = quat(rotation) * movable_box_rotations[bi];
						if (judge_box_target(bi)) {
							message_t = "the box " + std::to_string(bi) + " is finished!";
							show_message_t = true;
						}
					}

					// update intersection points
					intersection_points[i] = rotation * (intersection_points[i] - last_pos) + pos;

					// highlight box
					if (frame_boxes.size() > 0 && bi < frame_boxes.size()) {
						float qerr = quadratic_error(
							movable_boxes[bi], frame_boxes[bi],
							movable_box_translations[bi], frame_box_translations[bi],
							movable_box_rotations[bi], frame_box_rotations[bi]);
						if (qerr < 0.004f) {
							clear_highlighted_boxes(ci, bi);
							int h = 0;
							while (h < highlighted_boxes.size() && highlighted_box_id[h] != bi && highlighted_box_controler[h] != ci) ++h;
							const rgba highlighted_box_color = rgba(0.1, 0.95, 0.35, 0.8);
							if (h == highlighted_boxes.size()){
								highlighted_boxes.push_back(box3(movable_boxes[bi].get_min_pnt()*1.05, movable_boxes[bi].get_max_pnt()*1.05));
								highlighted_box_colors.push_back(highlighted_box_color);
								highlighted_box_translations.push_back(movable_box_translations[bi]);
								highlighted_box_rotations.push_back(movable_box_rotations[bi]);
								highlighted_box_controler.push_back(ci);
								highlighted_box_id.push_back(bi);
							}
							else {
								highlighted_box_colors[h] = highlighted_box_color;
								highlighted_box_translations[h] = movable_box_translations[bi];
								highlighted_box_rotations[h] = movable_box_rotations[bi];
							}

						}
						else {
							clear_highlighted_boxes(ci, bi);
						}
					}

					if (log_vr_events && box_trajectory_stream) {
						//<box-index> <controller-id> <grab-id> <box-translation> <box-rotation>
						*box_trajectory_stream
							<< ci << " "
							<< grab_number[ci] << " "
							<< time_stamp(vrr_t_start) << " "
							<< bi << " "
							<< movable_box_translations[bi] << " "
							<< movable_box_rotations[bi] << '\n';
					}
				}
			}
			else {// not grab
				size_t i = clear_intersections(ci);
				clear_highlighted_boxes(ci);

				// compute intersections
				vec3 origin, direction;
				vrpe.get_state().controller[ci].put_ray(&origin(0), &direction(0));
				compute_intersections(origin, direction, ci, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
				label_outofdate = true;


				// update state based on whether we have found at least 
				// one intersection with controller ray
				if (intersection_points.size() == i)
					state[ci] = IS_NONE;
				else
					if (state[ci] == IS_NONE)
						state[ci] = IS_OVER;
			}
			//log controller trajectory
			if (log_vr_events && controller_trajectory_stream) {
				mat3 rotation = vrpe.get_rotation_matrix();
				*controller_trajectory_stream
					<< ci << " "
					<< ((state[ci] == IS_GRAB) ? grab_number[ci] : -1) << " "
					<< time_stamp(vrr_t_start) << " "
					<< vrpe.get_position() << " "
					<< quat(vrpe.get_orientation()) << '\n';
			}
			post_redraw();
		}
		return true;
	}
	return false;
}

bool vr_box_stacking::init(cgv::render::context& ctx)
{
	if (!cgv::utils::has_option("NO_OPENVR"))
		ctx.set_gamma(1.0f);

	cgv::gui::connect_vr_server(true);

	auto view_ptr = find_view_as_node();
	if (view_ptr) {
		view_ptr->set_eye_keep_view_angle(dvec3(0, 4, -4));
		// if the view points to a vr_view_interactor
		vr_view_ptr = dynamic_cast<vr_view_interactor*>(view_ptr);
		if (vr_view_ptr) {
			// configure vr event processing
			vr_view_ptr->set_event_type_flags(
				cgv::gui::VREventTypeFlags(
					cgv::gui::VRE_KEY +
					cgv::gui::VRE_ONE_AXIS +
					cgv::gui::VRE_TWO_AXES +
					cgv::gui::VRE_TWO_AXES_GENERATES_DPAD +
					cgv::gui::VRE_POSE
				));
			vr_view_ptr->enable_vr_event_debugging(false);
			// configure vr rendering
			vr_view_ptr->draw_action_zone(false);
			vr_view_ptr->draw_vr_kits(true);
			vr_view_ptr->enable_blit_vr_views(true);
			vr_view_ptr->set_blit_vr_view_width(200);
		}
	}

	cgv::render::ref_box_renderer(ctx, 1);
	cgv::render::ref_sphere_renderer(ctx, 1);
	cgv::render::ref_rounded_cone_renderer(ctx, 1);
	cgv::render::ref_box_wire_renderer(ctx, 1);
	return true;
}

void vr_box_stacking::clear(cgv::render::context& ctx)
{
	cgv::render::ref_box_renderer(ctx, -1);
	cgv::render::ref_sphere_renderer(ctx, -1);
	cgv::render::ref_rounded_cone_renderer(ctx, -1);
	cgv::render::ref_box_wire_renderer(ctx, -1);
}

void vr_box_stacking::init_frame(cgv::render::context& ctx)
{
	if (label_fbo.get_width() != label_resolution) {
		label_tex.destruct(ctx);
		label_fbo.destruct(ctx);
	}
	if (!label_fbo.is_created()) {
		label_tex.create(ctx, cgv::render::TT_2D, label_resolution, label_resolution);
		label_fbo.create(ctx, label_resolution, label_resolution);
		label_tex.set_min_filter(cgv::render::TF_LINEAR_MIPMAP_LINEAR);
		label_tex.set_mag_filter(cgv::render::TF_LINEAR);
		label_fbo.attach(ctx, label_tex);
		label_outofdate = true;
	}
	if (label_outofdate && label_fbo.is_complete(ctx)) {
		glPushAttrib(GL_COLOR_BUFFER_BIT);
		label_fbo.enable(ctx);
		label_fbo.push_viewport(ctx);
		ctx.push_pixel_coords();
			glClearColor(0.5f,0.5f,0.5f,1.0f);
			glClear(GL_COLOR_BUFFER_BIT);

			glColor4f(label_color[0], label_color[1], label_color[2], 1);
			ctx.set_cursor(20, (int)ceil(label_size) + 20);
			ctx.enable_font_face(label_font_face, label_size);
			ctx.output_stream() << label_text << "\n";
			ctx.output_stream().flush(); // make sure to flush the stream before change of font size or font face

			ctx.enable_font_face(label_font_face, 0.7f*label_size);
			
			if (box_edit_mode) {
				const char axis[] = "XYZ";
				ctx.output_stream() << "new box[Trigger] \nextent=" << new_box.get_extent() << "\ncolor=" << new_box_color << '\n';
				ctx.output_stream() << "set as box template[Grip Button]\n";
				ctx.output_stream() << "delete box[trackpad up]\n";
				ctx.output_stream() << "change box extents[trackpad left,right,down]\n";
			}
			
			for (size_t i = 0; i < intersection_points.size(); ++i) {
				ctx.output_stream()
					<< "box " << intersection_box_indices[i]
					<< " at (" << intersection_points[i]
					<< ") with controller " << intersection_controller_indices[i] << "\n";
			}

			if(show_message_t)
				ctx.output_stream() << message_t << "\n";
			ctx.output_stream().flush();

		ctx.pop_pixel_coords();
		label_fbo.pop_viewport(ctx);
		label_fbo.disable(ctx);
		glPopAttrib();
		label_outofdate = false;

		label_tex.generate_mipmaps(ctx);
	}
}

void vr_box_stacking::draw(cgv::render::context& ctx)
{
	if (vr_view_ptr) {
		if (vr_view_ptr) {
			std::vector<vec3> P;
			std::vector<float> R;
			std::vector<rgb> C;
			const vr::vr_kit_state* state_ptr = vr_view_ptr->get_current_vr_state();
			if (state_ptr) {
				for (int ci = 0; ci < 4; ++ci) if (state_ptr->controller[ci].status == vr::VRS_TRACKED) {
					vec3 ray_origin, ray_direction;
					state_ptr->controller[ci].put_ray(&ray_origin(0), &ray_direction(0));
					P.push_back(ray_origin);
					R.push_back(0.002f);
					P.push_back(ray_origin + ray_length * ray_direction);
					R.push_back(0.003f);
					rgb c(float(1 - ci), 0.5f * (int)state[ci], float(ci));
					C.push_back(c);
					C.push_back(c);
				}
			}
			if (P.size() > 0) {
				auto& cr = cgv::render::ref_rounded_cone_renderer(ctx);
				cr.set_render_style(cone_style);
				//cr.set_eye_position(vr_view_ptr->get_eye_of_kit());
				cr.set_position_array(ctx, P);
				cr.set_color_array(ctx, C);
				cr.set_radius_array(ctx, R);
				if (!cr.render(ctx, 0, P.size())) {
					cgv::render::shader_program& prog = ctx.ref_default_shader_program();
					int pi = prog.get_position_index();
					int ci = prog.get_color_index();
					cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
					cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
					cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ci, C);
					cgv::render::attribute_array_binding::enable_global_array(ctx, ci);
					glLineWidth(3);
					prog.enable(ctx);
					glDrawArrays(GL_LINES, 0, (GLsizei)P.size());
					prog.disable(ctx);
					cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
					cgv::render::attribute_array_binding::disable_global_array(ctx, ci);
					glLineWidth(1);
				}
			}
		}
	}
	//cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
	cgv::render::box_wire_renderer& bwr = cgv::render::ref_box_wire_renderer(ctx);
	// draw wireframe boxes
	if (frame_boxes.size() > 0) { //pointing the renderer to uninitialized arrays causes assertions in debug mode
		bwr.set_render_style(wire_frame_style);
		bwr.set_box_array(ctx, frame_boxes);
		bwr.set_color_array(ctx, frame_box_colors);
		bwr.set_translation_array(ctx, frame_box_translations);
		bwr.set_rotation_array(ctx, frame_box_rotations);
		if (bwr.validate_and_enable(ctx)) {
			bwr.draw(ctx, 0, frame_boxes.size());
		}
		bwr.disable(ctx);
	}

	cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);

	// draw dynamic boxes 
	renderer.set_render_style(movable_style);
	renderer.set_box_array(ctx, movable_boxes);
	renderer.set_color_array(ctx, movable_box_colors);
	renderer.set_translation_array(ctx, movable_box_translations);
	renderer.set_rotation_array(ctx, movable_box_rotations);
	renderer.render(ctx, 0, movable_boxes.size());

	// draw static boxes
	renderer.set_render_style(style);
	renderer.set_box_array(ctx, boxes);
	renderer.set_color_array(ctx, box_colors);
	renderer.render(ctx, 0, boxes.size());

	// draw intersection points
	if (!intersection_points.empty()) {
		auto& sr = cgv::render::ref_sphere_renderer(ctx);
		sr.set_position_array(ctx, intersection_points);
		sr.set_color_array(ctx, intersection_colors);
		sr.set_render_style(srs);
		sr.render(ctx, 0, intersection_points.size());
	}

	// draw label
	if (vr_view_ptr && label_tex.is_created()) {
		cgv::render::shader_program& prog = ctx.ref_default_shader_program(true);
		int pi = prog.get_position_index();
		int ti = prog.get_texcoord_index();
		vec3 p(0, 1.5f, 0);
		vec3 y = label_upright ? vec3(0, 1.0f, 0) : normalize(vr_view_ptr->get_view_up_dir_of_kit());
		vec3 x = normalize(cross(vec3(vr_view_ptr->get_view_dir_of_kit()), y));
		float w = 0.5f, h = 0.5f;
		std::vector<vec3> P;
		std::vector<vec2> T;
		P.push_back(p - 0.5f * w * x - 0.5f * h * y); T.push_back(vec2(0.0f, 0.0f));
		P.push_back(p + 0.5f * w * x - 0.5f * h * y); T.push_back(vec2(1.0f, 0.0f));
		P.push_back(p - 0.5f * w * x + 0.5f * h * y); T.push_back(vec2(0.0f, 1.0f));
		P.push_back(p + 0.5f * w * x + 0.5f * h * y); T.push_back(vec2(1.0f, 1.0f));
		cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
		cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
		cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ti, T);
		cgv::render::attribute_array_binding::enable_global_array(ctx, ti);
		prog.enable(ctx);
		label_tex.enable(ctx);
		ctx.set_color(rgb(1, 1, 1));
		glDrawArrays(GL_TRIANGLE_STRIP, 0, (GLsizei)P.size());
		label_tex.disable(ctx);
		prog.disable(ctx);
		cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
		cgv::render::attribute_array_binding::disable_global_array(ctx, ti);
	}

	//draw box highlighting
	if (highlighted_boxes.size() > 0) {
		renderer.set_render_style(movable_style);
		renderer.set_box_array(ctx, highlighted_boxes);
		renderer.set_color_array(ctx, highlighted_box_colors);
		renderer.set_translation_array(ctx, highlighted_box_translations);
		renderer.set_rotation_array(ctx, highlighted_box_rotations);
		renderer.render(ctx, 0, highlighted_boxes.size());
	}
}

void vr_box_stacking::create_gui() 
{
	add_decorator("vr_box_stacking", "heading", "level=2");
	add_member_control(this, "ray_length", ray_length, "value_slider", "min=0.1;max=10;log=true;ticks=true");
	if (begin_tree_node("movable boxes", box_edit_mode)) {
		align("\a");
		connect_copy(add_button("save boxes")->click, rebind(this, &vr_box_stacking::on_save_movable_boxes_cb));
		connect_copy(add_button("load boxes")->click, rebind(this, &vr_box_stacking::on_load_movable_boxes_cb));
		connect_copy(add_button("load target")->click, rebind(this, &vr_box_stacking::on_load_wireframe_boxes_cb));
		add_member_control(this, "allow editing/creating boxes", box_edit_mode, "toggle");
		add_member_control(this, "max. box size", edit_box_max_size, "value_slider", "min=0;max=1;ticks=true");
		add_member_control(this, "edit step ize", edit_box_step, "value_slider", "min=0;max=1;ticks=true");
		add_member_control(this, "snap box to grid", snap_box_to_grid, "toggle");
		add_member_control(this, "grid step size", grid_step, "value_slider", "min=0.01;max=0.2;ticks=true");
		add_member_control(this, "rotation grid step", rotation_grid_step, "value_slider", "min=0.01;max=0.5;ticks=true");
		end_tree_node(box_edit_mode);
		align("\b");
	}
	if (begin_tree_node("VR events", log_vr_events)) {
		align("\a");
		add_member_control(this, "log vr events", log_vr_events, "toggle");
		connect_copy(add_button("select protocol file")->click, rebind(this, &vr_box_stacking::on_set_vr_event_streaming_file));
		add_view("protocol file", vr_events_record_path);
		end_tree_node(log_vr_events);
		align("\b");
	}
	if (begin_tree_node("box style", style)) {
		align("\a");
		add_gui("box style", style);
		align("\b");
		end_tree_node(style);
	}
	if (begin_tree_node("cone style", cone_style)) {
		align("\a");
		add_gui("cone style", cone_style);
		align("\b");
		end_tree_node(cone_style);
	}
	if(begin_tree_node("movable box style", movable_style)) {
		align("\a");
		add_gui("movable box style", movable_style);
		align("\b");
		end_tree_node(movable_style);
	}
	if(begin_tree_node("intersections", srs)) {
		align("\a");
		add_gui("sphere style", srs);
		align("\b");
		end_tree_node(srs);
	}
	if(begin_tree_node("label", label_size)) {
		align("\a");
		add_member_control(this, "text", label_text);
		add_member_control(this, "upright", label_upright);
		add_member_control(this, "font", (cgv::type::DummyEnum&)label_font_idx, "dropdown", font_enum_decl);
		add_member_control(this, "face", (cgv::type::DummyEnum&)label_face_type, "dropdown", "enums='regular,bold,italics,bold+italics'");
		add_member_control(this, "size", label_size, "value_slider", "min=8;max=64;ticks=true");
		add_member_control(this, "color", label_color);
		add_member_control(this, "resolution", (cgv::type::DummyEnum&)label_resolution, "dropdown", "enums='256=256,512=512,1024=1024,2048=2048'");
		align("\b");
		end_tree_node(label_size);
	}
}

bool vr_box_stacking::self_reflect(cgv::reflect::reflection_handler & rh)
{
	return	rh.reflect_member("vr_events_record_path", vr_events_record_path) && 
			rh.reflect_member("box_edit_mode", box_edit_mode);
}

bool vr_box_stacking::save_boxes(const std::string fn, const std::vector<box3>& boxes, const std::vector<rgb>& box_colors, const std::vector<vec3>& box_translations, const std::vector<quat>& box_rotations)
{
	std::stringstream data;


	if (boxes.size() != box_colors.size() || boxes.size() != box_translations.size() || boxes.size() != box_rotations.size()) {
		std::cerr << "vr_box_stacking::save_boxes: passed vectors have different sizes!";
		return false;
	}

	for (size_t i = 0; i < movable_boxes.size(); ++i) {
		//format: BOX <box.min_p> <box.max_p> <trans> <rot> <col>
		const vec3& box_translation = box_translations[i];
		const quat& box_rotation = box_rotations[i];
		const rgb& box_color = box_colors[i];
		data << "BOX "
			<< boxes[i].get_min_pnt() << " "
			<< boxes[i].get_max_pnt() << " "
			<< box_translation << " "
			<< box_rotation << " "
			<< box_color << " "
			<< '\n';
	}
	std::string s = data.str();
	if (!cgv::utils::file::write(fn, s.data(), s.size())) {
		std::cerr << "vr_box_stacking::save_boxes: failed writing data to file: " << fn;
	}
	return true;
}

bool vr_box_stacking::load_boxes(const std::string fn, std::vector<box3>& boxes, std::vector<rgb>& box_colors, std::vector<vec3>& box_translations, std::vector<quat>& box_rotations)
{
	std::string data;
	if (!cgv::utils::file::read(fn, data)) {
		std::cerr << "vr_box_stacking::load_boxes: failed reading data from file: " << fn << '\n';
		return false;
	}
	std::istringstream f(data);
	std::string line;

	while (!f.eof()) {
		std::getline(f, line); 	//read a line
		std::istringstream l(line);
		std::string sym;

		int limit = 1;
		bool valid = true;
		if (!l.eof()) {
			getline(l, sym, ' '); //get the first symbol determing the type
			if (sym == "BOX") { //in case of a box
				vec3 minp, maxp, trans;
				quat rot;
				rgb col;
				l >> minp;
				l >> maxp;
				l >> trans;
				l >> rot;
				l >> col;

				boxes.emplace_back(minp, maxp);
				box_translations.emplace_back(trans);
				box_rotations.emplace_back(rot);
				box_colors.emplace_back(col);
			}
		}
	}
	return true;
}

bool vr_box_stacking::judge_box_target(int box_id)
{
	mat3 rot_m;
	rot_m.normalize();
	movable_box_rotations[box_id].put_matrix(rot_m);
	vec3 gravity_m = movable_boxes[box_id].get_max_pnt() - movable_boxes[box_id].get_min_pnt();
	gravity_m = rot_m * gravity_m + movable_box_translations[box_id];
	mat3 rot_t;
	rot_t.normalize();
	frame_box_rotations[box_id].put_matrix(rot_t);
	vec3 gravity_t = frame_boxes[box_id].get_max_pnt() - frame_boxes[box_id].get_min_pnt();
	gravity_t = rot_t * gravity_t + frame_box_translations[box_id];
	vec3 diff = gravity_m - gravity_t;
	diff = dot(diff, diff);
	if (diff.x() < 0.0001 && diff.y() < 0.0001 && diff.z() < 0.0001)
	{
		return true;
	}
	else {
		return false;
	}
}

void vr_box_stacking::read_configuration(int cfg_index)
{
	clear_movable_boxes();
	clear_frame_boxes();
	if (cfg_index == 0) {
		load_boxes(QUOTE_SYMBOL_VALUE(INPUT_DIR) "/boxes_to_be_loaded.vrboxes", movable_boxes, movable_box_colors, movable_box_translations, movable_box_rotations);
		load_boxes(QUOTE_SYMBOL_VALUE(INPUT_DIR) "/target_position.txt", frame_boxes, frame_box_colors, frame_box_translations, frame_box_rotations);
	}
	else {
		load_boxes(QUOTE_SYMBOL_VALUE(INPUT_DIR) "/boxes_to_be_loaded_1.vrboxes", movable_boxes, movable_box_colors, movable_box_translations, movable_box_rotations);
		load_boxes(QUOTE_SYMBOL_VALUE(INPUT_DIR) "/target_position_1.txt", frame_boxes, frame_box_colors, frame_box_translations, frame_box_rotations);
	}
}

void vr_box_stacking::resize_box(int box_index, vec3 extends)
{
	vec3 center = (boxes[box_index].get_max_pnt() + boxes[box_index].get_min_pnt())*0.5f;
	vec3 minp = center - 0.5f*extends;
	vec3 maxp = center + 0.5f*extends;
	boxes[box_index] = box3(minp, maxp);
}

void vr_box_stacking::on_save_movable_boxes_cb()
{
	const std::string file_ending = ".vrboxes";
	std::string fn = cgv::gui::file_save_dialog("base file name", "Box configurations(vrboxes):*.vrboxes");
	if (!hasEnding(fn, file_ending)) {
		fn.append(file_ending);
	}
	if (fn.empty())
		return;

	save_boxes(fn, movable_boxes, movable_box_colors, movable_box_translations, movable_box_rotations);
}

void vr_box_stacking::on_load_movable_boxes_cb()
{
	std::string fn = cgv::gui::file_open_dialog("base file name", "Box configurations(vrboxes):*.vrboxes");
	if (!cgv::utils::file::exists(fn)) {
		std::cerr << "vr_box_stacking::on_load_movable_boxes_cb: file does not exist!\n";
		return;
	}
	clear_movable_boxes();
	if (!load_boxes(fn, movable_boxes, movable_box_colors, movable_box_translations, movable_box_rotations)) {
		std::cerr << "vr_box_stacking::on_load_movable_boxes_cb: failed to parse file!\n";
		clear_movable_boxes(); //delete all boxes after a failure to reach a valid logical state
	}
}

void vr_box_stacking::on_load_wireframe_boxes_cb()
{
	std::string fn = cgv::gui::file_open_dialog("base file name", "Box configurations(vrboxes):*.vrboxes");
	if (!cgv::utils::file::exists(fn)) {
		std::cerr << "vr_box_stacking::on_load_movable_boxes_cb: file does not exist!\n";
		return;
	}
	clear_frame_boxes();
	if (!load_boxes(fn, frame_boxes, frame_box_colors, frame_box_translations, frame_box_rotations)) {
		std::cerr << "vr_box_stacking::on_load_wireframe_boxes_cb: failed to parse file!\n";
		clear_frame_boxes(); //delete all boxes after a failure to reach a valid logical state
	}
}

void vr_box_stacking::on_set_vr_event_streaming_file()
{
	std::string fn = cgv::gui::file_save_dialog("base file name", "File Prefix"); //VR-Conntroller Record
	if (fn.empty())
		return;
	vr_events_record_path = fn;

	vr_events_stream = std::make_shared<std::ofstream>(fn + ".vrcr"); //VR Conntroller Record(.vrcr) :*.vrcr
	vr_events_stream->precision(std::numeric_limits<double>::max_digits10);
	box_trajectory_stream = std::make_shared<std::ofstream>(fn + ".btrj"); //Block trajectory : *.btrj
	box_trajectory_stream->precision(std::numeric_limits<double>::max_digits10);
	controller_trajectory_stream = std::make_shared<std::ofstream>(fn + ".ctrj"); //controller trajectory : *.ctrj
	controller_trajectory_stream->precision(std::numeric_limits<double>::max_digits10);
	if (!vr_events_stream->good()) {
		std::cerr << "vr_box_stacking::on_set_vr_event_streaming_file: can't write file!\n";
		vr_events_stream = nullptr;
	}
	post_recreate_gui();
}

void vr_box_stacking::clear_movable_boxes()
{
	movable_boxes.clear();
	movable_box_translations.clear();
	movable_box_rotations.clear();
	movable_box_colors.clear();
}

void vr_box_stacking::clear_frame_boxes()
{
	frame_boxes.clear();
	frame_box_translations.clear();
	frame_box_rotations.clear();
	frame_box_colors.clear();
}

#include <cgv/base/register.h>

cgv::base::object_registration<vr_box_stacking> vr_box_stacking_reg("vr_box_stacking");
