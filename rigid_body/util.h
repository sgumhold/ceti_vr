#pragma once

#include <cgv/render/render_types.h>

using namespace cgv::render;
/// type shortcut for 3D vectors
typedef render_types::vec3 vec3;
/// type shortcut for 3x3 matrices
typedef render_types::mat3 mat3;
/// type shortcut for 3D axis aligned bounding boxes
typedef render_types::box3 box3;
/// type shortcut for quaternions
typedef render_types::quat quat;
/// type shortcut for rgb colors
typedef render_types::rgb rgb;

namespace util {

	vec3 ortho_vec(vec3 v) {

		return fabsf(v.x()) > fabsf(v.z()) ? vec3(-v.y(), v.x(), 0.0f) : vec3(0.0f, -v.z(), v.y());
	}
}
