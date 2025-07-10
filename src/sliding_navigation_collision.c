#include "sliding_navigation_collision.h"
#include "sliding_navigation_collision_result.h"

static const float tolerance = 0.0001f;

int sliding_navigation_collision(
    const float *const from, const int face_index, const float *const to,
    const int *const face_vertex_counts, const int *const face_vertex_offsets,
    const float *const face_vertex_locations, const float *const face_normals,
    const float *const edge_exit_normals,
    const int *const face_edge_neighbor_counts, float *const result_from,
    float *const result_to) {
  float best_along = 1.0f / 0.0f;
  int output = SLIDING_NAVIGATION_COLLISION_RESULT_NONE;
  float best_normal[] = {0.0f, 0.0f, 0.0f};
  float best_escape = 0.0f;

  const int relevant_face_vertex_offset = face_vertex_offsets[face_index];
  const float *const relevant_face_vertex_locations =
      face_vertex_locations + relevant_face_vertex_offset * 3;
  const float *const relevant_face_normal = face_normals + face_index * 3;
  const float *const relevant_edge_exit_normals =
      edge_exit_normals + relevant_face_vertex_offset * 3;
  const int *const relevant_face_edge_neighbor_counts =
      face_edge_neighbor_counts + relevant_face_vertex_offset;

  const float to_surface_difference[] = {
      to[0] - relevant_face_vertex_locations[0],
      to[1] - relevant_face_vertex_locations[1],
      to[2] - relevant_face_vertex_locations[2],
  };

  const float to_surface_distance =
      to_surface_difference[0] * relevant_face_normal[0] +
      to_surface_difference[1] * relevant_face_normal[1] +
      to_surface_difference[2] * relevant_face_normal[2];

  if (to_surface_distance < 0.0f) {
    const float from_surface_difference[] = {
        from[0] - relevant_face_vertex_locations[0],
        from[1] - relevant_face_vertex_locations[1],
        from[2] - relevant_face_vertex_locations[2],
    };

    const float from_surface_distance =
        from_surface_difference[0] * relevant_face_normal[0] +
        from_surface_difference[1] * relevant_face_normal[1] +
        from_surface_difference[2] * relevant_face_normal[2];

    if (from_surface_distance >= 0.0f) {
      const float adjusted_from_surface_distance =
          from_surface_distance - tolerance;
      const float adjusted_to_surface_distance =
          to_surface_distance - tolerance;

      best_along =
          adjusted_from_surface_distance /
          (adjusted_from_surface_distance - adjusted_to_surface_distance);
      output = SLIDING_NAVIGATION_COLLISION_RESULT_SURFACE;
      best_normal[0] = relevant_face_normal[0];
      best_normal[1] = relevant_face_normal[1];
      best_normal[2] = relevant_face_normal[2];
      best_escape = -adjusted_to_surface_distance;
    }
  }

  const int relevant_face_vertex_count = face_vertex_counts[face_index];

  for (int vertex_index = 0; vertex_index < relevant_face_vertex_count;
       vertex_index++) {
    const float *const relevant_face_vertex_location =
        relevant_face_vertex_locations + vertex_index * 3;

    const float to_edge_difference[] = {
        to[0] - relevant_face_vertex_location[0],
        to[1] - relevant_face_vertex_location[1],
        to[2] - relevant_face_vertex_location[2],
    };

    const float *const relevant_edge_exit_normal =
        relevant_edge_exit_normals + vertex_index * 3;

    const float to_edge_distance =
        to_edge_difference[0] * relevant_edge_exit_normal[0] +
        to_edge_difference[1] * relevant_edge_exit_normal[1] +
        to_edge_difference[2] * relevant_edge_exit_normal[2];

    if (to_edge_distance > 0.0f) {
      const float from_edge_difference[] = {
          from[0] - relevant_face_vertex_location[0],
          from[1] - relevant_face_vertex_location[1],
          from[2] - relevant_face_vertex_location[2],
      };

      const float from_edge_distance =
          from_edge_difference[0] * relevant_edge_exit_normal[0] +
          from_edge_difference[1] * relevant_edge_exit_normal[1] +
          from_edge_difference[2] * relevant_edge_exit_normal[2];

      if (from_edge_distance <= 0.0f) {
        if (relevant_face_edge_neighbor_counts[vertex_index] == 0) {
          const float adjusted_from_edge_distance =
              from_edge_distance + tolerance;
          const float adjusted_to_edge_distance = to_edge_distance + tolerance;

          const float along =
              adjusted_from_edge_distance /
              (adjusted_from_edge_distance - adjusted_to_edge_distance);

          if (along <= best_along) {
            best_along = along;
            output = SLIDING_NAVIGATION_COLLISION_RESULT_EDGE;
            best_normal[0] = relevant_edge_exit_normal[0];
            best_normal[1] = relevant_edge_exit_normal[1];
            best_normal[2] = relevant_edge_exit_normal[2];
            best_escape = -adjusted_to_edge_distance;
          }
        } else {
          const float adjusted_from_edge_distance =
              from_edge_distance + tolerance;
          const float adjusted_to_edge_distance = to_edge_distance + tolerance;

          const float along =
              adjusted_from_edge_distance /
              (adjusted_from_edge_distance - adjusted_to_edge_distance);

          if (along <= best_along) {
            const float secondary_adjusted_from_edge_distance =
                from_edge_distance - tolerance;
            const float secondary_adjusted_to_edge_distance =
                to_edge_distance - tolerance;

            best_along = along;
            best_escape = secondary_adjusted_from_edge_distance /
                          (secondary_adjusted_from_edge_distance -
                           secondary_adjusted_to_edge_distance);
            output = vertex_index;
          }
        }
      }
    }
  }

  switch (output) {
  case SLIDING_NAVIGATION_COLLISION_RESULT_NONE:
    result_from[0] = from[0];
    result_from[1] = from[1];
    result_from[2] = from[2];
    result_to[0] = to[0];
    result_to[1] = to[1];
    result_to[2] = to[2];
    break;

  case SLIDING_NAVIGATION_COLLISION_RESULT_SURFACE:
  case SLIDING_NAVIGATION_COLLISION_RESULT_EDGE: {
    const float forward =
        best_along > 1.0f ? 1.0f : (best_along < 0.0f ? 0.0f : best_along);
    const float inverse = 1.0f - forward;

    result_from[0] = from[0] * inverse + to[0] * forward;
    result_from[1] = from[1] * inverse + to[1] * forward;
    result_from[2] = from[2] * inverse + to[2] * forward;

    const float adjustment[] = {
        best_normal[0] * best_escape,
        best_normal[1] * best_escape,
        best_normal[2] * best_escape,
    };

    result_to[0] = to[0] + adjustment[0];
    result_to[1] = to[1] + adjustment[1];
    result_to[2] = to[2] + adjustment[2];
    break;
  }

  default: {
    const float forward =
        best_escape > 1.0f ? 1.0f : (best_escape < 0.0f ? 0.0f : best_escape);
    const float inverse = 1.0f - forward;

    result_from[0] = from[0] * inverse + to[0] * forward;
    result_from[1] = from[1] * inverse + to[1] * forward;
    result_from[2] = from[2] * inverse + to[2] * forward;

    result_to[0] = to[0];
    result_to[1] = to[1];
    result_to[2] = to[2];
    break;
  }
  }

  return output;
}
