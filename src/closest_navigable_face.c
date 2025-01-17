#include "closest_navigable_face.h"

int closest_navigable_face(
    const float *const location,
    const int face_count,
    const int *const face_vertex_counts,
    const int *const face_vertex_offsets,
    const float *const face_vertex_locations,
    const float *const face_normals,
    const float *const edge_normals,
    const float *const edge_coefficients)
{
  int output = -1;
  float best_distance_squared = 1.0f / 0.0f;

  for (int face_index = 0; face_index < face_count; face_index++)
  {
    float constrained_location[3];

    const int relevant_face_vertex_offset = face_vertex_offsets[face_index];
    const float *const relevant_face_vertex_locations = face_vertex_locations + relevant_face_vertex_offset * 3;

    const float difference[] = {
        location[0] - relevant_face_vertex_locations[0],
        location[1] - relevant_face_vertex_locations[1],
        location[2] - relevant_face_vertex_locations[2],
    };

    const float *const relevant_face_normal = face_normals + face_index * 3;

    const float surface_distance = difference[0] * relevant_face_normal[0] + difference[1] * relevant_face_normal[1] + difference[2] * relevant_face_normal[2];

    const float surface_offset[] = {
        relevant_face_normal[0] * surface_distance,
        relevant_face_normal[1] * surface_distance,
        relevant_face_normal[2] * surface_distance,
    };

    constrained_location[0] = location[0] - surface_offset[0];
    constrained_location[1] = location[1] - surface_offset[1];
    constrained_location[2] = location[2] - surface_offset[2];

    const float *const relevant_edge_normals = edge_normals + relevant_face_vertex_offset * 3;

    const int relevant_face_vertex_count = face_vertex_counts[face_index];

    for (int vertex_index = 0; vertex_index < relevant_face_vertex_count; vertex_index++)
    {
      const float *const relevant_face_vertex_location = relevant_face_vertex_locations + vertex_index * 3;

      const float edge_difference[] = {
          constrained_location[0] - relevant_face_vertex_location[0],
          constrained_location[1] - relevant_face_vertex_location[1],
          constrained_location[2] - relevant_face_vertex_location[2],
      };

      const float *const relevant_edge_normal = relevant_edge_normals + vertex_index * 3;

      const float edge_distance = edge_difference[0] * relevant_edge_normal[0] + edge_difference[1] * relevant_edge_normal[1] + edge_difference[2] * relevant_edge_normal[2];

      if (edge_distance > 0.0f)
      {
        const float *const relevant_edge_coefficient = edge_coefficients + relevant_face_vertex_offset * 3 + vertex_index * 3;

        const float unclamped = edge_difference[0] * relevant_edge_coefficient[0] + edge_difference[1] * relevant_edge_coefficient[1] + edge_difference[2] * relevant_edge_coefficient[2];

        if (unclamped <= 0.0f)
        {
          constrained_location[0] = relevant_face_vertex_location[0];
          constrained_location[1] = relevant_face_vertex_location[1];
          constrained_location[2] = relevant_face_vertex_location[2];
          break;
        }

        const float *const next_face_vertex_location = relevant_face_vertex_locations + ((vertex_index + 1) % relevant_face_vertex_count) * 3;

        if (unclamped >= 1.0f)
        {
          constrained_location[0] = next_face_vertex_location[0];
          constrained_location[1] = next_face_vertex_location[1];
          constrained_location[2] = next_face_vertex_location[2];
          break;
        }

        const float inverse = 1.0f - unclamped;

        constrained_location[0] = relevant_face_vertex_location[0] * inverse + next_face_vertex_location[0] * unclamped;
        constrained_location[1] = relevant_face_vertex_location[1] * inverse + next_face_vertex_location[1] * unclamped;
        constrained_location[2] = relevant_face_vertex_location[2] * inverse + next_face_vertex_location[2] * unclamped;

        break;
      }
    }

    const float delta[] = {
        constrained_location[0] - location[0],
        constrained_location[1] - location[1],
        constrained_location[2] - location[2],
    };

    const float distance_squared = delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2];

    if (distance_squared <= best_distance_squared)
    {
      output = face_index;
      best_distance_squared = distance_squared;
    }
  }

  return output;
}
