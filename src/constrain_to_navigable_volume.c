#include "constrain_to_navigable_volume.h"

void constrain_to_navigable_volume(
    const float *const unconstrained_location,
    const int *const face_vertex_counts,
    const int *const face_vertex_offsets,
    const float *const face_vertex_locations,
    const float *const face_normals,
    const float *const edge_exit_normals,
    const float *const edge_normals,
    const float *const edge_coefficients,
    const float *const vertex_up_normals,
    const int face_index,
    float *const constrained_location)
{
  const int relevant_face_vertex_offset = face_vertex_offsets[face_index];
  const float *const relevant_face_vertex_locations = face_vertex_locations + relevant_face_vertex_offset * 3;

  const float difference[] = {
      unconstrained_location[0] - relevant_face_vertex_locations[0],
      unconstrained_location[1] - relevant_face_vertex_locations[1],
      unconstrained_location[2] - relevant_face_vertex_locations[2],
  };

  const float *const relevant_face_normal = face_normals + face_index * 3;

  const float surface_distance = difference[0] * relevant_face_normal[0] + difference[1] * relevant_face_normal[1] + difference[2] * relevant_face_normal[2];

  const float *const relevant_edge_normals = edge_normals + relevant_face_vertex_offset * 3;
  const float *const relevant_edge_exit_normals = edge_exit_normals + relevant_face_vertex_offset * 3;
  const float *const relevant_vertex_up_normals = vertex_up_normals + relevant_face_vertex_offset * 3;

  const int relevant_face_vertex_count = face_vertex_counts[face_index];

  if (surface_distance > 0.0f)
  {
    for (int vertex_index = 0; vertex_index < relevant_face_vertex_count; vertex_index++)
    {
      const float *const relevant_face_vertex_location = relevant_face_vertex_locations + vertex_index * 3;

      const float edge_difference[] = {
          unconstrained_location[0] - relevant_face_vertex_location[0],
          unconstrained_location[1] - relevant_face_vertex_location[1],
          unconstrained_location[2] - relevant_face_vertex_location[2],
      };

      const float *const relevant_edge_exit_normal = relevant_edge_exit_normals + vertex_index * 3;

      const float edge_distance = edge_difference[0] * relevant_edge_exit_normal[0] + edge_difference[1] * relevant_edge_exit_normal[1] + edge_difference[2] * relevant_edge_exit_normal[2];

      if (edge_distance > 0.0f)
      {
        const float edge_offset[] = {
            relevant_edge_exit_normal[0] * edge_distance,
            relevant_edge_exit_normal[1] * edge_distance,
            relevant_edge_exit_normal[2] * edge_distance,
        };

        const float edge_constrained_location[] = {
            unconstrained_location[0] - edge_offset[0],
            unconstrained_location[1] - edge_offset[1],
            unconstrained_location[2] - edge_offset[2],
        };

        const int previous_vertex_index = (vertex_index == 0 ? relevant_face_vertex_count : vertex_index) - 1;

        const float previous_difference[] = {
            edge_constrained_location[0] - relevant_face_vertex_location[0],
            edge_constrained_location[1] - relevant_face_vertex_location[1],
            edge_constrained_location[2] - relevant_face_vertex_location[2],
        };

        const float *const previous_edge_exit_normal = relevant_edge_exit_normals + previous_vertex_index * 3;

        const float previous_distance = previous_difference[0] * previous_edge_exit_normal[0] + previous_difference[1] * previous_edge_exit_normal[1] + previous_difference[2] * previous_edge_exit_normal[2];

        if (previous_distance > 0.0f)
        {
          const float *const relevant_vertex_up_normal = relevant_vertex_up_normals + vertex_index * 3;

          const float surface_distance = edge_difference[0] * relevant_vertex_up_normal[0] + edge_difference[1] * relevant_vertex_up_normal[1] + edge_difference[2] * relevant_vertex_up_normal[2];

          if (surface_distance <= 0.0f)
          {
            constrained_location[0] = relevant_face_vertex_location[0];
            constrained_location[1] = relevant_face_vertex_location[1];
            constrained_location[2] = relevant_face_vertex_location[2];
          }
          else
          {
            const float offset[] = {
                relevant_vertex_up_normal[0] * surface_distance,
                relevant_vertex_up_normal[1] * surface_distance,
                relevant_vertex_up_normal[2] * surface_distance,
            };

            constrained_location[0] = relevant_face_vertex_location[0] + offset[0];
            constrained_location[1] = relevant_face_vertex_location[1] + offset[1];
            constrained_location[2] = relevant_face_vertex_location[2] + offset[2];
          }
        }
        else
        {
          const int next_vertex_index = (vertex_index + 1) % relevant_face_vertex_count;

          const float *const next_face_vertex_location = relevant_face_vertex_locations + next_vertex_index * 3;

          const float next_difference[] = {
              edge_constrained_location[0] - next_face_vertex_location[0],
              edge_constrained_location[1] - next_face_vertex_location[1],
              edge_constrained_location[2] - next_face_vertex_location[2],
          };

          const float *const next_edge_exit_normal = relevant_edge_exit_normals + next_vertex_index * 3;

          const float next_distance = next_difference[0] * next_edge_exit_normal[0] + next_difference[1] * next_edge_exit_normal[1] + next_difference[2] * next_edge_exit_normal[2];

          if (next_distance > 0.0f)
          {
            const float *const next_vertex_up_normal = relevant_vertex_up_normals + next_vertex_index * 3;

            const float next_original_difference[] = {
                unconstrained_location[0] - next_face_vertex_location[0],
                unconstrained_location[1] - next_face_vertex_location[1],
                unconstrained_location[2] - next_face_vertex_location[2],
            };

            const float surface_distance = next_original_difference[0] * next_vertex_up_normal[0] + next_original_difference[1] * next_vertex_up_normal[1] + next_original_difference[2] * next_vertex_up_normal[2];

            if (surface_distance <= 0.0f)
            {
              constrained_location[0] = next_face_vertex_location[0];
              constrained_location[1] = next_face_vertex_location[1];
              constrained_location[2] = next_face_vertex_location[2];
            }
            else
            {
              const float offset[] = {
                  next_vertex_up_normal[0] * surface_distance,
                  next_vertex_up_normal[1] * surface_distance,
                  next_vertex_up_normal[2] * surface_distance,
              };

              constrained_location[0] = next_face_vertex_location[0] + offset[0];
              constrained_location[1] = next_face_vertex_location[1] + offset[1];
              constrained_location[2] = next_face_vertex_location[2] + offset[2];
            }
          }
          else
          {
            const float surface_distance = previous_difference[0] * relevant_face_normal[0] + previous_difference[1] * relevant_face_normal[1] + previous_difference[2] * relevant_face_normal[2];

            if (surface_distance < 0.0f)
            {
              const float *const relevant_edge_coefficient = edge_coefficients + relevant_face_vertex_offset * 3 + vertex_index * 3;

              const float unclamped = edge_difference[0] * relevant_edge_coefficient[0] + edge_difference[1] * relevant_edge_coefficient[1] + edge_difference[2] * relevant_edge_coefficient[2];

              if (unclamped <= 0.0f)
              {
                constrained_location[0] = relevant_face_vertex_location[0];
                constrained_location[1] = relevant_face_vertex_location[1];
                constrained_location[2] = relevant_face_vertex_location[2];
                return;
              }

              const float *const next_face_vertex_location = relevant_face_vertex_locations + ((vertex_index + 1) % relevant_face_vertex_count) * 3;

              if (unclamped >= 1.0f)
              {
                constrained_location[0] = next_face_vertex_location[0];
                constrained_location[1] = next_face_vertex_location[1];
                constrained_location[2] = next_face_vertex_location[2];
                return;
              }

              const float inverse = 1.0f - unclamped;

              constrained_location[0] = relevant_face_vertex_location[0] * inverse + next_face_vertex_location[0] * unclamped;
              constrained_location[1] = relevant_face_vertex_location[1] * inverse + next_face_vertex_location[1] * unclamped;
              constrained_location[2] = relevant_face_vertex_location[2] * inverse + next_face_vertex_location[2] * unclamped;
            }
            else
            {
              constrained_location[0] = edge_constrained_location[0];
              constrained_location[1] = edge_constrained_location[1];
              constrained_location[2] = edge_constrained_location[2];
            }
          }
        }

        return;
      }
    }

    constrained_location[0] = unconstrained_location[0];
    constrained_location[1] = unconstrained_location[1];
    constrained_location[2] = unconstrained_location[2];
  }
  else
  {
    const float surface_offset[] = {
        relevant_face_normal[0] * surface_distance,
        relevant_face_normal[1] * surface_distance,
        relevant_face_normal[2] * surface_distance,
    };

    const float surface_constrained_location[] = {
        unconstrained_location[0] - surface_offset[0],
        unconstrained_location[1] - surface_offset[1],
        unconstrained_location[2] - surface_offset[2],
    };

    for (int vertex_index = 0; vertex_index < relevant_face_vertex_count; vertex_index++)
    {
      const float *const relevant_face_vertex_location = relevant_face_vertex_locations + vertex_index * 3;

      const float edge_difference[] = {
          surface_constrained_location[0] - relevant_face_vertex_location[0],
          surface_constrained_location[1] - relevant_face_vertex_location[1],
          surface_constrained_location[2] - relevant_face_vertex_location[2],
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
          return;
        }

        const float *const next_face_vertex_location = relevant_face_vertex_locations + ((vertex_index + 1) % relevant_face_vertex_count) * 3;

        if (unclamped >= 1.0f)
        {
          constrained_location[0] = next_face_vertex_location[0];
          constrained_location[1] = next_face_vertex_location[1];
          constrained_location[2] = next_face_vertex_location[2];
          return;
        }

        const float inverse = 1.0f - unclamped;

        constrained_location[0] = relevant_face_vertex_location[0] * inverse + next_face_vertex_location[0] * unclamped;
        constrained_location[1] = relevant_face_vertex_location[1] * inverse + next_face_vertex_location[1] * unclamped;
        constrained_location[2] = relevant_face_vertex_location[2] * inverse + next_face_vertex_location[2] * unclamped;
        return;
      }
    }

    constrained_location[0] = surface_constrained_location[0];
    constrained_location[1] = surface_constrained_location[1];
    constrained_location[2] = surface_constrained_location[2];
  }
}
