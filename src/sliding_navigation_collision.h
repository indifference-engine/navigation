#ifndef SLIDING_NAVIGATION_COLLISION_H

#define SLIDING_NAVIGATION_COLLISION_H

/**
 * Performs a single iteration of sliding collision against a navigation mesh.
 * @param from The 3D vector describing the point the object is traveling from.
 *             Behavior is undefined if outside of the face's volume or if any
 *             component is NaN, infinity or negative infinity.
 * @param face_index The index of the face within which an iteration of sliding
 *                   collision is to be performed.
 * @param to The 3D vector describing the point the object is traveling to.
 *           Behavior is undefined if any component is NaN, infinity or negative
 *           infinity.
 * @param face_vertex_counts The number of vertices of each face in the
 *                           navigation mesh.  Behavior is defined if any is
 *                           less than 3.
 * @param face_vertex_offsets The exclusive running totals of the vertex count
 *                            of each face (e.g. for vertex counts 3, 5, 4, this
 *                            would contain 0, 3, 8).
 * @param face_vertex_locations A 3D vector describing the location of each
 *                              vertex within each face of the navigation mesh.
 *                              Behavior is undefined if any component is NaN,
 *                              infinity or negative infinity.
 * @param face_normals A 3D unit vector describing the surface normal of each
 *                     face within the navigation mesh.
 * @param edge_exit_normals A 3D unit vector perpendicular to each edge of each
 *                          face of the navigation mesh, pointning out of the
 *                          face into a hypothetical neighboring face.  Where no
 *                          neighbors exist, this is perpendicular to the face's
 *                          surface normal.  It is otherwise averaged with the
 *                          neighboring edge exit normals to ensure that there
 *                          is a consistent plane to cross to enter or exit the
 *                          face.
 * @param face_edge_neighbor_counts The number of neighboring faces for each
 *                                  edge of each face of the navigation mesh.
 * @param result_from Overwritten depending upon the result:
 *                    - None: Equal to "from".
 *                    - Surface: Equal to the point of impact, for "sticky"
 *                               collision.
 *                    - Edge (without neighbors): Equal to the point of impact,
 *                                                for "sticky" collision.
 *                    - Other (with neighbors): Equal to the point at which the
 *                                              motion crossed into the
 *                                              neighboring face, for "sticky"
 *                                              collision.
 *                    May be "from".
 * @param result_to Overwritten depending upon the result:
 *                  - None: Equal to "to".
 *                  - Surface: Equal to "to" projected onto the surface, for
 *                             "sliding" collision.  May land outside the volume
 *                             of the face.
 *                  - Edge (without neighbors): Equal to "to" projected onto the
 *                                              plane of the edge, for "sliding"
 *                                              collision.  May land outside the
 *                                              volume of the face.
 *                  - Other (with neighbors): Equal to "to" projected onto the
 *                                            corresponding edge, for "sliding"
 *                                            collision.
 *                  May be "to".
 * @param edge_index Overwritten depending upon the result:
 *                   - None: Undefined.
 *                   - Surface: Undefined.
 *                   - Edge: The index of the edge crossed or collided with.
 * @return A sliding navigation collision result.
 */
int sliding_navigation_collision(
    const float *const from, const int face_index, const float *const to,
    const int *const face_vertex_counts, const int *const face_vertex_offsets,
    const float *const face_vertex_locations, const float *const face_normals,
    const float *const edge_exit_normals,
    const int *const face_edge_neighbor_counts, float *const result_from,
    float *const result_to, int *const edge_index);

#endif
