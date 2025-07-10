#ifndef CONSTRAIN_TO_NAVIGABLE_VOLUME_H

#define CONSTRAIN_TO_NAVIGABLE_VOLUME_H

/**
 * Constrains a given location to the volume of its containing navigation face.
 * @param unconstrained_location The 3D vector describing the location to
 *                               constrain to the volume of the containing
 *                               navigation face.  Behavior is undefined if any
 *                               component is NaN, infinity or negative
 *                               infinity.
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
 *                          face of the navigation mesh, pointing out of the
 *                          face into a hypothetical neighboring face.  Where no
 *                          neighbors exist, this is perpendicular to the face's
 *                          surface normal.  It is otherwise averaged with the
 *                          neighboring edge exit normals to ensure that there
 *                          is a consistent plane to cross to enter or exit the
 *                          face.
 * @param edge_normals A 3D unit vector perpendicular to both each edge of each
 *                     face of the navigation mesh and its corresponding face
 *                     surface normal, pointing out of the face into a
 *                     hypothetical neighboring face.
 * @param edge_coefficients A 3D vector for each vertex of each face of the
 *                          navigation mesh.  Each points to the next vertex of
 *                          the face, with a magnitude equal to the reciprocal
 *                          of the distance to that vertex.
 * @param vertex_up_normals A 3D unit vector for each vertex of each face,
 *                          perpendicular to the edge exit normal either side of
 *                          it and facing "up" (in the same general direction as
 *                          the face normal).
 * @param face_index The index of the face to which the location is to be
 *                   constrained.
 * @param constrained_location The 3D vector which is overwritten with the
 *                             resulting location.  May overlap with the
 *                             unconstrained location.
 */
void constrain_to_navigable_volume(
    const float *const unconstrained_location,
    const int *const face_vertex_counts, const int *const face_vertex_offsets,
    const float *const face_vertex_locations, const float *const face_normals,
    const float *const edge_exit_normals, const float *const edge_normals,
    const float *const edge_coefficients, const float *const vertex_up_normals,
    const int face_index, float *const constrained_location);

#endif
