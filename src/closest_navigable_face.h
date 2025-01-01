#ifndef CLOSEST_NAVIGABLE_FACE_H

#define CLOSEST_NAVIGABLE_FACE_H

/**
 * Finds the closest navigable face to a given location.
 * @param location The 3D vector describing the location to search from.
 *                 Behavior is undefined if any component is NaN, infinity or
 *                 negative infinity.
 * @param face_count The number of faces in the navigation mesh.
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
 * @param edge_normals A 3D unit vector perpendicular to both each edge of each
 *                     face of the navigation mesh and its corresponding face
 *                     surface normal, pointing out of the face into a
 *                     hypothetical neighboring face.
 * @param edge_coefficients A 3D vector for each vertex of each face of the
 *                          navigation mesh.  Each points to the next vertex of
 *                          the face, with a magnitude equal to the reciprocal
 *                          of the distance to that vertex.
 * @return The index of the closest face to the given location.
 */
int closest_navigable_face(
    const float *const location,
    const int face_count,
    const int *const face_vertex_counts,
    const int *const face_vertex_offsets,
    const float *const face_vertex_locations,
    const float *const face_normals,
    const float *const edge_normals,
    const float *const edge_coefficients);

#endif
