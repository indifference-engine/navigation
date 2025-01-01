# Navigation

C library for navigation meshes.

## Usage

### Project Level

Your application's build pipeline will need to be configured to compile each C
file in the [src](./src) directory and recompile every C file should any H files
change.  Then, include each H file in the same directory to make its
corresponding function available.

### Assumptions

- The compilation environment supports C99.
- No float arguments:
  - Are NaN.
  - Are infinity.
  - Are negative infinity.

### Functions

| Name                             | Description                                                                   |
| -------------------------------- | ----------------------------------------------------------------------------- |
| `closest_navigable_face`         | Finds the closest face to a given location.                                   |
| `constrain_to_navigable_surface` | Constrains a given location to the surface of its containing navigation face. |
| `constrain_to_navigable_volume`  | Constrains a given location to the volume of its containing navigation face.  |
| `sliding_navigation_collision`   | Performs a single iteration of sliding collision against a navigation mesh.   |

## Tests

Execute `make` to run the test suite.

### Dependencies

- Make.
- MinGW-GCC for Windows, Clang for all other platforms.
- Bash.
