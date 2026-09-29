# GraspNet Synthetic Cylinder Contract

This document defines the synthetic cylinder used as the GraspNet input. It
is a stable contract for both ordinary synthetic-cylinder mode and box mode.

## Input geometry

1. Keep RGB-D pixels inside the selected object mask and the configured depth
   range.
2. Use the median valid depth as the observed front-surface depth.
3. Use the 5th and 95th mask pixel percentiles to estimate image width and
   height. The diameter is `pixel_width * front_depth / fx`, multiplied by
   `cylinder_diameter_scale`.
4. Convert the valid object pixels to camera XYZ and project them onto the
   configured reference axis. The 5th-to-95th percentile extent is the
   measured axial length, then `cylinder_height_scale` is applied.

## Coordinate contract

`R_base_reference[:, 2]` is the synthetic cylinder's positive Z axis in the
active arm base, and is transformed into camera coordinates through the
camera-to-base rotation. The right-arm contract is fixed:

```text
right synthetic-cylinder +Z = RealMan right-base +X = [1, 0, 0]
```

The loader validates this right-arm contract and rejects an overwritten or
malformed reference rotation.

## Center and samples

The image center at the observed front depth is moved along the camera-forward
direction projected perpendicular to the cylinder axis by
`radius - cylinder_forward_offset`. The center is then aligned to the axial
midpoint. The generated cloud is capless and samples:

```text
axis offset: uniform over +/- generated_axis_length / 2
angle:       uniform over pi +/- pi * cylinder_front_fraction
radius:      cylinder radius
```

With `cylinder_front_fraction = 1.0`, the full circumference is sampled.

## Box mode invariant

`--box y` does not change the cylinder dimensions, center computation, axis,
or point sampling. It only changes the later virtual-gripper orientation and
final pose post-processing. The cylinder surface and height correction must
still be applied before exporting the physical TCP pose.

The source of truth for the runtime values is the checked-in runtime
configuration and the argument defaults in `grasp.py`. Do not replace the
right-arm reference rotation with an identity matrix.
