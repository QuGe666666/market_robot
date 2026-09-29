# GraspNet Synthetic Cylinder Contract

The synthetic cylinder used by this ROS2 package follows the canonical rules
documented in `/home/lh/Supermarket/CYLINDER_GENERATION_RULES.md`.

The critical invariant is:

```text
right synthetic-cylinder +Z = RealMan right-base +X = [1, 0, 0]
```

`--box y` changes only the later gripper orientation and pose post-processing;
it must not change the synthetic-cylinder geometry or overwrite the right-arm
reference rotation.
