# SW2URDF

SolidWorks Assembly → URDF conversion pipeline for ROS-based robotics.

## Development History

| Version | Purpose | What we solved / discovered | What remained |
|---|---|---|---|
| **V1** | Initial CAD data extraction | Started extracting SolidWorks assembly components and mates | Raw CAD data wasn't yet understood kinematically |
| **V2** | Complete assembly extraction | Moved toward extracting **everything at once** instead of manually querying individual mates/features | Needed interpretation of relationships |
| **V3** | Kinematic relationship analysis | Grouped components by their mate connections and started determining what each connection means | `Concentric + Coincident` could mean fixed or revolute |
| **V4** | Kinematic model generation | Converted CAD relationships into a link/joint/tree representation | Joint classification rules needed refinement |
| **V5** | Joint classification refinement | Established the rule that `Concentric + Coincident` without `Lock` → revolute, while `Lock` → fixed | Joint frames/origins still needed better handling |
| **V6** | Coordinate-system groundwork | Introduced CAD → robot coordinate transformation | Robot's physical axis convention had to be explicitly defined |
| **V7** | Robot coordinate definition | Established **Robot X = Joint 2 axis, Robot Y = Joint 1 axis, Robot Z = X × Y** | Needed consistent parent/child joint frames |
| **V8** | Relative frame calculation | Combined kinematic structure with component transforms and calculated parent → child frames | Needed a cleaner, explicit frame representation |
| **V9** | Frame-resolution stage | Refined the joint-frame representation used downstream | V10 initially exposed an input-schema mismatch (`v9_frame`) |
| **V10** | Link/joint frame builder | Built explicit link frames, joint origins, revolute axes and fixed-joint rotations; generated RPY | We had the mathematical robot model but no visual meshes |
| **V11** | First URDF generation | Converted the validated kinematic model into an actual URDF | URDF contained links/joints but no CAD geometry |
| **V12** | Mesh integration | Added the four STL meshes to the URDF and validated the resulting structure | We discovered mesh placement/orientation was the next major issue |
| **V13A** | Mesh-frame diagnostic | Analyzed STL geometry: dimensions, centroid, PCA, triangle/vertex counts and CAD transforms | PCA cannot reliably tell us the original SolidWorks part frame |
| **V14 — Planned** | Feature/reference frame solver | Determine STL ↔ CAD frame using meaningful engineering geometry/features rather than assuming STL orientation | First feature-matching experiment still needs to be developed |
