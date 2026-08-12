# SW2URDF

**SolidWorks Assembly → URDF conversion pipeline for ROS-based robotics.**

SW2URDF is an experimental CAD-to-robotics pipeline designed to convert a SolidWorks robotic assembly into a structured **URDF robot model** suitable for use with ROS/ROS 2.

The project does not simply convert CAD geometry into meshes. The goal is to understand the **mechanical structure of the assembly**, determine its joints and degrees of freedom from SolidWorks mates, establish a robot coordinate system, calculate joint/link frames, generate URDF, and finally integrate the original CAD geometry as STL meshes.

The long-term goal is to develop a more general pipeline capable of taking a properly assembled SolidWorks robot and automatically producing a usable ROS robot description.

---

# Project Status

**Current development stage: V13A**

The pipeline has successfully progressed through:

```text
SolidWorks Assembly
        ↓
SolidWorks API Extraction
        ↓
Structured CAD Data
        ↓
Mate / Relationship Analysis
        ↓
Kinematic Model
        ↓
Robot Coordinate System
        ↓
Joint & Link Frames
        ↓
URDF Generation
        ↓
STL Mesh Integration
        ↓
Mesh Frame Analysis
        ↓
[CURRENT PROBLEM]
STL ↔ CAD / Robot Frame Alignment
```

The next planned stage is **V14 — Feature/Reference Frame Solver**.

---

# Why This Project Exists

A SolidWorks assembly already contains a large amount of information about a robot:

- Parts
- Component hierarchy
- Component transforms
- Assembly positions
- Local coordinate systems
- Concentric mates
- Coincident mates
- Lock mates
- Other assembly relationships

However, this information is not directly a URDF.

A URDF requires the robot to be represented as:

```text
Links
  +
Joints
  +
Joint origins
  +
Joint axes
  +
Parent/child relationships
```

Therefore, the main challenge is not simply exporting CAD geometry.

The real challenge is:

> **How can the mechanical intent contained inside a SolidWorks assembly be interpreted automatically and converted into a valid robot kinematic model?**

This project explores that problem step by step.

---

# Development Approach

The project is intentionally being developed incrementally.

Instead of trying to build a complete SolidWorks → URDF converter immediately, each version solves one specific problem.

The output of one stage becomes the input to the next stage.

This makes it possible to identify exactly where the CAD-to-robot conversion becomes incorrect.

---

# Development History

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

---

# The Important Discovery About CAD Mates

One of the most important problems discovered during development was that SolidWorks mate combinations cannot always be interpreted directly as a joint type.

For example:

```text
Concentric + Coincident
```

initially produced an ambiguous classification:

```text
REVOLUTE_OR_FIXED
```

because geometrically both interpretations are possible.

A concentric mate establishes a common axis, while a coincident mate constrains the components along that geometry.

The project therefore needed to determine the **mechanical intent** behind the mates.

After refining the assembly and classification rules, the current rule became:

```text
Concentric + Coincident
        +
No Lock
        ↓
REVOLUTE
```

while:

```text
Lock mate
        ↓
FIXED
```

A component explicitly fixed in the SolidWorks assembly is treated as the root/fixed component.

This was a major transition from simply reading SolidWorks mates to actually interpreting them kinematically.

---

# Robot Coordinate System

Another major stage of the project was establishing a robot coordinate system.

The robot's coordinate convention was defined from the physical joint arrangement rather than blindly inheriting the SolidWorks global coordinate system.

The current definition is:

```text
Robot X = Joint 2 axis

Robot Y = Joint 1 axis

Robot Z = X × Y
```

This establishes a consistent robot/ROS coordinate convention that can be used downstream when generating URDF frames.

The CAD-to-robot transformation is therefore treated explicitly rather than assuming that the SolidWorks global axes are already suitable for the robot.

---

# Current Robot Model

The test assembly currently contains four components:

```text
01.Base-1
      │
      └── joint_1
            │
            ▼
      0.First_Mount-1
            │
            └── joint_2
                  │
                  ▼
      05.UR_5_02_Nema17_Joint-1
            │
            └── joint_3
                  │
                  ▼
      Cap_End_Effector_Joint-5
```

The current kinematic structure is:

```text
01.Base-1
    │
    │ Revolute - Joint 1
    ▼
0.First_Mount-1
    │
    │ Revolute - Joint 2
    ▼
05.UR_5_02_Nema17_Joint-1
    │
    │ Fixed
    ▼
Cap_End_Effector_Joint-5
```

This structure was successfully converted into URDF during V11.

---

# SolidWorks Extraction Macro

The SolidWorks macro:

```text
extractor01.swp
```

is responsible for extracting assembly information from SolidWorks.

The macro extracts the CAD information required by the Python analysis pipeline, including:

- Assembly information
- Component names
- Component paths
- Fixed/suppressed state
- Component transforms
- Assembly origins
- Local component axes
- SolidWorks mates
- Mate types
- Mate entities
- Concentric geometry
- Coincident geometry

The extracted information is stored as structured JSON and is then processed by the Python analyzer.

---

# Extraction and Analysis Pipeline

The overall architecture currently follows this structure:

```text
                    SOLIDWORKS
                         │
                         ▼
                 Robot Assembly
                  (.SLDASM)
                         │
                         ▼
                extractor01.swp
                         │
                         ▼
              CAD Data Extraction
                         │
                         ▼
                robot_cad_data
                     (.json)
                         │
                         ▼
             ┌─────────────────────┐
             │ Python Analyzer      │
             │                     │
             │ Component Analysis  │
             │ Mate Analysis       │
             │ Joint Classification│
             │ Frame Calculation   │
             └─────────────────────┘
                         │
                         ▼
                Kinematic Model
                     (.json)
                         │
                         ▼
              Coordinate Conversion
                         │
                         ▼
                 Frame Resolution
                         │
                         ▼
                    URDF
                         │
                         ▼
                STL Mesh Integration
                         │
                         ▼
                  Mesh Analysis
                         │
                         ▼
              RViz / ROS / ROS 2
```

---

# Python Development Stages

The project contains separate analyzer versions because each version represents a specific development stage.

```text
analyzer.py
    ↓
analyzer_v2.py
    ↓
analyzer_v3.py
    ↓
analyzer_v4.py
    ↓
analyzer_v5.py
    ↓
analyzer_v6.py
    ↓
analyzer_v7.py
    ↓
analyzer_v8.py
    ↓
analyzer_v9.py
    ↓
analyzer_v10.py
    ↓
analyzer_v11.py
    ↓
analyzer_v12.py
    ↓
analyzer_v13a.py
```

These versions are intentionally retained because they document the evolution of the conversion pipeline and the problems discovered at each stage.

---

# V1–V2: Understanding the SolidWorks Assembly

The first stage was simply to understand what information could be extracted from SolidWorks through its API.

The initial extraction focused on:

```text
Components
Transforms
Origins
Axes
Mates
Mate types
```

The early realization was that extracting individual pieces of information was not enough.

The analyzer needed a complete representation of the assembly.

This led to the V2 approach of extracting the assembly information in a structured form so that the Python pipeline could reason about the entire assembly.

---

# V3–V5: From Mates to Joints

The next major challenge was determining what SolidWorks mates actually meant mechanically.

A connection such as:

```text
Component A
     │
     ├── Concentric
     │
     └── Coincident
     │
Component B
```

does not explicitly say:

```text
This is a revolute joint.
```

The project therefore introduced kinematic classification.

The key rule established during V5 was:

```text
Concentric + Coincident + No Lock
                    ↓
                REVOLUTE
```

and:

```text
Lock
 ↓
FIXED
```

This allowed the assembly to be transformed into a robot-like structure of links and joints.

---

# V6–V7: Coordinate Systems

Once the joints were identified, another problem appeared.

The SolidWorks coordinate system was not automatically the same as the desired robot coordinate system.

The robot's physical joint arrangement was therefore used to define the robot axes.

The chosen convention became:

```text
X → Joint 2 axis
Y → Joint 1 axis
Z → X × Y
```

This transformation became an explicit part of the model rather than being hidden inside the code.

---

# V8–V10: Frames

Identifying a joint is only half of the problem.

URDF also requires the location and orientation of the joint.

Therefore the project progressively developed:

- Joint origins
- Parent frames
- Child frames
- Parent-to-child transforms
- Revolute axes
- Fixed-joint rotations
- Rotation matrices
- RPY representations

V10 finally produced an explicit mathematical representation containing:

```text
Links
Joints
Joint origins
Joint axes
Parent frames
Child frames
Fixed-joint rotations
RPY
```

At this point, the robot's **kinematic model was mathematically available**.

---

# V11: First URDF

V11 converted the validated kinematic model into an actual URDF.

The resulting URDF contained:

```xml
<link />
<joint />
<parent />
<child />
<origin />
<axis />
<limit />
```

The first URDF successfully represented the kinematic structure.

However, the links were empty.

There was no CAD geometry to visualize.

This led directly to V12.

---

# V12: STL Mesh Integration

The four SolidWorks components were exported as STL files:

```text
meshes/
├── 01.Base.STL
├── 0.First_Mount.STL
├── 05.UR_5_02_Nema17_Joint.STL
└── Cap_End_Effector_Joint.STL
```

V12 successfully integrated all four meshes into the URDF.

The important milestone was:

```text
Meshes found   : 4
Meshes missing : 0
```

and:

```text
Links  : 4
Joints : 3

VALIDATION PASSED
```

This confirmed that the URDF structure and mesh references were valid.

However, this exposed the next and much more subtle problem.

---

# The STL Coordinate-Frame Problem

The fact that an STL file contains the correct geometry does **not** guarantee that the geometry is oriented correctly relative to the robot.

An STL generally contains triangles/vertices representing geometry, but it does not inherently preserve all of the semantic information that existed in the original SolidWorks part.

For example, a part may have been exported from SolidWorks in an orientation that does not correspond to:

```text
Part X
Part Y
Part Z
```

used during the kinematic calculations.

Therefore we can have:

```text
Correct STL geometry
        +
Correct URDF
        +
Correct joint frames
        =
Wrong visual placement
```

This became the major problem after V12.

---

# V13A: Mesh-Frame Diagnostics

V13A was created to investigate this problem rather than immediately attempting to guess the correct transformation.

The analyzer examined STL geometry including:

- Vertex count
- Triangle count
- Bounding dimensions
- Centroid
- Geometric extents
- PCA
- Principal directions
- CAD component transforms

The purpose was to determine whether the STL geometry itself could reveal its original coordinate frame.

The important discovery was:

> **PCA can describe the dominant geometric directions of a part, but it cannot reliably reconstruct the original SolidWorks part coordinate system.**

For example, the longest geometric direction of a component is not necessarily its CAD X axis.

Likewise, a symmetric part can produce multiple equally valid principal directions.

Therefore:

```text
STL geometry
     ↓
PCA
     ↓
Geometric orientation
```

is not necessarily:

```text
STL geometry
     ↓
PCA
     ↓
Original SolidWorks coordinate frame
```

This is why the next stage requires a more intelligent solution.

---

# V14 — Planned: Feature / Reference Frame Solver

The current direction is to avoid assuming that STL orientation can be reconstructed purely from statistical geometry.

Instead, the project will investigate using **meaningful engineering features** to determine the part frame.

Potential useful features include:

- Cylindrical holes
- Cylindrical bosses
- Circular faces
- Planar faces
- Parallel faces
- Perpendicular faces
- Symmetry
- Repeated hole patterns
- Known dimensions
- Joint interfaces
- Mate geometry
- Assembly reference geometry

The general idea is:

```text
SolidWorks
     │
     ├── Feature / reference geometry
     │
     └── Component transform
             │
             ▼
       Expected CAD frame
             │
             │
STL ── Geometry feature detection
             │
             ▼
       Detected STL feature
             │
             ▼
       Frame correspondence
             │
             ▼
        STL → CAD transform
             │
             ▼
       Correct URDF mesh pose
```

The goal is eventually to establish a reliable transformation:

```text
T_CAD←STL
```

or its equivalent representation, allowing the STL mesh to be placed correctly in the URDF regardless of the orientation used during STL export.

This is the next major research/development problem.

---

# Current Project Architecture

```text
sw2urdfv01/
│
├── Analyzer/
│   ├── analyzer.py
│   ├── analyzer_v2.py
│   ├── analyzer_v3.py
│   ├── analyzer_v4.py
│   ├── analyzer_v5.py
│   ├── analyzer_v6.py
│   ├── analyzer_v7.py
│   ├── analyzer_v8.py
│   ├── analyzer_v9.py
│   ├── analyzer_v10.py
│   ├── analyzer_v11.py
│   ├── analyzer_v12.py
│   ├── analyzer_v13a.py
│   ├── kinematic_model.json
│   ├── mesh_frame_analysis_v13a.json
│   └── robot_v11.urdf
│
├── meshes/
│   ├── 01.Base.STL
│   ├── 0.First_Mount.STL
│   ├── 05.UR_5_02_Nema17_Joint.STL
│   └── Cap_End_Effector_Joint.STL
│
├── Robot_Assembly(API).SLDASM
├── extractor01.swp
├── README.md
└── .gitignore
```

---

# Technologies

The project currently uses:

- **SolidWorks** — CAD assembly and mechanical definition
- **SolidWorks API / Macro** — CAD data extraction
- **Python** — analysis and conversion pipeline
- **JSON** — intermediate structured data representation
- **URDF** — robot description format
- **STL** — CAD mesh representation
- **ROS / ROS 2** — target robotics environment
- **RViz** — planned visualization and validation environment
- **Git / GitHub** — project version control

---

# Design Philosophy

The project follows one important principle:

> **Do not hide uncertainty.**

Whenever the CAD data is insufficient to determine something reliably, the analyzer should identify that ambiguity rather than silently making an assumption.

For example, the early analyzer correctly identified:

```text
Concentric + Coincident
        ↓
REVOLUTE_OR_FIXED
```

before enough information existed to classify it.

Only after establishing the `Lock`-based rule was the classification changed to:

```text
Concentric + Coincident + No Lock
        ↓
REVOLUTE
```

This approach is important because a CAD-to-robot converter can produce a syntactically valid URDF while still producing a mechanically incorrect robot.

The goal of this project is therefore **correctness of interpretation**, not simply successful file conversion.

---

# Long-Term Goal

The long-term objective is to develop a pipeline where a user can provide a SolidWorks robotic assembly and obtain a ROS-ready robot description with minimal manual intervention.

The intended workflow is:

```text
Design robot in SolidWorks
          ↓
Apply proper assembly mates
          ↓
Run SolidWorks extraction macro
          ↓
Automatically analyze assembly
          ↓
Identify links and joints
          ↓
Determine joint axes
          ↓
Determine joint origins
          ↓
Determine robot coordinate system
          ↓
Calculate link/joint frames
          ↓
Export URDF
          ↓
Resolve STL mesh frames
          ↓
Load robot into ROS / RViz
          ↓
Eventually simulate and control robot
```

---

# Future Development

Planned development areas include:

### V14
Feature/reference-based STL frame solving.

### Future stages

- Automatic feature correspondence
- Improved STL ↔ CAD registration
- Automatic mesh origin correction
- Collision geometry generation
- Visual geometry generation
- Joint limits from CAD information where possible
- Physical properties / mass extraction
- Inertia extraction
- Material information
- More SolidWorks mate types
- Multi-DOF joint detection
- Prismatic joint detection
- Joint limit extraction
- Mimic joints
- More complex assembly trees
- Automatic ROS package generation
- RViz validation
- Gazebo simulation
- MoveIt integration

---

# Important Lessons So Far

The development of this project has shown that CAD-to-URDF conversion is not simply a file-format conversion problem.

It is a combination of:

```text
CAD interpretation
        +
Mechanical reasoning
        +
Coordinate transformations
        +
Kinematics
        +
Geometry processing
        +
Robot description generation
```

The most important challenges discovered so far are:

1. **CAD mates do not directly equal robot joints.**
2. **Geometric constraints must be interpreted according to mechanical intent.**
3. **The CAD coordinate system may not be the desired robot coordinate system.**
4. **Joint identification alone is insufficient; joint frames are equally important.**
5. **A valid URDF can still display the robot incorrectly if mesh frames are wrong.**
6. **STL geometry alone does not reliably preserve the original CAD coordinate frame.**
7. **Engineering features and reference geometry may provide a better solution than purely statistical methods such as PCA.**

---

# Repository Development Strategy

Each major development stage is kept as a separate analyzer version.

This is intentional.

The project is being developed as an experimental pipeline where each stage solves a clearly defined problem.

Therefore, older versions are retained instead of being overwritten.

The version history acts as a record of:

```text
Problem
   ↓
Hypothesis
   ↓
Implementation
   ↓
Result
   ↓
New problem discovered
   ↓
Next version
```

This makes it possible to understand not only the final implementation, but also **why each stage exists**.

---

# Current Milestone

At the current V13A stage:

```text
✓ SolidWorks assembly extraction
✓ Component extraction
✓ Transform extraction
✓ Mate extraction
✓ Mate geometry analysis
✓ Kinematic relationship analysis
✓ Revolute/fixed classification
✓ Robot coordinate definition
✓ Parent/child frame calculation
✓ Joint axis calculation
✓ URDF generation
✓ STL mesh integration
✓ Mesh geometry diagnostics
✗ Automatic STL ↔ CAD frame resolution
```

The next major objective is therefore:

> **Develop a reliable feature/reference-based method for determining the relationship between an exported STL coordinate frame and the original SolidWorks/CAD coordinate frame.**

---

# Author

**Mohan Raj**

Mechatronics Engineering

This project is being developed as an experimental robotics/CAD automation project with the goal of connecting mechanical CAD design workflows with ROS-based robot simulation and control.

---

# Project Status

**Experimental / Under Development**

The current implementation is being developed incrementally and should not yet be considered a general-purpose SolidWorks-to-URDF converter.

The repository represents the development process and the progressively solved problems rather than only a final production-ready tool.
