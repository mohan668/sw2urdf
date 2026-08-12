import json
import math
from pathlib import Path


# ================================================================
# SOLIDWORKS → URDF ANALYZER
# V10 - LINK / JOINT FRAME BUILDER
# ================================================================

INPUT_FILE = "kinematic_model_v9.json"
OUTPUT_FILE = "kinematic_model_v10.json"

TOLERANCE = 1e-6


# ================================================================
# BASIC VECTOR FUNCTIONS
# ================================================================

def vec_add(a, b):
    return [
        a[0] + b[0],
        a[1] + b[1],
        a[2] + b[2]
    ]


def vec_sub(a, b):
    return [
        a[0] - b[0],
        a[1] - b[1],
        a[2] - b[2]
    ]


def vec_scale(a, s):
    return [
        a[0] * s,
        a[1] * s,
        a[2] * s
    ]


def dot(a, b):
    return (
        a[0] * b[0] +
        a[1] * b[1] +
        a[2] * b[2]
    )


def cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    ]


def norm(v):
    return math.sqrt(dot(v, v))


def normalize(v):

    n = norm(v)

    if n < TOLERANCE:
        raise ValueError(
            f"Cannot normalize zero vector: {v}"
        )

    return [
        v[0] / n,
        v[1] / n,
        v[2] / n
    ]


def almost_equal(a, b, tolerance=TOLERANCE):

    return all(
        abs(a[i] - b[i]) <= tolerance
        for i in range(3)
    )


# ================================================================
# MATRIX FUNCTIONS
# ================================================================

def mat_vec(R, v):

    return [
        R[0][0] * v[0] +
        R[0][1] * v[1] +
        R[0][2] * v[2],

        R[1][0] * v[0] +
        R[1][1] * v[1] +
        R[1][2] * v[2],

        R[2][0] * v[0] +
        R[2][1] * v[1] +
        R[2][2] * v[2]
    ]


def mat_transpose(R):

    return [
        [R[0][0], R[1][0], R[2][0]],
        [R[0][1], R[1][1], R[2][1]],
        [R[0][2], R[1][2], R[2][2]]
    ]


def mat_mul(A, B):

    result = [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0]
    ]

    for i in range(3):

        for j in range(3):

            for k in range(3):

                result[i][j] += (
                    A[i][k] * B[k][j]
                )

    return result


def identity_matrix():

    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]
    ]


# ================================================================
# ROTATION MATRIX → RPY
#
# ROS / URDF convention:
#
# R = Rz(yaw) * Ry(pitch) * Rx(roll)
# ================================================================

def rotation_to_rpy(R):

    sy = math.sqrt(
        R[0][0] ** 2 +
        R[1][0] ** 2
    )

    singular = sy < 1e-9

    if not singular:

        roll = math.atan2(
            R[2][1],
            R[2][2]
        )

        pitch = math.atan2(
            -R[2][0],
            sy
        )

        yaw = math.atan2(
            R[1][0],
            R[0][0]
        )

    else:

        roll = math.atan2(
            -R[1][2],
            R[1][1]
        )

        pitch = math.atan2(
            -R[2][0],
            sy
        )

        yaw = 0.0

    return [
        roll,
        pitch,
        yaw
    ]


def rad_to_deg(v):

    return [
        math.degrees(x)
        for x in v
    ]


# ================================================================
# LOAD V9
# ================================================================

def load_v9():

    path = Path(INPUT_FILE)

    if not path.exists():

        raise FileNotFoundError(
            f"Input file not found: {INPUT_FILE}\n"
            f"Place the V9 JSON in the Analyzer folder."
        )

    with open(
        path,
        "r",
        encoding="utf-8"
    ) as f:

        data = json.load(f)

    version = str(
        data.get("version", "")
    )

    if version != "9.0":

        raise ValueError(
            f"Expected V9 model, "
            f"but received version {version}"
        )

    return data


# ================================================================
# BUILD LINK MAP
# ================================================================

def build_link_map(model):

    links = {}

    for link in model.get(
        "links",
        []
    ):

        name = link["name"]

        links[name] = {

            "name": name,

            "cad_component":
                link.get(
                    "cad_component",
                    name
                )

        }

    return links


# ================================================================
# BUILD JOINT TREE
# ================================================================

def build_joint_tree(model):

    tree = {}

    for joint in model.get(
        "joints",
        []
    ):

        parent = joint["parent"]
        child = joint["child"]

        tree[child] = {

            "joint":
                joint["name"],

            "parent":
                parent

        }

    return tree


# ================================================================
# DETERMINE LINK ORDER
# ================================================================

def determine_link_order(model):

    root = model["root"]

    joints = model["joints"]

    children = {}

    for joint in joints:

        parent = joint["parent"]
        child = joint["child"]

        children.setdefault(
            parent,
            []
        ).append(child)

    order = []

    def walk(link):

        order.append(link)

        for child in children.get(
            link,
            []
        ):

            walk(child)

    walk(root)

    return order


# ================================================================
# V9 FRAME LOOKUP
#
# IMPORTANT:
#
# V9 does NOT store:
#
# joint["v9_frame"]
#
# Instead it stores:
#
# model["v9_joint_frames"]
#
# ================================================================

def build_v9_frame_map(model):

    frame_map = {}

    for frame_entry in model.get(
        "v9_joint_frames",
        []
    ):

        joint_name = frame_entry.get(
            "joint"
        )

        if not joint_name:
            continue

        frame_map[joint_name] = frame_entry

    return frame_map


def extract_joint_frame(
    joint,
    frame_map
):

    joint_name = joint["name"]

    if joint_name not in frame_map:

        raise ValueError(
            f"Resolved V9 frame not found "
            f"for joint: {joint_name}"
        )

    frame_entry = frame_map[
        joint_name
    ]

    frame = frame_entry.get(
        "frame",
        {}
    )

    status = frame.get(
        "status"
    )

    if status != "RESOLVED":

        raise ValueError(
            f"Joint {joint_name} "
            f"frame is not RESOLVED."
        )

    return frame


# ================================================================
# REVOLUTE JOINT FRAME
# ================================================================

def build_revolute_joint(
    joint,
    frame_map
):

    frame = extract_joint_frame(
        joint,
        frame_map
    )

    # ------------------------------------------------------------
    # Global robot-frame joint data
    # ------------------------------------------------------------

    origin_robot = frame.get(
        "origin_robot"
    )

    axis_robot = frame.get(
        "axis_robot"
    )

    if origin_robot is None:

        raise ValueError(
            f"{joint['name']} has no "
            f"origin_robot."
        )

    if axis_robot is None:

        raise ValueError(
            f"{joint['name']} has no "
            f"axis_robot."
        )

    axis_robot = normalize(
        axis_robot
    )

    # ------------------------------------------------------------
    # Parent-frame data
    # ------------------------------------------------------------

    origin_parent_robot = frame.get(
        "origin_parent_robot"
    )

    axis_parent_robot = frame.get(
        "axis_parent_robot"
    )

    if origin_parent_robot is None:

        raise ValueError(
            f"{joint['name']} has no "
            f"origin_parent_robot."
        )

    if axis_parent_robot is None:

        raise ValueError(
            f"{joint['name']} has no "
            f"axis_parent_robot."
        )

    axis_parent_robot = normalize(
        axis_parent_robot
    )

    # ------------------------------------------------------------
    # URDF joint origin and axis
    #
    # For this stage we preserve the resolved V9
    # parent-frame values.
    # ------------------------------------------------------------

    origin = origin_parent_robot

    axis = axis_parent_robot

    return {

        "name":
            joint["name"],

        "type":
            "revolute",

        "parent":
            joint["parent"],

        "child":
            joint["child"],

        "frame": {

            "origin_robot_global":
                origin_robot,

            "axis_robot_global":
                axis_robot,

            "origin_parent":
                origin,

            "axis_parent":
                axis,

            "origin":
                origin,

            "axis":
                axis

        },

        "source": {

            "source_mate":
                frame.get(
                    "source_mate"
                ),

            "candidate_concentric_mates":
                frame.get(
                    "candidate_concentric_mates",
                    []
                ),

            "geometry_validation":
                frame.get(
                    "geometry_validation",
                    {}
                )

        }

    }


# ================================================================
# FIXED JOINT FRAME
# ================================================================

def build_fixed_joint(
    joint,
    frame_map
):

    frame = extract_joint_frame(
        joint,
        frame_map
    )

    # ------------------------------------------------------------
    # Parent-frame origin
    # ------------------------------------------------------------

    origin = frame.get(
        "origin_parent_robot"
    )

    if origin is None:

        raise ValueError(
            f"Fixed joint {joint['name']} "
            f"has no origin_parent_robot."
        )

    # ------------------------------------------------------------
    # V9 stores the fixed-child orientation
    # as:
    #
    # child_rotation_in_parent_cad
    #
    # The V9 coordinate conversion is already represented
    # elsewhere. For V10 we preserve this matrix rather than
    # inventing another transformation.
    # ------------------------------------------------------------

    rotation = frame.get(
        "child_rotation_in_parent_cad"
    )

    if rotation is None:

        rotation = identity_matrix()

        rotation_source = (
            "identity_fallback"
        )

    else:

        rotation_source = (
            "V9 child_rotation_in_parent_cad"
        )

    rpy = rotation_to_rpy(
        rotation
    )

    return {

        "name":
            joint["name"],

        "type":
            "fixed",

        "parent":
            joint["parent"],

        "child":
            joint["child"],

        "frame": {

            "origin":
                origin,

            "rpy":
                rpy,

            "rpy_degrees":
                rad_to_deg(rpy),

            "rotation_matrix":
                rotation

        },

        "source": {

            "fixed_joint_source":
                "SolidWorks component transforms",

            "rotation_source":
                rotation_source

        }

    }


# ================================================================
# BUILD ALL JOINT FRAMES
# ================================================================

def build_joint_frames(model):

    result = []

    # ------------------------------------------------------------
    # Build lookup once.
    # ------------------------------------------------------------

    frame_map = build_v9_frame_map(
        model
    )

    # ------------------------------------------------------------
    # Verify all joints have V9 frames.
    # ------------------------------------------------------------

    for joint in model.get(
        "joints",
        []
    ):

        if joint["name"] not in frame_map:

            raise ValueError(
                f"Joint {joint['name']} "
                f"has no entry in v9_joint_frames."
            )

    # ------------------------------------------------------------
    # Process joints
    # ------------------------------------------------------------

    for joint in model["joints"]:

        joint_type = joint["type"]

        if joint_type == "revolute":

            built = build_revolute_joint(
                joint,
                frame_map
            )

        elif joint_type == "fixed":

            built = build_fixed_joint(
                joint,
                frame_map
            )

        else:

            raise ValueError(
                f"Unsupported joint type: "
                f"{joint_type}"
            )

        result.append(
            built
        )

    return result


# ================================================================
# BUILD LINK FRAME INFORMATION
# ================================================================

def build_link_frames(
    model,
    joints
):

    links = build_link_map(
        model
    )

    root = model["root"]

    link_frames = []

    # ------------------------------------------------------------
    # ROOT LINK
    # ------------------------------------------------------------

    if root not in links:

        raise ValueError(
            f"Root component not found: "
            f"{root}"
        )

    link_frames.append({

        "name":
            root,

        "cad_component":
            links[root][
                "cad_component"
            ],

        "parent":
            None,

        "parent_joint":
            None,

        "frame": {

            "origin":
                [
                    0.0,
                    0.0,
                    0.0
                ],

            "rpy":
                [
                    0.0,
                    0.0,
                    0.0
                ],

            "rpy_degrees":
                [
                    0.0,
                    0.0,
                    0.0
                ]

        },

        "status":
            "ROOT"

    })

    # ------------------------------------------------------------
    # CHILD LINKS
    # ------------------------------------------------------------

    for joint in joints:

        child = joint["child"]
        parent = joint["parent"]

        if child not in links:

            raise ValueError(
                f"Child component not found: "
                f"{child}"
            )

        frame = joint["frame"]

        link_frames.append({

            "name":
                child,

            "cad_component":
                links[child][
                    "cad_component"
                ],

            "parent":
                parent,

            "parent_joint":
                joint["name"],

            "frame": {

                "origin":
                    frame["origin"],

                "rpy":
                    frame.get(
                        "rpy",
                        [
                            0.0,
                            0.0,
                            0.0
                        ]
                    ),

                "rpy_degrees":
                    frame.get(
                        "rpy_degrees",
                        [
                            0.0,
                            0.0,
                            0.0
                        ]
                    )

            },

            "status":
                "CHILD"

        })

    return link_frames


# ================================================================
# VALIDATION
# ================================================================

def validate_model(
    model,
    joint_frames,
    link_frames
):

    errors = []
    warnings = []

    # ------------------------------------------------------------
    # Root
    # ------------------------------------------------------------

    root = model.get(
        "root"
    )

    if not root:

        errors.append(
            "No root component defined."
        )

    # ------------------------------------------------------------
    # Source links
    # ------------------------------------------------------------

    source_links = {

        x["name"]

        for x in model.get(
            "links",
            []
        )

    }

    output_links = {

        x["name"]

        for x in link_frames

    }

    missing_links = (
        source_links -
        output_links
    )

    if missing_links:

        errors.append(
            "Missing links: " +
            ", ".join(
                sorted(
                    missing_links
                )
            )
        )

    # ------------------------------------------------------------
    # Joint validation
    # ------------------------------------------------------------

    for joint in joint_frames:

        parent = joint["parent"]
        child = joint["child"]

        if parent not in output_links:

            errors.append(
                f"{joint['name']}: "
                f"parent link missing: "
                f"{parent}"
            )

        if child not in output_links:

            errors.append(
                f"{joint['name']}: "
                f"child link missing: "
                f"{child}"
            )

        # --------------------------------------------------------
        # Revolute axis validation
        # --------------------------------------------------------

        if joint["type"] == "revolute":

            axis = joint[
                "frame"
            ][
                "axis"
            ]

            axis_length = norm(
                axis
            )

            if abs(
                axis_length - 1.0
            ) > 1e-6:

                errors.append(
                    f"{joint['name']}: "
                    f"axis is not normalized: "
                    f"{axis}"
                )

    # ------------------------------------------------------------
    # Child-parent uniqueness
    # ------------------------------------------------------------

    child_count = {}

    for joint in joint_frames:

        child = joint["child"]

        child_count[child] = (
            child_count.get(
                child,
                0
            ) + 1
        )

    for child, count in child_count.items():

        if count > 1:

            errors.append(
                f"Link {child} has "
                f"{count} parent joints."
            )

    # ------------------------------------------------------------
    # Every non-root link should have one parent
    # ------------------------------------------------------------

    for link in output_links:

        if link == root:
            continue

        if link not in child_count:

            warnings.append(
                f"Link {link} has "
                f"no parent joint."
            )

    # ------------------------------------------------------------
    # Joint count sanity check
    # ------------------------------------------------------------

    expected_joint_count = (
        len(output_links) - 1
    )

    if len(joint_frames) != expected_joint_count:

        warnings.append(
            "Joint count does not equal "
            "(link count - 1). "
            "This may indicate a "
            "non-tree structure."
        )

    return errors, warnings


# ================================================================
# BUILD V10 MODEL
# ================================================================

def process_model(model):

    # ------------------------------------------------------------
    # Build joint frames
    # ------------------------------------------------------------

    joints = build_joint_frames(
        model
    )

    # ------------------------------------------------------------
    # Build link frames
    # ------------------------------------------------------------

    link_frames = build_link_frames(
        model,
        joints
    )

    # ------------------------------------------------------------
    # Validate
    # ------------------------------------------------------------

    errors, warnings = validate_model(
        model,
        joints,
        link_frames
    )

    if errors:

        print()
        print(
            "VALIDATION ERRORS"
        )

        for error in errors:

            print(
                " -",
                error
            )

        raise ValueError(
            "V10 validation failed."
        )

    # ------------------------------------------------------------
    # Link order
    # ------------------------------------------------------------

    link_order = determine_link_order(
        model
    )

    # ------------------------------------------------------------
    # V10 output
    # ------------------------------------------------------------

    v10 = {

        "format":
            "sw2urdf_kinematic_model",

        "version":
            "10.0",

        "stage":
            "link_joint_frame_builder",

        "source": {

            "input":
                INPUT_FILE,

            "input_version":
                model.get(
                    "version"
                ),

            "assembly":
                model.get(
                    "source_assembly",
                    {}
                )

        },

        "coordinate_system":
            model.get(
                "coordinate_system",
                {}
            ),

        "root":
            model["root"],

        "link_order":
            link_order,

        "links":
            link_frames,

        "joints":
            joints,

        "tree":
            model.get(
                "tree",
                []
            ),

        "validation": {

            "status":
                "PASS",

            "errors":
                errors,

            "warnings":
                warnings

        },

        "v10_processing": {

            "description":
                "Builds explicit link and joint frames from the resolved V9 kinematic model.",

            "v9_frame_source":
                "top-level v9_joint_frames array",

            "link_frames":
                True,

            "joint_parent_frames":
                True,

            "revolute_axes":
                True,

            "fixed_joint_rotations":
                True,

            "rpy_calculation":
                True,

            "mass_properties":
                False,

            "mesh_information":
                False

        }

    }

    return v10


# ================================================================
# PRINT SUMMARY
# ================================================================

def print_summary(v10):

    print()
    print(
        "=" * 72
    )

    print(
        "SOLIDWORKS CAD → URDF ANALYZER V10"
    )

    print(
        "LINK / JOINT FRAME BUILDER"
    )

    print(
        "=" * 72
    )

    print()

    print(
        "Input version :",
        v10[
            "source"
        ][
            "input_version"
        ]
    )

    print(
        "Output version:",
        v10[
            "version"
        ]
    )

    print(
        "Root          :",
        v10[
            "root"
        ]
    )

    # ------------------------------------------------------------
    # Links
    # ------------------------------------------------------------

    print()

    print(
        "-" * 72
    )

    print(
        "LINK TREE"
    )

    print(
        "-" * 72
    )

    for link in v10["links"]:

        print()

        print(
            f"LINK: {link['name']}"
        )

        print(
            f"  Status: {link['status']}"
        )

        if link["parent"]:

            print(
                f"  Parent: "
                f"{link['parent']}"
            )

            print(
                f"  Joint : "
                f"{link['parent_joint']}"
            )

        print(
            "  Origin:",
            link[
                "frame"
            ][
                "origin"
            ]
        )

        print(
            "  RPY:",
            link[
                "frame"
            ][
                "rpy"
            ]
        )

    # ------------------------------------------------------------
    # Joints
    # ------------------------------------------------------------

    print()

    print(
        "-" * 72
    )

    print(
        "JOINT FRAMES"
    )

    print(
        "-" * 72
    )

    for joint in v10["joints"]:

        print()

        print(
            f"JOINT: "
            f"{joint['name']}"
        )

        print(
            f"  Type  : "
            f"{joint['type']}"
        )

        print(
            f"  Parent: "
            f"{joint['parent']}"
        )

        print(
            f"  Child : "
            f"{joint['child']}"
        )

        print(
            "  Origin:",
            joint[
                "frame"
            ][
                "origin"
            ]
        )

        if joint["type"] == "revolute":

            print(
                "  Axis  :",
                joint[
                    "frame"
                ][
                    "axis"
                ]
            )

        else:

            print(
                "  RPY   :",
                joint[
                    "frame"
                ][
                    "rpy"
                ]
            )

    # ------------------------------------------------------------
    # Validation
    # ------------------------------------------------------------

    print()

    print(
        "-" * 72
    )

    print(
        "VALIDATION"
    )

    print(
        "-" * 72
    )

    print(
        "Status:",
        v10[
            "validation"
        ][
            "status"
        ]
    )

    warnings = v10[
        "validation"
    ][
        "warnings"
    ]

    if warnings:

        print()

        for warning in warnings:

            print(
                "WARNING:",
                warning
            )

    print()

    print(
        "=" * 72
    )


# ================================================================
# SAVE
# ================================================================

def save_model(v10):

    with open(
        OUTPUT_FILE,
        "w",
        encoding="utf-8"
    ) as f:

        json.dump(
            v10,
            f,
            indent=2
        )

    print()

    print(
        f"V10 saved to: "
        f"{OUTPUT_FILE}"
    )


# ================================================================
# MAIN
# ================================================================

def main():

    try:

        model = load_v9()

        print()

        print(
            "=" * 72
        )

        print(
            "LOADING V9 MODEL"
        )

        print(
            "=" * 72
        )

        print(
            "Input:",
            INPUT_FILE
        )

        print(
            "Version:",
            model.get(
                "version"
            )
        )

        v10 = process_model(
            model
        )

        print_summary(
            v10
        )

        save_model(
            v10
        )

    except Exception as e:

        print()

        print(
            "=" * 72
        )

        print(
            "V10 FAILED"
        )

        print(
            "=" * 72
        )

        print(
            type(e).__name__ +
            ":",
            e
        )

        raise


# ================================================================
# ENTRY POINT
# ================================================================

if __name__ == "__main__":

    main()