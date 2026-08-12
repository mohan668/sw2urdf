import json
import math
from pathlib import Path


# ============================================================
# FILES
# ============================================================

CAD_FILE = Path("robot_cad_data_v2.json")
V8_FILE = Path("kinematic_model_v8.json")
OUTPUT_FILE = Path("kinematic_model_v9.json")


# ============================================================
# SETTINGS
# ============================================================

EPS = 1e-9


# ============================================================
# VECTOR MATH
# ============================================================

def dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def norm(v):
    return math.sqrt(dot(v, v))


def normalize(v):
    n = norm(v)

    if n < EPS:
        raise ValueError(f"Cannot normalize zero vector: {v}")

    return [x / n for x in v]


def add(a, b):
    return [a[i] + b[i] for i in range(3)]


def sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def scale(v, s):
    return [x * s for x in v]


def mat_vec(R, v):
    return [
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    ]


def transpose(R):
    return [
        [R[0][0], R[1][0], R[2][0]],
        [R[0][1], R[1][1], R[2][1]],
        [R[0][2], R[1][2], R[2][2]],
    ]


def mat_mul(A, B):
    return [
        [
            sum(A[i][k] * B[k][j] for k in range(3))
            for j in range(3)
        ]
        for i in range(3)
    ]


# ============================================================
# JSON
# ============================================================

def load_json(path):

    if not path.exists():
        raise FileNotFoundError(
            f"File not found:\n{path.resolve()}"
        )

    with open(
        path,
        "r",
        encoding="utf-8"
    ) as f:

        return json.load(f)


def save_json(data, path):

    with open(
        path,
        "w",
        encoding="utf-8"
    ) as f:

        json.dump(
            data,
            f,
            indent=2
        )


# ============================================================
# CAD COMPONENT MAP
# ============================================================

def build_component_map(cad):

    result = {}

    for component in cad.get(
        "components",
        []
    ):

        name = component.get("name")

        if name:
            result[name] = component

    return result


# ============================================================
# CAD TRANSFORM
# ============================================================

def get_component_transform(component):

    t = component.get("transform")

    if not t or len(t) < 12:
        raise ValueError(
            f"Invalid transform for component: "
            f"{component.get('name')}"
        )

    R = [
        [t[0], t[1], t[2]],
        [t[3], t[4], t[5]],
        [t[6], t[7], t[8]],
    ]

    p = [
        t[9],
        t[10],
        t[11],
    ]

    return R, p


# ============================================================
# CAD → ROBOT COORDINATE CONVERSION
# ============================================================

def get_robot_matrix(v8):

    coordinate_system = v8.get(
        "coordinate_system",
        {}
    )

    matrix = coordinate_system.get(
        "cad_to_robot_matrix"
    )

    if matrix is None:

        # Identity fallback
        return [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]

    return matrix


def cad_point_to_robot(point, C):

    return mat_vec(C, point)


def cad_axis_to_robot(axis, C):

    return normalize(
        mat_vec(C, axis)
    )


# ============================================================
# FIND CONCENTRIC MATE
# ============================================================

def find_concentric_mates(
    cad,
    parent_name,
    child_name
):

    matches = []

    for mate in cad.get(
        "mates",
        []
    ):

        if mate.get(
            "typeName"
        ) != "MateConcentric":

            continue

        entities = mate.get(
            "entities",
            []
        )

        if len(entities) != 2:
            continue

        names = [
            entities[0].get("component"),
            entities[1].get("component")
        ]

        if (
            parent_name in names
            and
            child_name in names
        ):

            matches.append(
                mate
            )

    return matches


# ============================================================
# EXTRACT CONCENTRIC GEOMETRY
# ============================================================

def extract_concentric_geometry(
    mate,
    parent_name,
    child_name
):

    entities = mate.get(
        "entities",
        []
    )

    parent_entity = None
    child_entity = None

    for entity in entities:

        component = entity.get(
            "component"
        )

        if component == parent_name:
            parent_entity = entity

        elif component == child_name:
            child_entity = entity

    if (
        parent_entity is None
        or
        child_entity is None
    ):

        return None

    # --------------------------------------------------------
    # SolidWorks params:
    #
    # [0] X
    # [1] Y
    # [2] Z
    # [3] I
    # [4] J
    # [5] K
    # [6] Radius 1
    # [7] Radius 2
    # --------------------------------------------------------

    p = parent_entity.get(
        "params"
    )

    c = child_entity.get(
        "params"
    )

    if (
        not p
        or len(p) < 6
        or not c
        or len(c) < 6
    ):

        return None

    parent_point = [
        p[0],
        p[1],
        p[2]
    ]

    child_point = [
        c[0],
        c[1],
        c[2]
    ]

    parent_axis = normalize([
        p[3],
        p[4],
        p[5]
    ])

    child_axis = normalize([
        c[3],
        c[4],
        c[5]
    ])

    # Use the parent geometry as the canonical
    # joint axis. If the two axes point opposite
    # directions, this is still the same physical axis.

    axis = parent_axis

    # Average the two points.
    #
    # For a valid concentric mate they should lie
    # on the same physical axis. Averaging reduces
    # tiny numerical differences.
    origin = scale(
        add(
            parent_point,
            child_point
        ),
        0.5
    )

    return {

        "mate": mate.get("name"),

        "parent_point_cad": parent_point,

        "child_point_cad": child_point,

        "origin_cad": origin,

        "axis_parent_cad": parent_axis,

        "axis_child_cad": child_axis,

        "axis_cad": axis,

        "point_error_mm":
            norm(
                sub(
                    parent_point,
                    child_point
                )
            ),

        "axis_dot":
            dot(
                parent_axis,
                child_axis
            )
    }


# ============================================================
# RESOLVE JOINT FRAME
# ============================================================

def resolve_revolute_joint(
    joint,
    cad,
    C,
    component_map
):

    parent = joint["parent"]
    child = joint["child"]

    concentric_mates = find_concentric_mates(
        cad,
        parent,
        child
    )

    if not concentric_mates:

        return {

            "status": "UNRESOLVED",

            "reason":
                "No concentric mate found for "
                "revolute joint.",

            "origin_cad": None,
            "axis_cad": None,
            "origin_robot": None,
            "axis_robot": None,
            "origin_parent_robot": None,
            "axis_parent_robot": None
        }

    # --------------------------------------------------------
    # For now, use the first matching concentric mate.
    # V9 reports all available mates so we can later
    # add ambiguity detection.
    # --------------------------------------------------------

    geometry = extract_concentric_geometry(
        concentric_mates[0],
        parent,
        child
    )

    if geometry is None:

        return {

            "status": "UNRESOLVED",

            "reason":
                "Concentric mate exists but its "
                "geometry could not be extracted.",

            "origin_cad": None,
            "axis_cad": None,
            "origin_robot": None,
            "axis_robot": None,
            "origin_parent_robot": None,
            "axis_parent_robot": None
        }

    origin_cad = geometry[
        "origin_cad"
    ]

    axis_cad = geometry[
        "axis_cad"
    ]

    # --------------------------------------------------------
    # CAD → ROBOT
    # --------------------------------------------------------

    origin_robot = cad_point_to_robot(
        origin_cad,
        C
    )

    axis_robot = cad_axis_to_robot(
        axis_cad,
        C
    )

    # --------------------------------------------------------
    # Parent component frame
    # --------------------------------------------------------

    parent_component = component_map[
        parent
    ]

    parent_R_cad, parent_p_cad = \
        get_component_transform(
            parent_component
        )

    # --------------------------------------------------------
    # Parent-local origin
    #
    # p_parent_local =
    # R_parent^T * (p_joint - p_parent)
    # --------------------------------------------------------

    parent_R_T = transpose(
        parent_R_cad
    )

    origin_parent_cad = mat_vec(
        parent_R_T,
        sub(
            origin_cad,
            parent_p_cad
        )
    )

    axis_parent_cad = normalize(
        mat_vec(
            parent_R_T,
            axis_cad
        )
    )

    # --------------------------------------------------------
    # Convert parent-local CAD vector to robot vector.
    #
    # Since our robot coordinate system is defined
    # globally from CAD, use the global CAD→robot
    # transformation for the resulting vector.
    # --------------------------------------------------------

    origin_parent_robot = \
        cad_point_to_robot(
            origin_parent_cad,
            C
        )

    axis_parent_robot = \
        cad_axis_to_robot(
            axis_parent_cad,
            C
        )

    return {

        "status": "RESOLVED",

        "source_mate":
            geometry["mate"],

        "origin_cad":
            origin_cad,

        "axis_cad":
            axis_cad,

        "origin_robot":
            origin_robot,

        "axis_robot":
            axis_robot,

        "origin_parent_cad":
            origin_parent_cad,

        "axis_parent_cad":
            axis_parent_cad,

        "origin_parent_robot":
            origin_parent_robot,

        "axis_parent_robot":
            axis_parent_robot,

        "geometry_validation": {

            "point_error_mm":
                geometry[
                    "point_error_mm"
                ],

            "axis_dot":
                geometry[
                    "axis_dot"
                ]
        },

        "candidate_concentric_mates": [
            m.get("name")
            for m in concentric_mates
        ]
    }


# ============================================================
# RESOLVE FIXED JOINT
# ============================================================

def resolve_fixed_joint(
    joint,
    component_map,
    C
):

    parent = joint["parent"]
    child = joint["child"]

    parent_component = component_map[
        parent
    ]

    child_component = component_map[
        child
    ]

    parent_R, parent_p = \
        get_component_transform(
            parent_component
        )

    child_R, child_p = \
        get_component_transform(
            child_component
        )

    parent_R_T = transpose(
        parent_R
    )

    # Child origin in parent frame
    origin_parent_cad = mat_vec(
        parent_R_T,
        sub(
            child_p,
            parent_p
        )
    )

    # Child orientation in parent frame
    child_R_parent_cad = mat_mul(
        parent_R_T,
        child_R
    )

    origin_parent_robot = \
        cad_point_to_robot(
            origin_parent_cad,
            C
        )

    return {

        "status": "RESOLVED",

        "origin_cad":
            child_p,

        "origin_robot":
            cad_point_to_robot(
                child_p,
                C
            ),

        "origin_parent_cad":
            origin_parent_cad,

        "origin_parent_robot":
            origin_parent_robot,

        "child_rotation_in_parent_cad":
            child_R_parent_cad
    }


# ============================================================
# PROCESS MODEL
# ============================================================

def process_model(
    v8,
    cad
):

    component_map = \
        build_component_map(
            cad
        )

    C = get_robot_matrix(
        v8
    )

    result = dict(v8)

    result["version"] = "9.0"

    result[
        "v9_joint_frames"
    ] = []

    # --------------------------------------------------------
    # Header
    # --------------------------------------------------------

    print()
    print("=" * 72)
    print("SOLIDWORKS → URDF JOINT FRAME RESOLVER V9")
    print("=" * 72)

    print()
    print(
        f"CAD components: "
        f"{len(component_map)}"
    )

    print(
        f"Joints: "
        f"{len(v8.get('joints', []))}"
    )

    # --------------------------------------------------------
    # Process each joint
    # --------------------------------------------------------

    for joint in v8.get(
        "joints",
        []
    ):

        name = joint[
            "name"
        ]

        joint_type = joint[
            "type"
        ]

        parent = joint[
            "parent"
        ]

        child = joint[
            "child"
        ]

        print()
        print("-" * 72)

        print(
            f"JOINT: {name}"
        )

        print(
            f"TYPE: {joint_type}"
        )

        print(
            f"PARENT: {parent}"
        )

        print(
            f"CHILD: {child}"
        )

        # ----------------------------------------------------
        # Validate components
        # ----------------------------------------------------

        if parent not in component_map:

            raise ValueError(
                f"Parent CAD component not found: "
                f"{parent}"
            )

        if child not in component_map:

            raise ValueError(
                f"Child CAD component not found: "
                f"{child}"
            )

        # ----------------------------------------------------
        # Revolute
        # ----------------------------------------------------

        if joint_type == "revolute":

            frame = resolve_revolute_joint(
                joint,
                cad,
                C,
                component_map
            )

        # ----------------------------------------------------
        # Fixed
        # ----------------------------------------------------

        elif joint_type == "fixed":

            frame = resolve_fixed_joint(
                joint,
                component_map,
                C
            )

        # ----------------------------------------------------
        # Other joint types
        # ----------------------------------------------------

        else:

            frame = {

                "status":
                    "UNSUPPORTED",

                "reason":
                    f"Joint type '{joint_type}' "
                    f"is not resolved by V9."
            }

        # ----------------------------------------------------
        # Console diagnostics
        # ----------------------------------------------------

        print()

        print(
            f"STATUS: "
            f"{frame.get('status')}"
        )

        if frame.get(
            "source_mate"
        ):

            print(
                f"SOURCE MATE: "
                f"{frame['source_mate']}"
            )

        if frame.get(
            "origin_cad"
        ) is not None:

            print(
                "Origin CAD: ",
                frame[
                    "origin_cad"
                ]
            )

        if frame.get(
            "axis_cad"
        ) is not None:

            print(
                "Axis CAD:   ",
                frame[
                    "axis_cad"
                ]
            )

        if frame.get(
            "origin_robot"
        ) is not None:

            print(
                "Origin ROBOT:",
                frame[
                    "origin_robot"
                ]
            )

        if frame.get(
            "axis_robot"
        ) is not None:

            print(
                "Axis ROBOT:  ",
                frame[
                    "axis_robot"
                ]
            )

        if frame.get(
            "origin_parent_robot"
        ) is not None:

            print(
                "Origin PARENT:",
                frame[
                    "origin_parent_robot"
                ]
            )

        if frame.get(
            "axis_parent_robot"
        ) is not None:

            print(
                "Axis PARENT:  ",
                frame[
                    "axis_parent_robot"
                ]
            )

        if "geometry_validation" in frame:

            validation = frame[
                "geometry_validation"
            ]

            print(
                "Point error:",
                validation[
                    "point_error_mm"
                ],
                "mm"
            )

            print(
                "Axis dot:",
                validation[
                    "axis_dot"
                ]
            )

        # ----------------------------------------------------
        # Store
        # ----------------------------------------------------

        result[
            "v9_joint_frames"
        ].append({

            "joint":
                name,

            "type":
                joint_type,

            "parent":
                parent,

            "child":
                child,

            "frame":
                frame
        })

    # --------------------------------------------------------
    # V9 metadata
    # --------------------------------------------------------

    result[
        "v9_processing"
    ] = {

        "description":
            "Resolves true joint frames from "
            "SolidWorks mate geometry.",

        "joint_origin_source":
            "SolidWorks Concentric mate geometry "
            "for revolute joints.",

        "joint_axis_source":
            "SolidWorks Concentric mate geometry "
            "for revolute joints.",

        "fixed_joint_source":
            "SolidWorks component transforms.",

        "coordinate_conversion":
            "CAD to robot coordinate matrix from V7/V8.",

        "mass_properties":
            False,

        "mesh_information":
            False
    }

    return result


# ============================================================
# MAIN
# ============================================================

def main():

    print("=" * 72)
    print("V9 START")
    print("=" * 72)

    print()
    print(
        "Loading V8 model..."
    )

    v8 = load_json(
        V8_FILE
    )

    print(
        f"V8 version: "
        f"{v8.get('version')}"
    )

    print()
    print(
        "Loading SolidWorks CAD data..."
    )

    cad = load_json(
        CAD_FILE
    )

    print(
        f"CAD components: "
        f"{len(cad.get('components', []))}"
    )

    result = process_model(
        v8,
        cad
    )

    save_json(
        result,
        OUTPUT_FILE
    )

    print()
    print("=" * 72)
    print("V9 COMPLETE")
    print("=" * 72)

    print()
    print(
        f"Output: "
        f"{OUTPUT_FILE.resolve()}"
    )


if __name__ == "__main__":
    main()