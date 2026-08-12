import json
import math
from pathlib import Path


# ============================================================
# FILES
# ============================================================

CAD_FILE = Path("robot_cad_data_v2.json")
V7_FILE = Path("kinematic_model_v7.json")
OUTPUT_FILE = Path("kinematic_model_v8.json")


# ============================================================
# BASIC MATH
# ============================================================

def dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def norm(v):
    return math.sqrt(dot(v, v))


def normalize(v):
    n = norm(v)

    if n < 1e-12:
        raise ValueError(f"Cannot normalize zero vector: {v}")

    return [x / n for x in v]


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


def sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def add(a, b):
    return [a[i] + b[i] for i in range(3)]


# ============================================================
# SOLIDWORKS TRANSFORM
# ============================================================

def extract_transform(component):
    """
    SolidWorks transform layout used by our CAD extractor:

    [0..8]   rotation
    [9..11]  translation
    """

    t = component.get("transform")

    if not t or len(t) < 12:
        raise ValueError(
            f"Missing or invalid transform for "
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
# FRAME CONVERSION
# ============================================================

def relative_frame(parent_R, parent_p, child_R, child_p):
    """
    Calculate child frame expressed in parent coordinates.

    p_rel = R_parent^T * (p_child - p_parent)

    R_rel = R_parent^T * R_child
    """

    R_parent_T = transpose(parent_R)

    p_rel = mat_vec(
        R_parent_T,
        sub(child_p, parent_p)
    )

    R_rel = mat_mul(
        R_parent_T,
        child_R
    )

    return R_rel, p_rel


# ============================================================
# LOAD JSON
# ============================================================

def load_json(path):
    print(f"Loading: {path}")

    if not path.exists():
        raise FileNotFoundError(
            f"File not found: {path}"
        )

    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


# ============================================================
# CAD COMPONENT MAP
# ============================================================

def build_cad_component_map(cad):
    components = {}

    for component in cad.get("components", []):

        name = component.get("name")

        if not name:
            continue

        components[name] = component

    return components


# ============================================================
# V7 LINK MAP
# ============================================================

def build_link_map(v7):
    links = {}

    for link in v7.get("links", []):

        name = link.get("cad_component")

        if not name:
            name = link.get("name")

        if name:
            links[name] = link

    return links


# ============================================================
# FIND CAD COMPONENT
# ============================================================

def require_component(cad_components, name):

    if name not in cad_components:

        available = "\n".join(
            f"    - {x}"
            for x in cad_components.keys()
        )

        raise ValueError(
            f"\nCAD component not found: {name}\n"
            f"Available CAD components:\n"
            f"{available}"
        )

    return cad_components[name]


# ============================================================
# PROCESS JOINT
# ============================================================

def process_joint(
    joint,
    cad_components
):

    parent = joint["parent"]
    child = joint["child"]

    parent_component = require_component(
        cad_components,
        parent
    )

    child_component = require_component(
        cad_components,
        child
    )

    parent_R, parent_p = extract_transform(
        parent_component
    )

    child_R, child_p = extract_transform(
        child_component
    )

    relative_R, relative_p = relative_frame(
        parent_R,
        parent_p,
        child_R,
        child_p
    )

    result = dict(joint)

    result["v8_frame"] = {

        "parent_to_child_origin": [
            round(x, 12)
            for x in relative_p
        ],

        "child_frame_rotation_in_parent": [
            [
                round(x, 12)
                for x in row
            ]
            for row in relative_R
        ]
    }

    return result


# ============================================================
# PROCESS MODEL
# ============================================================

def process_model(v7, cad):

    cad_components = build_cad_component_map(cad)

    if not cad_components:
        raise ValueError(
            "No CAD components found in "
            "robot_cad_data_v2.json"
        )

    print()
    print("=" * 72)
    print("V8 CAD + KINEMATIC MODEL MERGER")
    print("=" * 72)

    print()
    print(
        f"CAD components found: "
        f"{len(cad_components)}"
    )

    print(
        f"V7 links found: "
        f"{len(v7.get('links', []))}"
    )

    print(
        f"V7 joints found: "
        f"{len(v7.get('joints', []))}"
    )

    # --------------------------------------------------------
    # Verify every V7 link exists in CAD
    # --------------------------------------------------------

    print()
    print("VERIFYING LINK ↔ CAD COMPONENT MATCHES")
    print("-" * 72)

    for link in v7.get("links", []):

        name = link.get(
            "cad_component",
            link.get("name")
        )

        if name in cad_components:

            print(
                f"[OK] {name}"
            )

        else:

            print(
                f"[ERROR] {name}"
            )

            raise ValueError(
                f"V7 link has no matching CAD component: "
                f"{name}"
            )

    # --------------------------------------------------------
    # Process joints
    # --------------------------------------------------------

    processed_joints = []

    print()
    print("CALCULATING PARENT-LOCAL FRAMES")
    print("-" * 72)

    for joint in v7.get("joints", []):

        parent = joint["parent"]
        child = joint["child"]

        print()
        print(
            f"{joint['name']}: "
            f"{parent} -> {child}"
        )

        processed = process_joint(
            joint,
            cad_components
        )

        frame = processed["v8_frame"]

        print(
            "  Relative origin:",
            frame["parent_to_child_origin"]
        )

        processed_joints.append(
            processed
        )

    # --------------------------------------------------------
    # Build V8
    # --------------------------------------------------------

    v8 = {

        "format":
            "sw2urdf_kinematic_model",

        "version":
            "8.0",

        "coordinate_system":
            v7.get(
                "coordinate_system",
                {}
            ),

        "root":
            v7.get("root"),

        "links":
            v7.get("links", []),

        "joints":
            processed_joints,

        "tree":
            v7.get("tree", []),

        "source_files": {

            "cad_data":
                str(CAD_FILE),

            "kinematic_model":
                str(V7_FILE)
        },

        "v8_processing": {

            "description":
                "V7 kinematic structure merged with "
                "SolidWorks CAD component transforms.",

            "relative_frames_calculated":
                True,

            "mass_properties":
                False,

            "mesh_information":
                False
        }
    }

    return v8


# ============================================================
# SAVE
# ============================================================

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

    print()
    print("=" * 72)
    print(
        f"V8 OUTPUT SAVED:"
    )
    print(
        f"{path.resolve()}"
    )
    print("=" * 72)


# ============================================================
# MAIN
# ============================================================

def main():

    print("=" * 72)
    print("SOLIDWORKS → URDF ANALYZER V8")
    print("=" * 72)

    print()
    print("Loading V7 kinematic model...")

    v7 = load_json(V7_FILE)

    print(
        f"Assembly: "
        f"{v7.get('source_assembly', {}).get('name', 'Unknown')}"
    )

    print(
        f"Input version: "
        f"{v7.get('version', 'Unknown')}"
    )

    print()
    print("Loading SolidWorks CAD data...")

    cad = load_json(CAD_FILE)

    print(
        f"CAD assembly: "
        f"{cad.get('assembly', {}).get('name', 'Unknown')}"
    )

    print(
        f"CAD components: "
        f"{len(cad.get('components', []))}"
    )

    v8 = process_model(
        v7,
        cad
    )

    save_json(
        v8,
        OUTPUT_FILE
    )


if __name__ == "__main__":
    main()