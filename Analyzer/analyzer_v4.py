import json
import math
from pathlib import Path
from collections import defaultdict, deque


# ============================================================
# FILES
# ============================================================

INPUT_CAD = Path("robot_cad_data_v2.json")
INPUT_KINEMATIC = Path("kinematic_model.json")
OUTPUT_FILE = Path("kinematic_model_v4.json")


# ============================================================
# TOLERANCE
# ============================================================

TOLERANCE = 1e-9


# ============================================================
# VECTOR FUNCTIONS
# ============================================================

def dot(a, b):
    return sum(
        a[i] * b[i]
        for i in range(3)
    )


def norm(v):
    return math.sqrt(
        dot(v, v)
    )


def normalize(v):

    n = norm(v)

    if n < TOLERANCE:
        return [0.0, 0.0, 0.0]

    return [
        x / n
        for x in v
    ]


def subtract(a, b):

    return [
        a[i] - b[i]
        for i in range(3)
    ]


def add(a, b):

    return [
        a[i] + b[i]
        for i in range(3)
    ]


def scale(v, s):

    return [
        x * s
        for x in v
    ]


def cross(a, b):

    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    ]


def round_vec(v, digits=12):

    return [
        round(x, digits)
        for x in v
    ]


# ============================================================
# MATRIX FUNCTIONS
# ============================================================

def mat3_from_axes(axes):

    """
    Construct rotation matrix whose columns are
    the local X, Y, Z axes expressed in assembly coordinates.
    """

    x = axes["x"]
    y = axes["y"]
    z = axes["z"]

    return [
        [x[0], y[0], z[0]],
        [x[1], y[1], z[1]],
        [x[2], y[2], z[2]]
    ]


def transpose(m):

    return [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]]
    ]


def mat_vec(m, v):

    return [
        m[0][0] * v[0] +
        m[0][1] * v[1] +
        m[0][2] * v[2],

        m[1][0] * v[0] +
        m[1][1] * v[1] +
        m[1][2] * v[2],

        m[2][0] * v[0] +
        m[2][1] * v[1] +
        m[2][2] * v[2]
    ]


# ============================================================
# COORDINATE CONVERSION
# ============================================================

def assembly_point_to_local(
    point_assembly,
    component
):

    """
    Convert an assembly-coordinate point into
    the local coordinate frame of a component.
    """

    origin = component[
        "assembly_origin"
    ]

    axes = component[
        "local_axes_in_assembly"
    ]

    rotation = mat3_from_axes(
        axes
    )

    rotation_T = transpose(
        rotation
    )

    relative = subtract(
        point_assembly,
        origin
    )

    return mat_vec(
        rotation_T,
        relative
    )


def assembly_axis_to_local(
    axis_assembly,
    component
):

    """
    Convert an assembly-coordinate direction
    into a component's local coordinate frame.
    """

    axes = component[
        "local_axes_in_assembly"
    ]

    rotation = mat3_from_axes(
        axes
    )

    rotation_T = transpose(
        rotation
    )

    return normalize(
        mat_vec(
            rotation_T,
            axis_assembly
        )
    )


# ============================================================
# LOAD DATA
# ============================================================

def load_json(path):

    if not path.exists():

        raise FileNotFoundError(
            f"Could not find {path}"
        )

    with open(
        path,
        "r",
        encoding="utf-8"
    ) as f:

        return json.load(f)


# ============================================================
# COMPONENT LOOKUP
# ============================================================

def component_lookup(cad_data):

    return {
        component["name"]: component
        for component
        in cad_data["components"]
    }


# ============================================================
# CONNECTION GRAPH
# ============================================================

def build_graph(
    connections
):

    graph = defaultdict(list)

    for connection in connections:

        a = connection[
            "parent_candidate"
        ]

        b = connection[
            "child_candidate"
        ]

        graph[a].append(
            (
                b,
                connection
            )
        )

        graph[b].append(
            (
                a,
                connection
            )
        )

    return graph


# ============================================================
# ROOT
# ============================================================

def find_root(
    cad_data,
    kinematic_data
):

    roots = kinematic_data[
        "root_components"
    ]

    if roots:

        return roots[0]

    for component in cad_data[
        "components"
    ]:

        if component.get(
            "fixed",
            False
        ):

            return component["name"]

    raise RuntimeError(
        "No fixed root component found."
    )


# ============================================================
# TREE TRAVERSAL
# ============================================================

def build_tree(
    root,
    graph
):

    visited = set()

    ordered_connections = []

    queue = deque()

    queue.append(root)

    visited.add(root)

    while queue:

        current = queue.popleft()

        for (
            neighbor,
            connection
        ) in graph[current]:

            if neighbor in visited:
                continue

            visited.add(
                neighbor
            )

            queue.append(
                neighbor
            )

            ordered_connections.append(
                {
                    "parent":
                        current,

                    "child":
                        neighbor,

                    "connection":
                        connection
                }
            )

    return ordered_connections


# ============================================================
# FIND JOINT AXIS
# ============================================================

def get_joint_axis(
    connection
):

    geometry = connection.get(
        "concentric_geometry",
        []
    )

    if not geometry:

        return None

    axis = geometry[0].get(
        "axis"
    )

    if axis is None:

        return None

    return normalize(
        axis
    )


# ============================================================
# FIND JOINT ORIGIN
# ============================================================

def get_joint_origin(
    connection
):

    origin = connection.get(
        "joint_origin_cad"
    )

    if origin is None:

        return None

    return origin


# ============================================================
# DETERMINE JOINT TYPE
# ============================================================

def determine_joint_type(
    connection
):

    classification = \
        connection[
            "classification"
        ]

    ctype = classification[
        "type"
    ]

    if ctype == "fixed_root":

        return "fixed"

    if ctype == "fixed":

        return "fixed"

    if ctype == \
            "revolute_candidate":

        return "revolute"

    if ctype == \
            "cylindrical_candidate":

        return "cylindrical"

    return "unknown"


# ============================================================
# BUILD JOINT FRAME
# ============================================================

def build_joint_frame(
    parent_name,
    child_name,
    connection,
    components
):

    parent = components[
        parent_name
    ]

    child = components[
        child_name
    ]

    joint_origin = \
        get_joint_origin(
            connection
        )

    joint_axis = \
        get_joint_axis(
            connection
        )

    result = {

        "origin_assembly": None,

        "axis_assembly": None,

        "origin_parent": None,

        "axis_parent": None,

        "origin_child": None,

        "axis_child": None
    }

    if joint_origin is not None:

        result[
            "origin_assembly"
        ] = round_vec(
            joint_origin
        )

        result[
            "origin_parent"
        ] = round_vec(
            assembly_point_to_local(
                joint_origin,
                parent
            )
        )

        result[
            "origin_child"
        ] = round_vec(
            assembly_point_to_local(
                joint_origin,
                child
            )
        )

    if joint_axis is not None:

        result[
            "axis_assembly"
        ] = round_vec(
            joint_axis
        )

        result[
            "axis_parent"
        ] = round_vec(
            assembly_axis_to_local(
                joint_axis,
                parent
            )
        )

        result[
            "axis_child"
        ] = round_vec(
            assembly_axis_to_local(
                joint_axis,
                child
            )
        )

    return result


# ============================================================
# BUILD LINK DATA
# ============================================================

def build_links(
    cad_data,
    tree
):

    components = component_lookup(
        cad_data
    )

    links = []

    used = set()

    for item in tree:

        parent = item[
            "parent"
        ]

        child = item[
            "child"
        ]

        if parent not in used:

            links.append(
                {
                    "name": parent,
                    "cad_component":
                        parent
                }
            )

            used.add(parent)

        if child not in used:

            links.append(
                {
                    "name": child,
                    "cad_component":
                        child
                }
            )

            used.add(child)

    return links


# ============================================================
# BUILD JOINTS
# ============================================================

def build_joints(
    cad_data,
    tree
):

    components = component_lookup(
        cad_data
    )

    joints = []

    joint_number = 1

    for item in tree:

        parent = item[
            "parent"
        ]

        child = item[
            "child"
        ]

        connection = item[
            "connection"
        ]

        joint_type = \
            determine_joint_type(
                connection
            )

        frame = build_joint_frame(
            parent,
            child,
            connection,
            components
        )

        joints.append(
            {
                "name":
                    f"joint_{joint_number}",

                "type":
                    joint_type,

                "parent":
                    parent,

                "child":
                    child,

                "frame":
                    frame,

                "source_mates":
                    connection["mates"],

                "classification":
                    connection[
                        "classification"
                    ]
            }
        )

        joint_number += 1

    return joints


# ============================================================
# BUILD V4 MODEL
# ============================================================

def build_v4(
    cad_data,
    kinematic_data
):

    components = component_lookup(
        cad_data
    )

    graph = build_graph(
        kinematic_data[
            "connections"
        ]
    )

    root = find_root(
        cad_data,
        kinematic_data
    )

    tree = build_tree(
        root,
        graph
    )

    links = build_links(
        cad_data,
        tree
    )

    joints = build_joints(
        cad_data,
        tree
    )

    return {

        "format":
            "sw2urdf_kinematic_model",

        "version":
            "4.0",

        "coordinate_system":
            {
                "source":
                    "SolidWorks assembly",

                "conversion_applied":
                    False,

                "note":
                    "CAD-to-robot coordinate "
                    "conversion is intentionally "
                    "deferred."
            },

        "root":
            root,

        "links":
            links,

        "joints":
            joints,

        "tree":
            [
                {
                    "parent":
                        item["parent"],

                    "child":
                        item["child"],

                    "joint":
                        f"joint_{i + 1}"
                }

                for i, item
                in enumerate(tree)
            ]
    }


# ============================================================
# PRINT REPORT
# ============================================================

def print_report(model):

    print()
    print("=" * 70)
    print("SOLIDWORKS KINEMATIC ANALYZER V4")
    print("=" * 70)

    print()

    print(
        f"ROOT: {model['root']}"
    )

    print()

    print("=" * 70)
    print("KINEMATIC TREE")
    print("=" * 70)

    for item in model["tree"]:

        print(
            f"{item['parent']}"
        )

        print(
            f"    |"
        )

        print(
            f"    +-- "
            f"{item['joint']}"
        )

        print(
            f"    |"
        )

        print(
            f"    v"
        )

        print(
            f"{item['child']}"
        )

        print()

    print("=" * 70)
    print("JOINT FRAMES")
    print("=" * 70)

    for joint in model[
        "joints"
    ]:

        print()

        print(
            f"{joint['name']}"
        )

        print(
            f"  Type: "
            f"{joint['type']}"
        )

        print(
            f"  Parent: "
            f"{joint['parent']}"
        )

        print(
            f"  Child: "
            f"{joint['child']}"
        )

        frame = joint[
            "frame"
        ]

        print()

        print(
            "  Origin in assembly:"
        )

        print(
            f"    "
            f"{frame['origin_assembly']}"
        )

        print(
            "  Axis in assembly:"
        )

        print(
            f"    "
            f"{frame['axis_assembly']}"
        )

        print()

        print(
            "  Origin in parent frame:"
        )

        print(
            f"    "
            f"{frame['origin_parent']}"
        )

        print(
            "  Axis in parent frame:"
        )

        print(
            f"    "
            f"{frame['axis_parent']}"
        )

        print()

        print(
            "  Origin in child frame:"
        )

        print(
            f"    "
            f"{frame['origin_child']}"
        )

        print(
            "  Axis in child frame:"
        )

        print(
            f"    "
            f"{frame['axis_child']}"
        )

    print()

    print("=" * 70)
    print(
        "V4 MODEL SAVED:"
    )

    print(
        f"  {OUTPUT_FILE}"
    )

    print("=" * 70)


# ============================================================
# MAIN
# ============================================================

def main():

    cad_data = load_json(
        INPUT_CAD
    )

    kinematic_data = load_json(
        INPUT_KINEMATIC
    )

    model = build_v4(
        cad_data,
        kinematic_data
    )

    with open(
        OUTPUT_FILE,
        "w",
        encoding="utf-8"
    ) as f:

        json.dump(
            model,
            f,
            indent=2
        )

    print_report(
        model
    )


if __name__ == "__main__":

    main()