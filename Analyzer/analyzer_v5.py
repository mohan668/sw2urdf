import json
import math
from pathlib import Path
from collections import defaultdict, deque


# ============================================================
# FILES
# ============================================================

INPUT_CAD = Path("robot_cad_data_v2.json")
INPUT_KINEMATIC = Path("kinematic_model.json")
OUTPUT_FILE = Path("kinematic_model_v5.json")

TOLERANCE = 1e-9


# ============================================================
# VECTOR MATH
# ============================================================

def dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def norm(v):
    return math.sqrt(dot(v, v))


def normalize(v):
    n = norm(v)

    if n < TOLERANCE:
        return [0.0, 0.0, 0.0]

    return [x / n for x in v]


def subtract(a, b):
    return [a[i] - b[i] for i in range(3)]


def round_vec(v, digits=12):
    return [round(x, digits) for x in v]


# ============================================================
# MATRIX MATH
# ============================================================

def mat3_from_axes(axes):
    """
    Columns of the rotation matrix are the component's
    local X/Y/Z axes expressed in assembly coordinates.
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


def mat_mul(a, b):
    result = [[0.0] * 3 for _ in range(3)]

    for i in range(3):
        for j in range(3):
            for k in range(3):
                result[i][j] += (
                    a[i][k] * b[k][j]
                )

    return result


# ============================================================
# COMPONENT FRAME CONVERSIONS
# ============================================================

def assembly_point_to_local(point, component):

    origin = component["assembly_origin"]

    axes = component[
        "local_axes_in_assembly"
    ]

    rotation = mat3_from_axes(axes)

    relative = subtract(
        point,
        origin
    )

    return mat_vec(
        transpose(rotation),
        relative
    )


def assembly_axis_to_local(axis, component):

    axes = component[
        "local_axes_in_assembly"
    ]

    rotation = mat3_from_axes(axes)

    return normalize(
        mat_vec(
            transpose(rotation),
            axis
        )
    )


# ============================================================
# LOAD JSON
# ============================================================

def load_json(path):

    if not path.exists():
        raise FileNotFoundError(
            f"Could not find: {path}"
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

def build_component_lookup(data):

    return {
        component["name"]: component
        for component in data["components"]
    }


# ============================================================
# CONNECTION GRAPH
# ============================================================

def build_graph(connections):

    graph = defaultdict(list)

    for connection in connections:

        a = connection[
            "parent_candidate"
        ]

        b = connection[
            "child_candidate"
        ]

        graph[a].append(
            (b, connection)
        )

        graph[b].append(
            (a, connection)
        )

    return graph


# ============================================================
# FIND ROOT
# ============================================================

def find_root(cad_data, kinematic_data):

    roots = kinematic_data.get(
        "root_components",
        []
    )

    if roots:
        return roots[0]

    for component in cad_data["components"]:

        if component.get(
            "fixed",
            False
        ):

            return component["name"]

    raise RuntimeError(
        "No fixed SolidWorks root found."
    )


# ============================================================
# BUILD TREE
# ============================================================

def build_tree(root, graph):

    visited = {root}

    queue = deque([root])

    tree = []

    while queue:

        current = queue.popleft()

        for neighbor, connection in graph[current]:

            if neighbor in visited:
                continue

            visited.add(neighbor)

            queue.append(neighbor)

            tree.append({
                "parent": current,
                "child": neighbor,
                "connection": connection
            })

    return tree


# ============================================================
# CLASSIFICATION
# ============================================================

def classify_joint(connection):

    mates = connection["mates"]

    mate_types = [
        mate["type"]
        for mate in mates
    ]

    has_lock = (
        "MateLock"
        in mate_types
    )

    has_concentric = (
        "MateConcentric"
        in mate_types
    )

    has_coincident = (
        "MateCoincident"
        in mate_types
    )

    # --------------------------------------------------------
    # LOCK ALWAYS WINS
    # --------------------------------------------------------

    if has_lock:

        return {
            "type": "fixed",
            "confidence": "HIGH",
            "reason":
                "Lock mate explicitly removes "
                "relative motion."
        }

    # --------------------------------------------------------
    # CONCENTRIC + COINCIDENT
    # --------------------------------------------------------

    if (
        has_concentric
        and has_coincident
    ):

        return {
            "type": "revolute",
            "confidence": "HIGH",
            "reason":
                "Concentric + Coincident with "
                "no Lock indicates one rotational DOF "
                "under the current exporter rules."
        }

    # --------------------------------------------------------
    # CONCENTRIC ONLY
    # --------------------------------------------------------

    if has_concentric:

        return {
            "type": "cylindrical_candidate",
            "confidence": "LOW",
            "reason":
                "Concentric without Coincident "
                "or Lock."
        }

    # --------------------------------------------------------
    # OTHER
    # --------------------------------------------------------

    return {
        "type": "unknown",
        "confidence": "LOW",
        "reason":
            "Constraint combination is not "
            "currently understood."
    }


# ============================================================
# CONCENTRIC GEOMETRY
# ============================================================

def get_concentric_geometry(connection):

    geometries = []

    for mate in connection["mates"]:

        if mate["type"] != "MateConcentric":
            continue

        # Find the original mate in the CAD data
        # through the connection's embedded geometry.

    for geometry in connection.get(
        "concentric_geometry",
        []
    ):

        if not geometry:
            continue

        geometries.append(geometry)

    return geometries


# ============================================================
# REVOLUTE FRAME
# ============================================================

def build_revolute_frame(
    connection,
    parent,
    child
):

    geometries = get_concentric_geometry(
        connection
    )

    if not geometries:

        return {
            "origin_assembly": None,
            "axis_assembly": None,
            "origin_parent": None,
            "axis_parent": None,
            "origin_child": None,
            "axis_child": None
        }

    geometry = geometries[0]

    origin = geometry[
        "point_a"
    ]

    axis = normalize(
        geometry["axis_a"]
    )

    return {
        "origin_assembly":
            round_vec(origin),

        "axis_assembly":
            round_vec(axis),

        "origin_parent":
            round_vec(
                assembly_point_to_local(
                    origin,
                    parent
                )
            ),

        "axis_parent":
            round_vec(
                assembly_axis_to_local(
                    axis,
                    parent
                )
            ),

        "origin_child":
            round_vec(
                assembly_point_to_local(
                    origin,
                    child
                )
            ),

        "axis_child":
            round_vec(
                assembly_axis_to_local(
                    axis,
                    child
                )
            )
    }


# ============================================================
# FIXED JOINT FRAME
# ============================================================

def build_fixed_frame(
    parent,
    child
):

    """
    For a fixed connection, there is no concentric axis
    to use as the joint frame.

    We therefore use the CHILD component's assembly
    frame as the fixed joint frame.

    The origin is expressed relative to the parent.
    The child's local axes are also expressed relative
    to the parent.

    This gives us a complete rigid transform between
    parent and child.
    """

    child_origin = child[
        "assembly_origin"
    ]

    child_axes = child[
        "local_axes_in_assembly"
    ]

    parent_origin = parent[
        "assembly_origin"
    ]

    parent_axes = parent[
        "local_axes_in_assembly"
    ]

    # Child origin expressed in parent coordinates
    origin_parent = \
        assembly_point_to_local(
            child_origin,
            parent
        )

    # Child local axes expressed in parent coordinates
    axis_x_parent = \
        assembly_axis_to_local(
            child_axes["x"],
            parent
        )

    axis_y_parent = \
        assembly_axis_to_local(
            child_axes["y"],
            parent
        )

    axis_z_parent = \
        assembly_axis_to_local(
            child_axes["z"],
            parent
        )

    return {

        "origin_assembly":
            round_vec(
                child_origin
            ),

        "axis_assembly":
            None,

        "origin_parent":
            round_vec(
                origin_parent
            ),

        "axis_parent":
            None,

        "origin_child":
            [0.0, 0.0, 0.0],

        "axis_child":
            None,

        "child_frame_in_parent": {
            "x": round_vec(
                axis_x_parent
            ),

            "y": round_vec(
                axis_y_parent
            ),

            "z": round_vec(
                axis_z_parent
            )
        }
    }


# ============================================================
# BUILD JOINT FRAME
# ============================================================

def build_joint_frame(
    joint_type,
    connection,
    parent,
    child
):

    if joint_type == "revolute":

        return build_revolute_frame(
            connection,
            parent,
            child
        )

    if joint_type == "fixed":

        return build_fixed_frame(
            parent,
            child
        )

    # Unknown / future joint types
    return {
        "origin_assembly": None,
        "axis_assembly": None,
        "origin_parent": None,
        "axis_parent": None,
        "origin_child": None,
        "axis_child": None
    }


# ============================================================
# BUILD LINKS
# ============================================================

def build_links(
    root,
    tree
):

    links = []

    used = set()

    def add(name):

        if name in used:
            return

        used.add(name)

        links.append({
            "name": name,
            "cad_component": name
        })

    add(root)

    for item in tree:

        add(item["parent"])
        add(item["child"])

    return links


# ============================================================
# BUILD JOINTS
# ============================================================

def build_joints(
    tree,
    components
):

    joints = []

    for index, item in enumerate(
        tree,
        start=1
    ):

        parent_name = item[
            "parent"
        ]

        child_name = item[
            "child"
        ]

        connection = item[
            "connection"
        ]

        parent = components[
            parent_name
        ]

        child = components[
            child_name
        ]

        classification = \
            classify_joint(
                connection
            )

        joint_type = \
            classification["type"]

        frame = build_joint_frame(
            joint_type,
            connection,
            parent,
            child
        )

        joints.append({

            "name":
                f"joint_{index}",

            "type":
                joint_type,

            "parent":
                parent_name,

            "child":
                child_name,

            "classification":
                classification,

            "frame":
                frame,

            "source_mates":
                connection["mates"]
        })

    return joints


# ============================================================
# BUILD MODEL
# ============================================================

def build_model(
    cad_data,
    kinematic_data
):

    components = \
        build_component_lookup(
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
        root,
        tree
    )

    joints = build_joints(
        tree,
        components
    )

    return {

        "format":
            "sw2urdf_kinematic_model",

        "version":
            "5.0",

        "coordinate_system": {

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

        "tree": [
            {
                "parent":
                    item["parent"],

                "child":
                    item["child"],

                "joint":
                    f"joint_{index}"
            }

            for index, item
            in enumerate(
                tree,
                start=1
            )
        ]
    }


# ============================================================
# REPORT
# ============================================================

def print_report(model):

    print()
    print("=" * 70)
    print("SOLIDWORKS KINEMATIC ANALYZER V5")
    print("=" * 70)

    print()
    print(
        f"ROOT LINK: {model['root']}"
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
    print("JOINT ANALYSIS")
    print("=" * 70)

    for joint in model["joints"]:

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

        classification = \
            joint["classification"]

        print(
            f"  Confidence: "
            f"{classification['confidence']}"
        )

        print(
            f"  Reason: "
            f"{classification['reason']}"
        )

        frame = joint["frame"]

        print()

        print(
            "  Origin in assembly:"
        )

        print(
            f"    "
            f"{frame['origin_assembly']}"
        )

        print(
            "  Origin in parent:"
        )

        print(
            f"    "
            f"{frame['origin_parent']}"
        )

        if frame[
            "axis_assembly"
        ] is not None:

            print()

            print(
                "  Axis in assembly:"
            )

            print(
                f"    "
                f"{frame['axis_assembly']}"
            )

            print(
                "  Axis in parent:"
            )

            print(
                f"    "
                f"{frame['axis_parent']}"
            )

        if (
            "child_frame_in_parent"
            in frame
        ):

            print()

            print(
                "  Fixed child frame "
                "relative to parent:"
            )

            fixed_frame = frame[
                "child_frame_in_parent"
            ]

            print(
                f"    X: "
                f"{fixed_frame['x']}"
            )

            print(
                f"    Y: "
                f"{fixed_frame['y']}"
            )

            print(
                f"    Z: "
                f"{fixed_frame['z']}"
            )

    print()
    print("=" * 70)
    print("V5 COMPLETE")
    print("=" * 70)

    print(
        f"Output: {OUTPUT_FILE}"
    )


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

    model = build_model(
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