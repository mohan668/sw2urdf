import json
import math
from pathlib import Path
from collections import defaultdict


# ============================================================
# CONFIGURATION
# ============================================================

INPUT_FILE = Path("robot_cad_data_v2.json")
OUTPUT_FILE = Path("kinematic_model.json")

TOLERANCE = 1e-6


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


def distance(a, b):
    return norm(subtract(a, b))


def round_vec(v, digits=12):
    return [round(x, digits) for x in v]


# ============================================================
# LOAD DATA
# ============================================================

def load_cad_data():

    if not INPUT_FILE.exists():
        raise FileNotFoundError(
            f"Could not find {INPUT_FILE}"
        )

    with open(
        INPUT_FILE,
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
# GROUP MATES
# ============================================================

def component_pair(a, b):
    return tuple(sorted([a, b]))


def group_mates(data):

    groups = defaultdict(list)

    for mate in data["mates"]:

        entities = mate.get("entities", [])

        if len(entities) < 2:
            continue

        a = entities[0]["component"]
        b = entities[1]["component"]

        groups[
            component_pair(a, b)
        ].append(mate)

    return groups


# ============================================================
# FIND CONCENTRIC AXIS
# ============================================================

def get_concentric_axis(mates):

    for mate in mates:

        if mate["typeName"] != "MateConcentric":
            continue

        entities = mate.get("entities", [])

        if len(entities) < 2:
            continue

        entity = entities[0]

        params = entity.get("params")

        if not params or len(params) < 6:
            continue

        axis = normalize(
            params[3:6]
        )

        point = params[0:3]

        return {
            "axis": axis,
            "point": point,
            "mate_name": mate["name"],
        }

    return None


# ============================================================
# FIND CONCENTRIC GEOMETRY DETAILS
# ============================================================

def analyze_concentric(mates):

    results = []

    for mate in mates:

        if mate["typeName"] != "MateConcentric":
            continue

        entities = mate.get("entities", [])

        if len(entities) < 2:
            continue

        a = entities[0]
        b = entities[1]

        pa = a.get("params")
        pb = b.get("params")

        if not pa or not pb:
            continue

        point_a = pa[0:3]
        point_b = pb[0:3]

        axis_a = normalize(
            pa[3:6]
        )

        axis_b = normalize(
            pb[3:6]
        )

        results.append({
            "mate": mate["name"],

            "component_a":
                a["component"],

            "component_b":
                b["component"],

            "point_a":
                point_a,

            "point_b":
                point_b,

            "axis_a":
                axis_a,

            "axis_b":
                axis_b,

            "point_distance":
                distance(
                    point_a,
                    point_b
                ),

            "axis_dot":
                dot(
                    axis_a,
                    axis_b
                ),

            "axis":
                axis_a,
        })

    return results


# ============================================================
# DETERMINE JOINT ORIGIN
# ============================================================
#
# For now:
#
# Use the point from the first concentric mate.
#
# IMPORTANT:
# This is an initial strategy.
# We will later make joint-frame selection smarter.
# ============================================================

def determine_joint_origin(
    concentric_data
):

    if not concentric_data:
        return None

    return concentric_data[0]["point_a"]


# ============================================================
# CLASSIFY CONNECTION
# ============================================================

def classify_connection(
    component_a,
    component_b,
    mates,
    component_lookup
):

    mate_types = [
        mate["typeName"]
        for mate in mates
    ]

    mate_names = [
        mate["name"]
        for mate in mates
    ]

    has_concentric = (
        "MateConcentric"
        in mate_types
    )

    has_coincident = (
        "MateCoincident"
        in mate_types
    )

    has_lock = (
        "MateLock"
        in mate_types
    )

    # --------------------------------------------------------
    # BASE CHECK
    # --------------------------------------------------------

    component_a_data = \
        component_lookup.get(component_a)

    component_b_data = \
        component_lookup.get(component_b)

    a_fixed = (
        component_a_data is not None
        and component_a_data.get(
            "fixed",
            False
        )
    )

    b_fixed = (
        component_b_data is not None
        and component_b_data.get(
            "fixed",
            False
        )
    )

    # --------------------------------------------------------
    # ROOT CONNECTION
    # --------------------------------------------------------

    if a_fixed or b_fixed:

        return {
            "type": "fixed_root",
            "confidence": "HIGH",
            "reason":
                "One component is fixed "
                "in the SolidWorks assembly."
        }

    # --------------------------------------------------------
    # LOCK
    # --------------------------------------------------------

    if has_lock:

        return {
            "type": "fixed",
            "confidence": "HIGH",
            "reason":
                "SolidWorks Lock mate explicitly "
                "removes relative motion."
        }

    # --------------------------------------------------------
    # CONCENTRIC + COINCIDENT
    # --------------------------------------------------------

    if (
        has_concentric
        and has_coincident
    ):

        return {
            "type": "revolute_candidate",
            "confidence": "MEDIUM",
            "reason":
                "Concentric + Coincident found "
                "without a Lock mate."
        }

    # --------------------------------------------------------
    # CONCENTRIC ONLY
    # --------------------------------------------------------

    if has_concentric:

        return {
            "type": "cylindrical_candidate",
            "confidence": "LOW",
            "reason":
                "Concentric constraint found "
                "without Coincident or Lock."
        }

    # --------------------------------------------------------
    # COINCIDENT ONLY
    # --------------------------------------------------------

    if has_coincident:

        return {
            "type": "ambiguous",
            "confidence": "LOW",
            "reason":
                "Coincident constraint alone "
                "does not define a URDF joint."
        }

    # --------------------------------------------------------
    # UNKNOWN
    # --------------------------------------------------------

    return {
        "type": "ambiguous",
        "confidence": "LOW",
        "reason":
            "No recognized kinematic constraint "
            "combination."
    }


# ============================================================
# BUILD KINEMATIC MODEL
# ============================================================

def build_kinematic_model(data):

    component_lookup = \
        build_component_lookup(data)

    groups = group_mates(data)

    connections = []

    root_components = []

    # --------------------------------------------------------
    # FIND ROOT / FIXED COMPONENTS
    # --------------------------------------------------------

    for component in data["components"]:

        if component.get("fixed", False):

            root_components.append(
                component["name"]
            )

    # --------------------------------------------------------
    # BUILD CONNECTIONS
    # --------------------------------------------------------

    for (
        component_a,
        component_b
    ), mates in groups.items():

        classification = classify_connection(
            component_a,
            component_b,
            mates,
            component_lookup
        )

        concentric_data = \
            analyze_concentric(mates)

        joint_origin = \
            determine_joint_origin(
                concentric_data
            )

        connection = {

            "parent_candidate":
                component_a,

            "child_candidate":
                component_b,

            "mates": [
                {
                    "name": mate["name"],
                    "type": mate["typeName"]
                }
                for mate in mates
            ],

            "classification": classification,

            "concentric_geometry":
                concentric_data,

            "joint_origin_cad":
                joint_origin,

        }

        connections.append(
            connection
        )

    # --------------------------------------------------------
    # RETURN MODEL
    # --------------------------------------------------------

    return {

        "format": "sw2urdf_kinematic_model",

        "version": "3.0",

        "source_assembly":
            data["assembly"],

        "root_components":
            root_components,

        "components": [
            {
                "name":
                    component["name"],

                "path":
                    component["path"],

                "fixed":
                    component["fixed"],

                "assembly_origin":
                    component[
                        "assembly_origin"
                    ],

                "local_axes_in_assembly":
                    component[
                        "local_axes_in_assembly"
                    ],
            }
            for component
            in data["components"]
        ],

        "connections":
            connections,
    }


# ============================================================
# SAVE MODEL
# ============================================================

def save_model(model):

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


# ============================================================
# PRINT HUMAN-READABLE REPORT
# ============================================================

def print_report(model):

    print()
    print("=" * 70)
    print("SOLIDWORKS KINEMATIC ANALYZER V3")
    print("=" * 70)

    print()

    print(
        f"Root components: "
        f"{len(model['root_components'])}"
    )

    for root in model[
        "root_components"
    ]:

        print(
            f"  ROOT: {root}"
        )

    print()

    print(
        f"Connections: "
        f"{len(model['connections'])}"
    )

    for i, connection in enumerate(
        model["connections"],
        start=1
    ):

        print()
        print("-" * 70)

        print(
            f"CONNECTION {i}"
        )

        print(
            f"  A: "
            f"{connection['parent_candidate']}"
        )

        print(
            f"  B: "
            f"{connection['child_candidate']}"
        )

        classification = \
            connection["classification"]

        print()

        print(
            f"  CLASSIFICATION: "
            f"{classification['type']}"
        )

        print(
            f"  CONFIDENCE: "
            f"{classification['confidence']}"
        )

        print(
            f"  REASON: "
            f"{classification['reason']}"
        )

        print()

        print("  Mates:")

        for mate in connection[
            "mates"
        ]:

            print(
                f"    {mate['name']} "
                f"({mate['type']})"
            )

        if connection[
            "joint_origin_cad"
        ] is not None:

            print()

            print(
                "  Joint origin candidate: "
                f"{round_vec(connection['joint_origin_cad'])}"
            )

        concentric = connection[
            "concentric_geometry"
        ]

        if concentric:

            print()

            print(
                "  Joint axis candidate:"
            )

            axis = concentric[0][
                "axis"
            ]

            print(
                f"    {round_vec(axis)}"
            )

    print()
    print("=" * 70)
    print("KINEMATIC MODEL SAVED")
    print("=" * 70)

    print(
        f"File: {OUTPUT_FILE}"
    )


# ============================================================
# MAIN
# ============================================================

def main():

    data = load_cad_data()

    model = build_kinematic_model(
        data
    )

    save_model(model)

    print_report(model)


if __name__ == "__main__":
    main()