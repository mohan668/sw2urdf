import json
import math
from pathlib import Path


# ============================================================
# CONFIGURATION
# ============================================================

JSON_FILE = Path("robot_cad_data_v2.json")

# Numerical tolerance in meters.
# SolidWorks data contains tiny floating-point errors such as
# 1e-16, so we don't treat those as real geometric errors.
TOLERANCE = 1e-6


# ============================================================
# VECTOR FUNCTIONS
# ============================================================

def vector(a, b):
    """Vector from point a to point b."""
    return [
        b[i] - a[i]
        for i in range(3)
    ]


def dot(a, b):
    return sum(
        a[i] * b[i]
        for i in range(3)
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
        return [0.0, 0.0, 0.0]

    return [
        x / n
        for x in v
    ]


def distance(a, b):
    return norm(vector(a, b))


def round_vec(v, digits=6):
    return [
        round(x, digits)
        for x in v
    ]


# ============================================================
# GEOMETRIC ANALYSIS
# ============================================================

def analyze_axes(axis_a, axis_b):
    """
    Compare two axis directions.

    Returns:
        dot product
        angle
        direction relationship
    """

    a = normalize(axis_a)
    b = normalize(axis_b)

    d = max(-1.0, min(1.0, dot(a, b)))

    angle = math.degrees(
        math.acos(abs(d))
    )

    if d >= 0:
        relationship = "SAME"
    else:
        relationship = "OPPOSITE"

    return {
        "dot": d,
        "angle_deg": angle,
        "relationship": relationship,
    }


def analyze_concentric(entity_a, entity_b):
    """
    Analyze a SolidWorks concentric mate.

    Entity params format used by our extractor:

        [X, Y, Z, I, J, K, Radius1, Radius2]
    """

    pa = entity_a["params"]
    pb = entity_b["params"]

    point_a = pa[0:3]
    axis_a = pa[3:6]

    point_b = pb[0:3]
    axis_b = pb[3:6]

    axis_result = analyze_axes(
        axis_a,
        axis_b
    )

    point_delta = vector(
        point_a,
        point_b
    )

    point_distance = norm(
        point_delta
    )

    axis = normalize(axis_a)

    # Component of the point difference
    # along the common axis.
    axial_distance = dot(
        point_delta,
        axis
    )

    # Remaining component is perpendicular
    # to the axis.
    axial_vector = [
        axial_distance * x
        for x in axis
    ]

    perpendicular_vector = [
        point_delta[i] - axial_vector[i]
        for i in range(3)
    ]

    perpendicular_distance = norm(
        perpendicular_vector
    )

    common_axis = (
        axis_result["angle_deg"] < 0.01
        and perpendicular_distance < TOLERANCE
    )

    return {
        "point_a": point_a,
        "point_b": point_b,
        "axis_a": normalize(axis_a),
        "axis_b": normalize(axis_b),
        "point_distance": point_distance,
        "axial_distance": axial_distance,
        "perpendicular_distance": perpendicular_distance,
        "axis_dot": axis_result["dot"],
        "axis_angle_deg": axis_result["angle_deg"],
        "direction": axis_result["relationship"],
        "common_axis": common_axis,
    }


def analyze_coincident(entity_a, entity_b):
    """
    Analyze a SolidWorks coincident mate.

    Entity params format:

        [X, Y, Z, I, J, K, ...]
    """

    pa = entity_a["params"]
    pb = entity_b["params"]

    point_a = pa[0:3]
    point_b = pb[0:3]

    direction_a = pa[3:6]
    direction_b = pb[3:6]

    point_distance = distance(
        point_a,
        point_b
    )

    axis_result = analyze_axes(
        direction_a,
        direction_b
    )

    return {
        "point_a": point_a,
        "point_b": point_b,
        "point_distance": point_distance,
        "direction_dot": axis_result["dot"],
        "direction_angle_deg":
            axis_result["angle_deg"],
        "direction":
            axis_result["relationship"],
    }


# ============================================================
# LOAD DATA
# ============================================================

def load_data():

    if not JSON_FILE.exists():

        raise FileNotFoundError(
            f"Could not find {JSON_FILE}"
        )

    with open(
        JSON_FILE,
        "r",
        encoding="utf-8"
    ) as f:

        return json.load(f)


# ============================================================
# COMPONENT REPORT
# ============================================================

def print_components(data):

    components = data["components"]

    print()
    print("=" * 70)
    print("COMPONENTS")
    print("=" * 70)

    print(
        f"Total components: {len(components)}"
    )

    for component in components:

        print()
        print(
            f"Component: "
            f"{component['name']}"
        )

        print(
            f"  Fixed: "
            f"{component['fixed']}"
        )

        print(
            f"  Suppressed: "
            f"{component['suppressed']}"
        )

        origin = component[
            "assembly_origin"
        ]

        print(
            "  Assembly origin: "
            f"{round_vec(origin)}"
        )

        axes = component[
            "local_axes_in_assembly"
        ]

        print(
            "  Local X: "
            f"{round_vec(axes['x'])}"
        )

        print(
            "  Local Y: "
            f"{round_vec(axes['y'])}"
        )

        print(
            "  Local Z: "
            f"{round_vec(axes['z'])}"
        )


# ============================================================
# CONNECTION GRAPH
# ============================================================

def print_connection_graph(data):

    print()
    print("=" * 70)
    print("COMPONENT CONNECTION GRAPH")
    print("=" * 70)

    for mate in data["mates"]:

        entities = mate["entities"]

        if len(entities) < 2:
            continue

        a = entities[0]["component"]
        b = entities[1]["component"]

        print(
            f"{a} "
            f"--[{mate['name']} / "
            f"{mate['typeName']}]--> "
            f"{b}"
        )


# ============================================================
# MATE ANALYSIS
# ============================================================

def print_mate_analysis(data):

    print()
    print("=" * 70)
    print("MATE GEOMETRY ANALYSIS")
    print("=" * 70)

    for mate in data["mates"]:

        entities = mate["entities"]

        if len(entities) < 2:
            continue

        a = entities[0]
        b = entities[1]

        print()
        print("-" * 70)

        print(
            f"Mate: {mate['name']}"
        )

        print(
            f"Type: {mate['typeName']}"
        )

        print(
            f"A: {a['component']}"
        )

        print(
            f"B: {b['component']}"
        )

        # ----------------------------------------------------
        # CONCENTRIC
        # ----------------------------------------------------

        if mate["typeName"] == "MateConcentric":

            result = analyze_concentric(
                a,
                b
            )

            print(
                f"Axis A: "
                f"{round_vec(result['axis_a'])}"
            )

            print(
                f"Axis B: "
                f"{round_vec(result['axis_b'])}"
            )

            print(
                f"Point A: "
                f"{round_vec(result['point_a'])}"
            )

            print(
                f"Point B: "
                f"{round_vec(result['point_b'])}"
            )

            print(
                f"Point distance: "
                f"{result['point_distance'] * 1000:.6f} mm"
            )

            print(
                f"Axial distance: "
                f"{result['axial_distance'] * 1000:.6f} mm"
            )

            print(
                f"Perpendicular distance: "
                f"{result['perpendicular_distance'] * 1000:.6f} mm"
            )

            print(
                f"Axis dot product: "
                f"{result['axis_dot']:.9f}"
            )

            print(
                f"Axis angle: "
                f"{result['axis_angle_deg']:.9f} deg"
            )

            print(
                f"Directions: "
                f"{result['direction']}"
            )

            print(
                f"COMMON AXIS: "
                f"{'YES' if result['common_axis'] else 'NO'}"
            )

        # ----------------------------------------------------
        # COINCIDENT
        # ----------------------------------------------------

        elif mate["typeName"] == "MateCoincident":

            result = analyze_coincident(
                a,
                b
            )

            print(
                f"Point A: "
                f"{round_vec(result['point_a'])}"
            )

            print(
                f"Point B: "
                f"{round_vec(result['point_b'])}"
            )

            print(
                f"Point distance: "
                f"{result['point_distance'] * 1000:.6f} mm"
            )

            print(
                f"Direction dot: "
                f"{result['direction_dot']:.9f}"
            )

            print(
                f"Direction angle: "
                f"{result['direction_angle_deg']:.9f} deg"
            )

            print(
                f"Directions: "
                f"{result['direction']}"
            )

        else:

            print(
                "Geometry analysis for this mate "
                "type is not implemented yet."
            )


# ============================================================
# MAIN
# ============================================================

def main():

    print()
    print("=" * 70)
    print("SOLIDWORKS CAD KINEMATIC ANALYZER V1")
    print("=" * 70)

    data = load_data()

    assembly = data["assembly"]

    print()
    print(
        f"Assembly: "
        f"{assembly['name']}"
    )

    print(
        f"Source: "
        f"{assembly['path']}"
    )

    print_components(data)

    print_connection_graph(data)

    print_mate_analysis(data)

    print()
    print("=" * 70)
    print("ANALYSIS COMPLETE")
    print("=" * 70)


if __name__ == "__main__":
    main()