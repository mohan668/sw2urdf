import json
import math
from pathlib import Path
from collections import defaultdict


# ============================================================
# CONFIGURATION
# ============================================================

JSON_FILE = Path("robot_cad_data_v2.json")

TOLERANCE = 1e-6
ANGLE_TOLERANCE_DEG = 0.01


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
    return [
        a[i] - b[i]
        for i in range(3)
    ]


def distance(a, b):
    return norm(subtract(a, b))


def angle_between_axes(a, b):
    a = normalize(a)
    b = normalize(b)

    d = max(-1.0, min(1.0, abs(dot(a, b))))

    return math.degrees(math.acos(d))


def round_vec(v, digits=6):
    return [round(x, digits) for x in v]


# ============================================================
# LOAD CAD DATA
# ============================================================

def load_data():

    if not JSON_FILE.exists():

        raise FileNotFoundError(
            f"Could not find: {JSON_FILE}"
        )

    with open(
        JSON_FILE,
        "r",
        encoding="utf-8"
    ) as f:

        return json.load(f)


# ============================================================
# COMPONENT LOOKUP
# ============================================================

def build_component_lookup(data):

    lookup = {}

    for component in data["components"]:

        lookup[component["name"]] = component

    return lookup


# ============================================================
# NORMALIZED COMPONENT PAIR
# ============================================================

def component_pair(a, b):

    # Sorting makes:
    #
    # A -> B
    #
    # and
    #
    # B -> A
    #
    # belong to the same connection.

    return tuple(sorted([a, b]))


# ============================================================
# GROUP MATES BY COMPONENT PAIR
# ============================================================

def group_mates(data):

    groups = defaultdict(list)

    for mate in data["mates"]:

        entities = mate.get("entities", [])

        if len(entities) < 2:
            continue

        a = entities[0]["component"]
        b = entities[1]["component"]

        key = component_pair(a, b)

        groups[key].append(mate)

    return groups


# ============================================================
# EXTRACT CONCENTRIC GEOMETRY
# ============================================================

def get_concentric_geometry(mate):

    entities = mate["entities"]

    if len(entities) < 2:
        return None

    a = entities[0]
    b = entities[1]

    if "params" not in a or "params" not in b:
        return None

    pa = a["params"]
    pb = b["params"]

    if len(pa) < 6 or len(pb) < 6:
        return None

    point_a = pa[0:3]
    point_b = pb[0:3]

    axis_a = normalize(pa[3:6])
    axis_b = normalize(pb[3:6])

    # Use entity A as the candidate axis.
    axis = axis_a

    # Point difference.
    delta = subtract(point_b, point_a)

    axial_distance = dot(
        delta,
        axis
    )

    axial_component = [
        axial_distance * x
        for x in axis
    ]

    perpendicular_component = [
        delta[i] - axial_component[i]
        for i in range(3)
    ]

    perpendicular_distance = norm(
        perpendicular_component
    )

    axis_angle = angle_between_axes(
        axis_a,
        axis_b
    )

    return {
        "point_a": point_a,
        "point_b": point_b,
        "axis_a": axis_a,
        "axis_b": axis_b,
        "axis": axis,
        "axial_distance": axial_distance,
        "perpendicular_distance":
            perpendicular_distance,
        "axis_angle_deg": axis_angle,
    }


# ============================================================
# EXTRACT COINCIDENT GEOMETRY
# ============================================================

def get_coincident_geometry(mate):

    entities = mate["entities"]

    if len(entities) < 2:
        return None

    a = entities[0]
    b = entities[1]

    if "params" not in a or "params" not in b:
        return None

    pa = a["params"]
    pb = b["params"]

    if len(pa) < 3 or len(pb) < 3:
        return None

    point_a = pa[0:3]
    point_b = pb[0:3]

    result = {
        "point_a": point_a,
        "point_b": point_b,
        "point_distance":
            distance(point_a, point_b),
    }

    if len(pa) >= 6 and len(pb) >= 6:

        direction_a = normalize(pa[3:6])
        direction_b = normalize(pb[3:6])

        result["direction_a"] = direction_a
        result["direction_b"] = direction_b

        result["direction_dot"] = dot(
            direction_a,
            direction_b
        )

    return result


# ============================================================
# ANALYZE ONE CONNECTION
# ============================================================

def analyze_connection(
    component_a,
    component_b,
    mates
):

    concentric_mates = []
    coincident_mates = []
    other_mates = []

    for mate in mates:

        type_name = mate["typeName"]

        if type_name == "MateConcentric":

            concentric_mates.append(mate)

        elif type_name == "MateCoincident":

            coincident_mates.append(mate)

        else:

            other_mates.append(mate)

    concentric_geometry = []

    for mate in concentric_mates:

        geometry = get_concentric_geometry(
            mate
        )

        if geometry:

            concentric_geometry.append(
                geometry
            )

    coincident_geometry = []

    for mate in coincident_mates:

        geometry = get_coincident_geometry(
            mate
        )

        if geometry:

            coincident_geometry.append(
                geometry
            )

    # --------------------------------------------------------
    # DETERMINE CANDIDATE TYPE
    # --------------------------------------------------------

    has_concentric = (
        len(concentric_mates) > 0
    )

    has_coincident = (
        len(coincident_mates) > 0
    )

    candidate = "AMBIGUOUS"

    reason = []

    if has_concentric and has_coincident:

        candidate = "REVOLUTE_OR_FIXED"

        reason.append(
            "Concentric + Coincident constraints found."
        )

        reason.append(
            "Geometry alone cannot determine "
            "mechanical intent."
        )

    elif has_concentric:

        candidate = "REVOLUTE_OR_CYLINDRICAL"

        reason.append(
            "Concentric constraint found "
            "without Coincident."
        )

    elif has_coincident:

        candidate = "FIXED_CANDIDATE"

        reason.append(
            "Coincident constraint found "
            "without concentric geometry."
        )

    else:

        candidate = "UNKNOWN"

        reason.append(
            "No recognized constraint combination."
        )

    return {
        "component_a": component_a,
        "component_b": component_b,
        "mates": mates,
        "concentric_mates":
            concentric_mates,
        "coincident_mates":
            coincident_mates,
        "other_mates":
            other_mates,
        "concentric_geometry":
            concentric_geometry,
        "coincident_geometry":
            coincident_geometry,
        "candidate": candidate,
        "reason": reason,
    }


# ============================================================
# PRINT CONNECTION
# ============================================================

def print_connection(index, connection):

    print()
    print("=" * 70)

    print(
        f"CONNECTION {index}"
    )

    print("=" * 70)

    print(
        f"A: {connection['component_a']}"
    )

    print(
        f"B: {connection['component_b']}"
    )

    print()

    print("SolidWorks mates:")

    for mate in connection["mates"]:

        print(
            f"  - {mate['name']} "
            f"({mate['typeName']})"
        )

    # --------------------------------------------------------
    # CONCENTRIC DATA
    # --------------------------------------------------------

    if connection["concentric_geometry"]:

        print()
        print("Concentric geometry:")

        for i, geometry in enumerate(
            connection["concentric_geometry"],
            start=1
        ):

            print(
                f"  Concentric geometry {i}:"
            )

            print(
                f"    Axis: "
                f"{round_vec(geometry['axis'])}"
            )

            print(
                f"    Axis angle: "
                f"{geometry['axis_angle_deg']:.9f} deg"
            )

            print(
                f"    Point A: "
                f"{round_vec(geometry['point_a'])}"
            )

            print(
                f"    Point B: "
                f"{round_vec(geometry['point_b'])}"
            )

            print(
                f"    Axial distance: "
                f"{geometry['axial_distance'] * 1000:.6f} mm"
            )

            print(
                f"    Perpendicular distance: "
                f"{geometry['perpendicular_distance'] * 1000:.6f} mm"
            )

    # --------------------------------------------------------
    # COINCIDENT DATA
    # --------------------------------------------------------

    if connection["coincident_geometry"]:

        print()
        print("Coincident geometry:")

        for i, geometry in enumerate(
            connection["coincident_geometry"],
            start=1
        ):

            print(
                f"  Coincident geometry {i}:"
            )

            print(
                f"    Point A: "
                f"{round_vec(geometry['point_a'])}"
            )

            print(
                f"    Point B: "
                f"{round_vec(geometry['point_b'])}"
            )

            print(
                f"    Reference distance: "
                f"{geometry['point_distance'] * 1000:.6f} mm"
            )

            if "direction_dot" in geometry:

                print(
                    f"    Direction dot: "
                    f"{geometry['direction_dot']:.9f}"
                )

    # --------------------------------------------------------
    # CLASSIFICATION
    # --------------------------------------------------------

    print()
    print(
        "KINEMATIC CANDIDATE:"
    )

    print(
        f"  >>> {connection['candidate']}"
    )

    print()

    print("Reason:")

    for reason in connection["reason"]:

        print(
            f"  - {reason}"
        )


# ============================================================
# PRINT SUMMARY
# ============================================================

def print_summary(connections):

    print()
    print("=" * 70)
    print("KINEMATIC CONNECTION SUMMARY")
    print("=" * 70)

    for index, connection in enumerate(
        connections,
        start=1
    ):

        print(
            f"{index}. "
            f"{connection['component_a']}"
            f" <--> "
            f"{connection['component_b']}"
        )

        print(
            f"   Candidate: "
            f"{connection['candidate']}"
        )

        mate_names = [
            mate["name"]
            for mate in connection["mates"]
        ]

        print(
            f"   Mates: "
            f"{', '.join(mate_names)}"
        )


# ============================================================
# MAIN
# ============================================================

def main():

    print()
    print("=" * 70)
    print("SOLIDWORKS KINEMATIC ANALYZER V2")
    print("=" * 70)

    data = load_data()

    print()
    print(
        f"Assembly: "
        f"{data['assembly']['name']}"
    )

    groups = group_mates(data)

    print()
    print(
        f"Unique component connections: "
        f"{len(groups)}"
    )

    connections = []

    for (component_a, component_b), mates in groups.items():

        connection = analyze_connection(
            component_a,
            component_b,
            mates
        )

        connections.append(
            connection
        )

    # --------------------------------------------------------
    # PRINT ALL CONNECTIONS
    # --------------------------------------------------------

    for index, connection in enumerate(
        connections,
        start=1
    ):

        print_connection(
            index,
            connection
        )

    print_summary(
        connections
    )

    print()
    print("=" * 70)
    print("ANALYZER V2 COMPLETE")
    print("=" * 70)


if __name__ == "__main__":
    main()