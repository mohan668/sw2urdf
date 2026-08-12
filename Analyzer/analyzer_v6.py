import json
import math
from pathlib import Path


# ============================================================
# FILES
# ============================================================

INPUT_FILE = Path("kinematic_model_v5.json")
OUTPUT_FILE = Path("kinematic_model_v6.json")


# ============================================================
# COORDINATE CONVERSION
# ============================================================
#
# IMPORTANT:
#
# We are intentionally NOT assuming that SolidWorks X/Y/Z
# already correspond to ROS X/Y/Z.
#
# This matrix is the SINGLE global CAD -> URDF transformation.
#
# At the moment it is identity:
#
# CAD X -> URDF X
# CAD Y -> URDF Y
# CAD Z -> URDF Z
#
# We will change this ONLY after validating your intended
# physical robot coordinate system.
#
# ============================================================

CAD_TO_URDF = [
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0]
]


# ============================================================
# VECTOR FUNCTIONS
# ============================================================

def mat_vec(matrix, vector):

    return [
        matrix[0][0] * vector[0]
        + matrix[0][1] * vector[1]
        + matrix[0][2] * vector[2],

        matrix[1][0] * vector[0]
        + matrix[1][1] * vector[1]
        + matrix[1][2] * vector[2],

        matrix[2][0] * vector[0]
        + matrix[2][1] * vector[1]
        + matrix[2][2] * vector[2]
    ]


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

    length = norm(v)

    if length < 1e-12:
        return [0.0, 0.0, 0.0]

    return [
        x / length
        for x in v
    ]


def round_vector(v, digits=12):

    return [
        round(x, digits)
        for x in v
    ]


# ============================================================
# ROTATION MATRIX FUNCTIONS
# ============================================================

def matrix_multiply(A, B):

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


def transpose(A):

    return [
        [A[0][0], A[1][0], A[2][0]],
        [A[0][1], A[1][1], A[2][1]],
        [A[0][2], A[1][2], A[2][2]]
    ]


def determinant(A):

    return (

        A[0][0] *
        (
            A[1][1] * A[2][2]
            -
            A[1][2] * A[2][1]
        )

        -

        A[0][1] *
        (
            A[1][0] * A[2][2]
            -
            A[1][2] * A[2][0]
        )

        +

        A[0][2] *
        (
            A[1][0] * A[2][1]
            -
            A[1][1] * A[2][0]
        )
    )


# ============================================================
# VECTOR TRANSFORMATION
# ============================================================

def transform_vector(vector):

    return normalize(
        mat_vec(
            CAD_TO_URDF,
            vector
        )
    )


def transform_point(point):

    return mat_vec(
        CAD_TO_URDF,
        point
    )


# ============================================================
# FRAME TRANSFORMATION
# ============================================================

def transform_frame_axes(frame):

    return {

        "x":
            round_vector(
                transform_vector(
                    frame["x"]
                )
            ),

        "y":
            round_vector(
                transform_vector(
                    frame["y"]
                )
            ),

        "z":
            round_vector(
                transform_vector(
                    frame["z"]
                )
            )
    }


# ============================================================
# VALIDATE COORDINATE MATRIX
# ============================================================

def validate_coordinate_matrix():

    determinant_value = determinant(
        CAD_TO_URDF
    )

    print()
    print("=" * 70)
    print("COORDINATE SYSTEM VALIDATION")
    print("=" * 70)

    print()
    print("CAD -> URDF matrix:")

    for row in CAD_TO_URDF:
        print(
            "   ",
            row
        )

    print()
    print(
        "Determinant:",
        determinant_value
    )

    if abs(
        determinant_value - 1.0
    ) < 1e-9:

        print(
            "STATUS: VALID RIGHT-HANDED ROTATION"
        )

    elif abs(
        determinant_value + 1.0
    ) < 1e-9:

        print(
            "WARNING: REFLECTION / MIRRORED FRAME"
        )

    else:

        print(
            "WARNING: MATRIX IS NOT A PURE ROTATION"
        )


# ============================================================
# TRANSFORM JOINT
# ============================================================

def transform_joint(joint):

    new_joint = json.loads(
        json.dumps(joint)
    )

    frame = new_joint["frame"]

    # --------------------------------------------------------
    # Assembly origin
    # --------------------------------------------------------

    if frame[
        "origin_assembly"
    ] is not None:

        frame[
            "origin_urdf"
        ] = round_vector(
            transform_point(
                frame[
                    "origin_assembly"
                ]
            )
        )

    else:

        frame[
            "origin_urdf"
        ] = None

    # --------------------------------------------------------
    # Assembly axis
    # --------------------------------------------------------

    if frame[
        "axis_assembly"
    ] is not None:

        frame[
            "axis_urdf"
        ] = round_vector(
            transform_vector(
                frame[
                    "axis_assembly"
                ]
            )
        )

    else:

        frame[
            "axis_urdf"
        ] = None

    # --------------------------------------------------------
    # Parent origin
    #
    # IMPORTANT:
    #
    # For now we retain the parent-local values untouched.
    #
    # Why?
    #
    # Because parent-local coordinates depend on the parent
    # frame itself. We will perform the complete frame
    # transformation once the global coordinate convention
    # is confirmed.
    #
    # --------------------------------------------------------

    if frame[
        "origin_parent"
    ] is not None:

        frame[
            "origin_parent_cad"
        ] = frame[
            "origin_parent"
        ]

    if frame[
        "axis_parent"
    ] is not None:

        frame[
            "axis_parent_cad"
        ] = frame[
            "axis_parent"
        ]

    # --------------------------------------------------------
    # Child frame
    # --------------------------------------------------------

    if (
        "child_frame_in_parent"
        in frame
    ):

        child_frame = \
            frame[
                "child_frame_in_parent"
            ]

        frame[
            "child_frame_in_parent_urdf"
        ] = transform_frame_axes(
            child_frame
        )

    return new_joint


# ============================================================
# TRANSFORM MODEL
# ============================================================

def transform_model(model):

    new_model = json.loads(
        json.dumps(model)
    )

    new_model[
        "version"
    ] = "6.0"

    new_model[
        "coordinate_system"
    ] = {

        "source":
            "SolidWorks assembly",

        "conversion_applied":
            True,

        "transformation":
            CAD_TO_URDF,

        "description":
            "Global CAD-to-URDF coordinate "
            "transformation.",

        "note":
            "Parent-local frame conversion is "
            "deferred until the physical robot "
            "coordinate convention is validated."
    }

    transformed_joints = []

    for joint in model[
        "joints"
    ]:

        transformed_joints.append(
            transform_joint(
                joint
            )
        )

    new_model[
        "joints"
    ] = transformed_joints

    return new_model


# ============================================================
# REPORT
# ============================================================

def print_report(model):

    print()
    print("=" * 70)
    print("SOLIDWORKS KINEMATIC ANALYZER V6")
    print("=" * 70)

    print()
    print(
        "Global CAD -> URDF conversion:"
    )

    print()

    for row in CAD_TO_URDF:

        print(
            "   ",
            row
        )

    print()

    print(
        "Coordinate conversion is currently "
        "configured as identity."
    )

    print(
        "This is intentional."
    )

    print()
    print("=" * 70)
    print("JOINT COORDINATES")
    print("=" * 70)

    for joint in model[
        "joints"
    ]:

        frame = joint[
            "frame"
        ]

        print()

        print(
            joint["name"],
            ":",
            joint["type"]
        )

        print(
            "Parent:",
            joint["parent"]
        )

        print(
            "Child:",
            joint["child"]
        )

        print()

        print(
            "CAD origin:",
            frame[
                "origin_assembly"
            ]
        )

        print(
            "URDF origin:",
            frame[
                "origin_urdf"
            ]
        )

        print(
            "CAD axis:",
            frame[
                "axis_assembly"
            ]
        )

        print(
            "URDF axis:",
            frame[
                "axis_urdf"
            ]
        )

        if (
            "child_frame_in_parent_urdf"
            in frame
        ):

            print()

            print(
                "Child frame in parent "
                "(URDF coordinates):"
            )

            child_frame = frame[
                "child_frame_in_parent_urdf"
            ]

            print(
                "  X:",
                child_frame["x"]
            )

            print(
                "  Y:",
                child_frame["y"]
            )

            print(
                "  Z:",
                child_frame["z"]
            )

    print()
    print("=" * 70)
    print("V6 COMPLETE")
    print("=" * 70)


# ============================================================
# MAIN
# ============================================================

def main():

    if not INPUT_FILE.exists():

        raise FileNotFoundError(
            f"Missing input file: "
            f"{INPUT_FILE}"
        )

    with open(
        INPUT_FILE,
        "r",
        encoding="utf-8"
    ) as f:

        model = json.load(f)

    validate_coordinate_matrix()

    transformed = transform_model(
        model
    )

    with open(
        OUTPUT_FILE,
        "w",
        encoding="utf-8"
    ) as f:

        json.dump(
            transformed,
            f,
            indent=2
        )

    print_report(
        transformed
    )

    print()
    print(
        f"Saved: {OUTPUT_FILE}"
    )


if __name__ == "__main__":
    main()