import json
import math
from pathlib import Path


# ============================================================
# FILES
# ============================================================

INPUT_FILE = Path("kinematic_model_v5.json")
OUTPUT_FILE = Path("kinematic_model_v7.json")


# ============================================================
# ROBOT COORDINATE SYSTEM
# ============================================================
#
# These are the ROBOT axes expressed in SolidWorks CAD
# coordinates.
#
# Robot +X = Joint 2 axis
# Robot +Y = Joint 1 axis
# Robot +Z = +X cross +Y
#
# ============================================================

ROBOT_X_IN_CAD = [
    0.999288738788429,
    0.0,
    -0.0377096344536993
]

ROBOT_Y_IN_CAD = [
    0.0,
    -1.0,
    0.0
]


# ============================================================
# VECTOR MATH
# ============================================================

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

    return math.sqrt(
        dot(v, v)
    )


def normalize(v):

    n = norm(v)

    if n < 1e-12:

        raise ValueError(
            "Cannot normalize zero vector."
        )

    return [
        x / n
        for x in v
    ]


def round_vector(v, digits=12):

    return [
        round(x, digits)
        for x in v
    ]


# ============================================================
# BUILD ROBOT Z
# ============================================================

ROBOT_X_IN_CAD = normalize(
    ROBOT_X_IN_CAD
)

ROBOT_Y_IN_CAD = normalize(
    ROBOT_Y_IN_CAD
)

ROBOT_Z_IN_CAD = normalize(
    cross(
        ROBOT_X_IN_CAD,
        ROBOT_Y_IN_CAD
    )
)


# ============================================================
# CAD -> ROBOT TRANSFORMATION
# ============================================================
#
# The columns below are the robot axes expressed in CAD.
#
# Therefore:
#
# robot_vector = R * cad_vector
#
# where each row of R is the corresponding robot axis
# expressed in CAD coordinates.
#
# ============================================================

CAD_TO_ROBOT = [

    [
        ROBOT_X_IN_CAD[0],
        ROBOT_X_IN_CAD[1],
        ROBOT_X_IN_CAD[2]
    ],

    [
        ROBOT_Y_IN_CAD[0],
        ROBOT_Y_IN_CAD[1],
        ROBOT_Y_IN_CAD[2]
    ],

    [
        ROBOT_Z_IN_CAD[0],
        ROBOT_Z_IN_CAD[1],
        ROBOT_Z_IN_CAD[2]
    ]
]


# ============================================================
# MATRIX / VECTOR
# ============================================================

def mat_vec(M, v):

    return [

        M[0][0] * v[0]
        + M[0][1] * v[1]
        + M[0][2] * v[2],

        M[1][0] * v[0]
        + M[1][1] * v[1]
        + M[1][2] * v[2],

        M[2][0] * v[0]
        + M[2][1] * v[1]
        + M[2][2] * v[2]
    ]


# ============================================================
# MATRIX DETERMINANT
# ============================================================

def determinant(M):

    return (

        M[0][0] *
        (
            M[1][1] * M[2][2]
            -
            M[1][2] * M[2][1]
        )

        -

        M[0][1] *
        (
            M[1][0] * M[2][2]
            -
            M[1][2] * M[2][0]
        )

        +

        M[0][2] *
        (
            M[1][0] * M[2][1]
            -
            M[1][1] * M[2][0]
        )
    )


# ============================================================
# TRANSFORM POINT
# ============================================================

def cad_to_robot_point(point):

    return mat_vec(
        CAD_TO_ROBOT,
        point
    )


# ============================================================
# TRANSFORM DIRECTION
# ============================================================

def cad_to_robot_direction(direction):

    result = mat_vec(
        CAD_TO_ROBOT,
        direction
    )

    return normalize(
        result
    )


# ============================================================
# ANGLE BETWEEN VECTORS
# ============================================================

def angle_between(a, b):

    a = normalize(a)
    b = normalize(b)

    value = dot(a, b)

    value = max(
        -1.0,
        min(1.0, value)
    )

    return math.degrees(
        math.acos(value)
    )


# ============================================================
# JOINT VALIDATION
# ============================================================

def validate_joint_axes(model):

    print()
    print("=" * 70)
    print("JOINT AXIS VALIDATION")
    print("=" * 70)

    for joint in model["joints"]:

        frame = joint["frame"]

        axis = frame.get(
            "axis_assembly"
        )

        if axis is None:

            print()
            print(
                joint["name"],
                ": fixed / no axis"
            )

            continue

        axis_robot = \
            cad_to_robot_direction(
                axis
            )

        print()
        print(
            joint["name"],
            ":",
            joint["type"]
        )

        print(
            "CAD axis:",
            round_vector(axis)
        )

        print(
            "Robot axis:",
            round_vector(
                axis_robot
            )
        )

        # ----------------------------------------------------
        # Joint 1 expected to align with +Y
        # ----------------------------------------------------

        if joint["name"] == "joint_1":

            angle = angle_between(
                axis_robot,
                [0, 1, 0]
            )

            print(
                "Angle to robot +Y:",
                round(angle, 9),
                "deg"
            )

        # ----------------------------------------------------
        # Joint 2 expected to align with +X
        # ----------------------------------------------------

        if joint["name"] == "joint_2":

            angle = angle_between(
                axis_robot,
                [1, 0, 0]
            )

            print(
                "Angle to robot +X:",
                round(angle, 9),
                "deg"
            )


# ============================================================
# TRANSFORM JOINTS
# ============================================================

def transform_joints(model):

    joints = []

    for original_joint in model["joints"]:

        joint = json.loads(
            json.dumps(
                original_joint
            )
        )

        frame = joint["frame"]

        # ----------------------------------------------------
        # Assembly origin
        # ----------------------------------------------------

        if frame.get(
            "origin_assembly"
        ) is not None:

            frame[
                "origin_robot"
            ] = round_vector(
                cad_to_robot_point(
                    frame[
                        "origin_assembly"
                    ]
                )
            )

        else:

            frame[
                "origin_robot"
            ] = None

        # ----------------------------------------------------
        # Assembly axis
        # ----------------------------------------------------

        if frame.get(
            "axis_assembly"
        ) is not None:

            frame[
                "axis_robot"
            ] = round_vector(
                cad_to_robot_direction(
                    frame[
                        "axis_assembly"
                    ]
                )
            )

        else:

            frame[
                "axis_robot"
            ] = None

        # ----------------------------------------------------
        # Parent-local origin
        #
        # IMPORTANT:
        #
        # These values are already expressed in the parent
        # component frame.
        #
        # We DO NOT blindly multiply them by the global
        # CAD->Robot matrix.
        #
        # Their numerical values remain valid when the
        # complete parent frame is transformed consistently.
        #
        # ----------------------------------------------------

        if frame.get(
            "origin_parent"
        ) is not None:

            frame[
                "origin_parent_robot"
            ] = frame[
                "origin_parent"
            ]

        else:

            frame[
                "origin_parent_robot"
            ] = None

        # ----------------------------------------------------
        # Parent-local axis
        # ----------------------------------------------------

        if frame.get(
            "axis_parent"
        ) is not None:

            frame[
                "axis_parent_robot"
            ] = frame[
                "axis_parent"
            ]

        else:

            frame[
                "axis_parent_robot"
            ] = None

        joints.append(
            joint
        )

    return joints


# ============================================================
# VALIDATE ROBOT FRAME
# ============================================================

def validate_robot_frame():

    print()
    print("=" * 70)
    print("ROBOT COORDINATE FRAME")
    print("=" * 70)

    print()
    print(
        "Robot +X in CAD:"
    )

    print(
        " ",
        round_vector(
            ROBOT_X_IN_CAD
        )
    )

    print()
    print(
        "Robot +Y in CAD:"
    )

    print(
        " ",
        round_vector(
            ROBOT_Y_IN_CAD
        )
    )

    print()
    print(
        "Robot +Z in CAD:"
    )

    print(
        " ",
        round_vector(
            ROBOT_Z_IN_CAD
        )
    )

    print()

    print(
        "X · Y =",
        dot(
            ROBOT_X_IN_CAD,
            ROBOT_Y_IN_CAD
        )
    )

    print(
        "Y · Z =",
        dot(
            ROBOT_Y_IN_CAD,
            ROBOT_Z_IN_CAD
        )
    )

    print(
        "Z · X =",
        dot(
            ROBOT_Z_IN_CAD,
            ROBOT_X_IN_CAD
        )
    )

    print()

    det = determinant(
        CAD_TO_ROBOT
    )

    print(
        "Transformation determinant:",
        det
    )

    if abs(det - 1.0) < 1e-9:

        print(
            "STATUS: VALID RIGHT-HANDED FRAME"
        )

    else:

        print(
            "ERROR: INVALID FRAME"
        )


# ============================================================
# BUILD V7 MODEL
# ============================================================

def build_v7(model):

    new_model = json.loads(
        json.dumps(model)
    )

    new_model[
        "version"
    ] = "7.0"

    new_model[
        "coordinate_system"
    ] = {

        "source":
            "SolidWorks assembly",

        "conversion_applied":
            True,

        "target":
            "Robot / ROS coordinate system",

        "robot_axes_in_cad": {

            "x":
                round_vector(
                    ROBOT_X_IN_CAD
                ),

            "y":
                round_vector(
                    ROBOT_Y_IN_CAD
                ),

            "z":
                round_vector(
                    ROBOT_Z_IN_CAD
                )
        },

        "cad_to_robot_matrix":
            [
                round_vector(row)
                for row
                in CAD_TO_ROBOT
            ],

        "definition": {

            "x":
                "Robot X is Joint 2 axis",

            "y":
                "Robot Y is Joint 1 axis",

            "z":
                "Robot Z = X cross Y"
        }
    }

    new_model[
        "joints"
    ] = transform_joints(
        model
    )

    return new_model


# ============================================================
# REPORT
# ============================================================

def print_report(model):

    print()
    print("=" * 70)
    print("SOLIDWORKS KINEMATIC ANALYZER V7")
    print("=" * 70)

    print()

    for joint in model["joints"]:

        frame = joint["frame"]

        print(
            joint["name"],
            ":",
            joint["type"]
        )

        print(
            "  Parent:",
            joint["parent"]
        )

        print(
            "  Child:",
            joint["child"]
        )

        print()

        print(
            "  Origin CAD:"
        )

        print(
            "   ",
            frame.get(
                "origin_assembly"
            )
        )

        print(
            "  Origin ROBOT:"
        )

        print(
            "   ",
            frame.get(
                "origin_robot"
            )
        )

        if frame.get(
            "axis_assembly"
        ) is not None:

            print()

            print(
                "  Axis CAD:"
            )

            print(
                "   ",
                frame[
                    "axis_assembly"
                ]
            )

            print(
                "  Axis ROBOT:"
            )

            print(
                "   ",
                frame[
                    "axis_robot"
                ]
            )

        print()
        print("-" * 70)


# ============================================================
# MAIN
# ============================================================

def main():

    if not INPUT_FILE.exists():

        raise FileNotFoundError(
            f"Could not find {INPUT_FILE}"
        )

    with open(
        INPUT_FILE,
        "r",
        encoding="utf-8"
    ) as f:

        model = json.load(f)

    validate_robot_frame()

    validate_joint_axes(
        model
    )

    v7 = build_v7(
        model
    )

    with open(
        OUTPUT_FILE,
        "w",
        encoding="utf-8"
    ) as f:

        json.dump(
            v7,
            f,
            indent=2
        )

    print_report(
        v7
    )

    print()
    print("=" * 70)
    print(
        "V7 COMPLETE"
    )
    print("=" * 70)

    print(
        "Saved:",
        OUTPUT_FILE
    )


if __name__ == "__main__":
    main()