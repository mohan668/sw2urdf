import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


# ================================================================
# SOLIDWORKS → URDF
# V11 - FIRST ACTUAL URDF GENERATOR
# ================================================================

INPUT_FILE = "kinematic_model_v10.json"
OUTPUT_FILE = "robot_v11.urdf"

ROBOT_NAME = "black_pepper_robot"

# Temporary joint limits.
# These are NOT physical limits yet.
# They only make the revolute joints complete URDF joints.
TEMP_LOWER = -3.141592653589793
TEMP_UPPER = 3.141592653589793
TEMP_EFFORT = 1.0
TEMP_VELOCITY = 1.0


# ================================================================
# HELPERS
# ================================================================

def fmt(value):

    if abs(value) < 1e-12:
        value = 0.0

    return f"{value:.12g}"


def fmt_vector(values):

    return " ".join(
        fmt(float(v))
        for v in values
    )


def load_v10():

    path = Path(INPUT_FILE)

    if not path.exists():

        raise FileNotFoundError(
            f"Input file not found:\n{INPUT_FILE}"
        )

    with open(
        path,
        "r",
        encoding="utf-8"
    ) as f:

        model = json.load(f)

    version = str(
        model.get("version", "")
    )

    if version != "10.0":

        raise ValueError(
            f"Expected V10 model, "
            f"but received version {version}"
        )

    return model


# ================================================================
# LINK NAME SANITIZATION
# ================================================================

def urdf_link_name(cad_name):

    # Keep the original CAD name where possible.
    #
    # URDF allows names containing many normal characters,
    # but spaces and some punctuation are inconvenient.
    #
    # Our current component names contain underscores, numbers
    # and hyphens, so we only replace spaces.

    return cad_name.replace(
        " ",
        "_"
    )


# ================================================================
# CREATE ROOT XML
# ================================================================

def create_robot():

    return ET.Element(
        "robot",
        {
            "name": ROBOT_NAME
        }
    )


# ================================================================
# ADD LINKS
# ================================================================

def add_links(
    robot,
    model
):

    links = model.get(
        "links",
        []
    )

    if not links:

        raise ValueError(
            "V10 contains no links."
        )

    created = set()

    for link in links:

        name = link.get(
            "name"
        )

        if not name:

            raise ValueError(
                "A link has no name."
            )

        urdf_name = urdf_link_name(
            name
        )

        if urdf_name in created:

            raise ValueError(
                f"Duplicate URDF link name: "
                f"{urdf_name}"
            )

        ET.SubElement(
            robot,
            "link",
            {
                "name": urdf_name
            }
        )

        created.add(
            urdf_name
        )


# ================================================================
# ADD REVOLUTE JOINT
# ================================================================

def add_revolute_joint(
    robot,
    joint
):

    name = joint["name"]

    parent = urdf_link_name(
        joint["parent"]
    )

    child = urdf_link_name(
        joint["child"]
    )

    frame = joint.get(
        "frame",
        {}
    )

    origin = frame.get(
        "origin"
    )

    axis = frame.get(
        "axis"
    )

    if origin is None:

        raise ValueError(
            f"{name}: missing joint origin."
        )

    if axis is None:

        raise ValueError(
            f"{name}: missing joint axis."
        )

    # ------------------------------------------------------------
    # Normalize axis
    # ------------------------------------------------------------

    length = math.sqrt(
        axis[0] ** 2 +
        axis[1] ** 2 +
        axis[2] ** 2
    )

    if length < 1e-12:

        raise ValueError(
            f"{name}: zero-length joint axis."
        )

    axis = [
        axis[0] / length,
        axis[1] / length,
        axis[2] / length
    ]

    # ------------------------------------------------------------
    # Joint
    # ------------------------------------------------------------

    joint_xml = ET.SubElement(
        robot,
        "joint",
        {
            "name": name,
            "type": "revolute"
        }
    )

    # ------------------------------------------------------------
    # Parent
    # ------------------------------------------------------------

    ET.SubElement(
        joint_xml,
        "parent",
        {
            "link": parent
        }
    )

    # ------------------------------------------------------------
    # Child
    # ------------------------------------------------------------

    ET.SubElement(
        joint_xml,
        "child",
        {
            "link": child
        }
    )

    # ------------------------------------------------------------
    # Origin
    # ------------------------------------------------------------

    ET.SubElement(
        joint_xml,
        "origin",
        {
            "xyz": fmt_vector(origin),
            "rpy": "0 0 0"
        }
    )

    # ------------------------------------------------------------
    # Axis
    # ------------------------------------------------------------

    ET.SubElement(
        joint_xml,
        "axis",
        {
            "xyz": fmt_vector(axis)
        }
    )

    # ------------------------------------------------------------
    # TEMPORARY LIMITS
    #
    # These are placeholders.
    # We will replace them with actual robot limits later.
    # ------------------------------------------------------------

    ET.SubElement(
        joint_xml,
        "limit",
        {
            "lower": fmt(TEMP_LOWER),
            "upper": fmt(TEMP_UPPER),
            "effort": fmt(TEMP_EFFORT),
            "velocity": fmt(TEMP_VELOCITY)
        }
    )


# ================================================================
# ADD FIXED JOINT
# ================================================================

def add_fixed_joint(
    robot,
    joint
):

    name = joint["name"]

    parent = urdf_link_name(
        joint["parent"]
    )

    child = urdf_link_name(
        joint["child"]
    )

    frame = joint.get(
        "frame",
        {}
    )

    origin = frame.get(
        "origin"
    )

    rpy = frame.get(
        "rpy"
    )

    if origin is None:

        raise ValueError(
            f"{name}: missing fixed-joint origin."
        )

    if rpy is None:

        rpy = [
            0.0,
            0.0,
            0.0
        ]

    # ------------------------------------------------------------
    # Joint
    # ------------------------------------------------------------

    joint_xml = ET.SubElement(
        robot,
        "joint",
        {
            "name": name,
            "type": "fixed"
        }
    )

    # ------------------------------------------------------------
    # Parent
    # ------------------------------------------------------------

    ET.SubElement(
        joint_xml,
        "parent",
        {
            "link": parent
        }
    )

    # ------------------------------------------------------------
    # Child
    # ------------------------------------------------------------

    ET.SubElement(
        joint_xml,
        "child",
        {
            "link": child
        }
    )

    # ------------------------------------------------------------
    # Origin
    # ------------------------------------------------------------

    ET.SubElement(
        joint_xml,
        "origin",
        {
            "xyz": fmt_vector(origin),
            "rpy": fmt_vector(rpy)
        }
    )


# ================================================================
# ADD JOINTS
# ================================================================

def add_joints(
    robot,
    model
):

    joints = model.get(
        "joints",
        []
    )

    if not joints:

        raise ValueError(
            "V10 contains no joints."
        )

    for joint in joints:

        joint_type = joint.get(
            "type"
        )

        if joint_type == "revolute":

            add_revolute_joint(
                robot,
                joint
            )

        elif joint_type == "fixed":

            add_fixed_joint(
                robot,
                joint
            )

        else:

            raise ValueError(
                f"Unsupported joint type "
                f"'{joint_type}' in "
                f"{joint['name']}"
            )


# ================================================================
# XML INDENTATION
# ================================================================

def indent_xml(
    element,
    level=0
):

    indentation = "\n" + (
        "  " * level
    )

    if len(element):

        if not element.text or not element.text.strip():

            element.text = indentation + "  "

        for child in element:

            indent_xml(
                child,
                level + 1
            )

        if (
            not element[-1].tail
            or not element[-1].tail.strip()
        ):

            element[-1].tail = indentation

    if level and (
        not element.tail
        or not element.tail.strip()
    ):

        element.tail = indentation


# ================================================================
# VALIDATE URDF STRUCTURE
# ================================================================

def validate_urdf(
    robot
):

    errors = []

    links = robot.findall(
        "link"
    )

    joints = robot.findall(
        "joint"
    )

    link_names = [
        link.get("name")
        for link in links
    ]

    link_set = set(
        link_names
    )

    # ------------------------------------------------------------
    # Duplicate links
    # ------------------------------------------------------------

    if len(link_names) != len(
        link_set
    ):

        errors.append(
            "Duplicate link names."
        )

    # ------------------------------------------------------------
    # Joint validation
    # ------------------------------------------------------------

    child_links = []

    for joint in joints:

        name = joint.get(
            "name"
        )

        parent = joint.find(
            "parent"
        )

        child = joint.find(
            "child"
        )

        if parent is None:

            errors.append(
                f"{name}: missing parent."
            )

        if child is None:

            errors.append(
                f"{name}: missing child."
            )

        if parent is not None:

            parent_name = parent.get(
                "link"
            )

            if parent_name not in link_set:

                errors.append(
                    f"{name}: parent "
                    f"{parent_name} does not exist."
                )

        if child is not None:

            child_name = child.get(
                "link"
            )

            if child_name not in link_set:

                errors.append(
                    f"{name}: child "
                    f"{child_name} does not exist."
                )

            child_links.append(
                child_name
            )

        # --------------------------------------------------------
        # Revolute
        # --------------------------------------------------------

        if joint.get(
            "type"
        ) == "revolute":

            axis = joint.find(
                "axis"
            )

            limit = joint.find(
                "limit"
            )

            origin = joint.find(
                "origin"
            )

            if axis is None:

                errors.append(
                    f"{name}: revolute joint "
                    f"missing axis."
                )

            if limit is None:

                errors.append(
                    f"{name}: revolute joint "
                    f"missing limit."
                )

            if origin is None:

                errors.append(
                    f"{name}: missing origin."
                )

    # ------------------------------------------------------------
    # Every child should have only one parent
    # ------------------------------------------------------------

    if len(child_links) != len(
        set(child_links)
    ):

        errors.append(
            "A link appears as the child "
            "of multiple joints."
        )

    # ------------------------------------------------------------
    # Tree check
    # ------------------------------------------------------------

    if len(links) != len(joints) + 1:

        errors.append(
            "Link/joint count does not "
            "form a simple tree: "
            f"{len(links)} links, "
            f"{len(joints)} joints."
        )

    if errors:

        raise ValueError(
            "URDF validation failed:\n"
            +
            "\n".join(
                f"  - {x}"
                for x in errors
            )
        )


# ================================================================
# SAVE URDF
# ================================================================

def save_urdf(
    robot
):

    indent_xml(
        robot
    )

    tree = ET.ElementTree(
        robot
    )

    ET.register_namespace(
        "",
        ""
    )

    tree.write(
        OUTPUT_FILE,
        encoding="utf-8",
        xml_declaration=True
    )


# ================================================================
# PRINT URDF SUMMARY
# ================================================================

def print_summary(
    robot
):

    links = robot.findall(
        "link"
    )

    joints = robot.findall(
        "joint"
    )

    print()
    print(
        "=" * 72
    )

    print(
        "V11 URDF GENERATED SUCCESSFULLY"
    )

    print(
        "=" * 72
    )

    print()

    print(
        "Robot:",
        robot.get("name")
    )

    print(
        "Links:",
        len(links)
    )

    print(
        "Joints:",
        len(joints)
    )

    print()

    print(
        "-" * 72
    )

    print(
        "LINKS"
    )

    print(
        "-" * 72
    )

    for link in links:

        print(
            f"  {link.get('name')}"
        )

    print()

    print(
        "-" * 72
    )

    print(
        "JOINTS"
    )

    print(
        "-" * 72
    )

    for joint in joints:

        name = joint.get(
            "name"
        )

        joint_type = joint.get(
            "type"
        )

        parent = joint.find(
            "parent"
        ).get(
            "link"
        )

        child = joint.find(
            "child"
        ).get(
            "link"
        )

        print()

        print(
            f"  {name}"
        )

        print(
            f"    type   : {joint_type}"
        )

        print(
            f"    parent : {parent}"
        )

        print(
            f"    child  : {child}"
        )

        origin = joint.find(
            "origin"
        )

        if origin is not None:

            print(
                f"    xyz    : "
                f"{origin.get('xyz')}"
            )

            print(
                f"    rpy    : "
                f"{origin.get('rpy')}"
            )

        axis = joint.find(
            "axis"
        )

        if axis is not None:

            print(
                f"    axis   : "
                f"{axis.get('xyz')}"
            )

    print()

    print(
        "=" * 72
    )

    print(
        f"Output: {OUTPUT_FILE}"
    )

    print(
        "=" * 72
    )


# ================================================================
# MAIN
# ================================================================

def main():

    print(
        "=" * 72
    )

    print(
        "SOLIDWORKS CAD → URDF GENERATOR V11"
    )

    print(
        "=" * 72
    )

    print()

    print(
        "Loading:",
        INPUT_FILE
    )

    model = load_v10()

    print(
        "Input version:",
        model.get(
            "version"
        )
    )

    print()

    # ------------------------------------------------------------
    # Create robot
    # ------------------------------------------------------------

    robot = create_robot()

    # ------------------------------------------------------------
    # Links
    # ------------------------------------------------------------

    add_links(
        robot,
        model
    )

    # ------------------------------------------------------------
    # Joints
    # ------------------------------------------------------------

    add_joints(
        robot,
        model
    )

    # ------------------------------------------------------------
    # Validate
    # ------------------------------------------------------------

    print(
        "Validating URDF..."
    )

    validate_urdf(
        robot
    )

    print(
        "Validation: PASS"
    )

    # ------------------------------------------------------------
    # Save
    # ------------------------------------------------------------

    save_urdf(
        robot
    )

    # ------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------

    print_summary(
        robot
    )


# ================================================================
# ENTRY POINT
# ================================================================

if __name__ == "__main__":

    main()