import os
import re
import xml.etree.ElementTree as ET
from pathlib import Path


# ============================================================
# CONFIGURATION
# ============================================================

INPUT_URDF = Path("robot_v11.urdf")

OUTPUT_URDF = Path("black_pepper_robot_v12.urdf")

MESH_DIR = Path("../meshes")

ROBOT_NAME = "black_pepper_robot"


# ============================================================
# COMPONENT → MESH MAPPING
# ============================================================

MESH_MAP = {
    "01.Base-1": "01.Base.STL",
    "0.First_Mount-1": "0.First_Mount.STL",
    "05.UR_5_02_Nema17_Joint-1": "05.UR_5_02_Nema17_Joint.STL",
    "Cap_End_Effector_Joint-5": "Cap_End_Effector_Joint.STL",
}


# ============================================================
# HELPERS
# ============================================================

def normalize_path(path):
    return path.replace("\\", "/")


def mesh_exists(filename):
    path = MESH_DIR / filename
    return path.exists()


def mesh_uri(filename):
    return "meshes/" + filename


def pretty_xml(element):
    """
    Pretty-print XML using indentation.
    """
    indent(element)


def indent(elem, level=0):
    """
    Recursive XML indentation.
    """

    spacing = "\n" + level * "  "

    if len(elem):

        if not elem.text or not elem.text.strip():
            elem.text = spacing + "  "

        for child in elem:
            indent(child, level + 1)

        if not child.tail or not child.tail.strip():
            child.tail = spacing

    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = spacing


# ============================================================
# MESH CREATION
# ============================================================

def create_geometry(parent, filename):
    """
    Create:

        <geometry>
            <mesh filename="meshes/example.stl"/>
        </geometry>
    """

    geometry = ET.SubElement(parent, "geometry")

    mesh = ET.SubElement(
        geometry,
        "mesh"
    )

    mesh.set(
        "filename",
        mesh_uri(filename)
    )

    return geometry


def create_visual(link, filename):
    """
    Add visual geometry.
    """

    visual = ET.SubElement(
        link,
        "visual"
    )

    create_geometry(
        visual,
        filename
    )

    return visual


def create_collision(link, filename):
    """
    Add collision geometry.
    """

    collision = ET.SubElement(
        link,
        "collision"
    )

    create_geometry(
        collision,
        filename
    )

    return collision


# ============================================================
# MESH VALIDATION
# ============================================================

def validate_meshes(robot):

    print()
    print("=" * 72)
    print("MESH VALIDATION")
    print("=" * 72)

    missing = []
    found = []

    for link in robot.findall("link"):

        name = link.get("name")

        filename = MESH_MAP.get(name)

        if filename is None:

            print(
                f"[WARNING] No mesh mapping for: {name}"
            )

            missing.append(name)

            continue

        path = MESH_DIR / filename

        if path.exists():

            print(
                f"[FOUND]   {name}"
            )

            print(
                f"          {path}"
            )

            found.append(name)

        else:

            print(
                f"[MISSING] {name}"
            )

            print(
                f"          Expected: {path}"
            )

            missing.append(name)

    print()
    print(f"Meshes found   : {len(found)}")
    print(f"Meshes missing : {len(missing)}")

    return missing


# ============================================================
# ADD MESHES
# ============================================================

def add_meshes(robot):

    print()
    print("=" * 72)
    print("ADDING MESH GEOMETRY")
    print("=" * 72)

    added = 0

    for link in robot.findall("link"):

        name = link.get("name")

        filename = MESH_MAP.get(name)

        if filename is None:

            print(
                f"[SKIP] No mesh mapping: {name}"
            )

            continue

        path = MESH_DIR / filename

        if not path.exists():

            print(
                f"[SKIP] Mesh not found: {filename}"
            )

            continue

        # ----------------------------------------------------
        # Remove existing visual/collision geometry
        # ----------------------------------------------------

        for old_visual in link.findall("visual"):
            link.remove(old_visual)

        for old_collision in link.findall("collision"):
            link.remove(old_collision)

        # ----------------------------------------------------
        # Add visual
        # ----------------------------------------------------

        create_visual(
            link,
            filename
        )

        # ----------------------------------------------------
        # Add collision
        # ----------------------------------------------------

        create_collision(
            link,
            filename
        )

        print(
            f"[ADDED] {name}"
        )

        print(
            f"        mesh: {mesh_uri(filename)}"
        )

        added += 1

    return added


# ============================================================
# URDF VALIDATION
# ============================================================

def validate_urdf(robot):

    print()
    print("=" * 72)
    print("URDF VALIDATION")
    print("=" * 72)

    links = robot.findall("link")
    joints = robot.findall("joint")

    link_names = {
        link.get("name")
        for link in links
    }

    errors = []

    print(
        f"Links  : {len(links)}"
    )

    print(
        f"Joints : {len(joints)}"
    )

    # --------------------------------------------------------
    # Validate joints
    # --------------------------------------------------------

    for joint in joints:

        name = joint.get("name")

        parent = joint.find("parent")
        child = joint.find("child")

        if parent is None:

            errors.append(
                f"{name}: missing parent"
            )

        else:

            parent_name = parent.get("link")

            if parent_name not in link_names:

                errors.append(
                    f"{name}: parent link does not exist: "
                    f"{parent_name}"
                )

        if child is None:

            errors.append(
                f"{name}: missing child"
            )

        else:

            child_name = child.get("link")

            if child_name not in link_names:

                errors.append(
                    f"{name}: child link does not exist: "
                    f"{child_name}"
                )

    # --------------------------------------------------------
    # Print result
    # --------------------------------------------------------

    if errors:

        print()
        print("VALIDATION FAILED")

        for error in errors:

            print(
                f"[ERROR] {error}"
            )

        return False

    print()
    print("VALIDATION PASSED")

    return True


# ============================================================
# WRITE URDF
# ============================================================

def write_urdf(robot):

    indent(robot)

    tree = ET.ElementTree(robot)

    tree.write(
        OUTPUT_URDF,
        encoding="utf-8",
        xml_declaration=True
    )


# ============================================================
# MAIN
# ============================================================

def main():

    print()
    print("=" * 72)
    print("SOLIDWORKS → URDF V12")
    print("MESH INTEGRATION")
    print("=" * 72)

    # --------------------------------------------------------
    # Check input
    # --------------------------------------------------------

    if not INPUT_URDF.exists():

        print()
        print(
            f"[ERROR] Input URDF not found:"
        )

        print(
            f"        {INPUT_URDF}"
        )

        return

    # --------------------------------------------------------
    # Check mesh directory
    # --------------------------------------------------------

    if not MESH_DIR.exists():

        print()
        print(
            f"[WARNING] Mesh directory does not exist:"
        )

        print(
            f"          {MESH_DIR}"
        )

        print()
        print(
            "Create it and place your STL files there."
        )

        MESH_DIR.mkdir(
            parents=True,
            exist_ok=True
        )

    # --------------------------------------------------------
    # Load URDF
    # --------------------------------------------------------

    print()
    print(
        f"Input : {INPUT_URDF}"
    )

    print(
        f"Output: {OUTPUT_URDF}"
    )

    tree = ET.parse(
        INPUT_URDF
    )

    robot = tree.getroot()

    # --------------------------------------------------------
    # Validate robot
    # --------------------------------------------------------

    if robot.tag != "robot":

        raise ValueError(
            "Input file is not a valid URDF robot."
        )

    print(
        f"Robot : {robot.get('name')}"
    )

    # --------------------------------------------------------
    # Validate meshes
    # --------------------------------------------------------

    missing = validate_meshes(
        robot
    )

    # --------------------------------------------------------
    # Add available meshes
    # --------------------------------------------------------

    added = add_meshes(
        robot
    )

    # --------------------------------------------------------
    # Validate resulting URDF
    # --------------------------------------------------------

    valid = validate_urdf(
        robot
    )

    if not valid:

        print()
        print(
            "V12 aborted because URDF validation failed."
        )

        return

    # --------------------------------------------------------
    # Write
    # --------------------------------------------------------

    write_urdf(
        robot
    )

    # --------------------------------------------------------
    # Summary
    # --------------------------------------------------------

    print()
    print("=" * 72)
    print("V12 COMPLETE")
    print("=" * 72)

    print(
        f"Meshes added : {added}"
    )

    print(
        f"Meshes missing: {len(missing)}"
    )

    print(
        f"Output       : {OUTPUT_URDF}"
    )

    if missing:

        print()
        print(
            "WARNING:"
        )

        print(
            "Some links have no STL yet."
        )

        print(
            "The URDF was still generated for the "
            "available meshes."
        )

    print()
    print(
        "Next stage: load V12 into RViz."
    )


if __name__ == "__main__":
    main()