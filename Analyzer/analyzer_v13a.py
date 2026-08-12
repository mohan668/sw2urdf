import json
import math
import struct
from pathlib import Path

import numpy as np


# ============================================================
# PATHS
# ============================================================

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parent

MESH_DIR = PROJECT_DIR / "meshes"

CAD_DATA_FILE = SCRIPT_DIR / "robot_cad_data_v2.json"

OUTPUT_FILE = SCRIPT_DIR / "mesh_frame_analysis_v13a.json"


# ============================================================
# COMPONENT → STL
# ============================================================

MESH_MAP = {
    "01.Base-1": "01.Base.STL",
    "0.First_Mount-1": "0.First_Mount.STL",
    "05.UR_5_02_Nema17_Joint-1":
        "05.UR_5_02_Nema17_Joint.STL",
    "Cap_End_Effector_Joint-5":
        "Cap_End_Effector_Joint.STL",
}


# ============================================================
# NUMERICAL SETTINGS
# ============================================================

EPS = 1e-12


# ============================================================
# VECTOR HELPERS
# ============================================================

def normalize(v):

    v = np.asarray(v, dtype=float)

    n = np.linalg.norm(v)

    if n < EPS:
        return v

    return v / n


def rotation_matrix_from_transform(transform):

    """
    SolidWorks API transform format used by our CAD extractor:

        [0..8]  = rotation matrix
        [9..11] = translation
    """

    if len(transform) < 12:
        raise ValueError(
            "Transform does not contain enough values."
        )

    R = np.array(
        [
            transform[0:3],
            transform[3:6],
            transform[6:9],
        ],
        dtype=float
    )

    t = np.array(
        transform[9:12],
        dtype=float
    )

    return R, t


# ============================================================
# STL READER
# ============================================================

def is_probably_binary_stl(data):

    if len(data) < 84:
        return False

    try:

        triangle_count = struct.unpack_from(
            "<I",
            data,
            80
        )[0]

        expected_size = 84 + triangle_count * 50

        return expected_size == len(data)

    except Exception:

        return False


def read_binary_stl(data):

    if len(data) < 84:

        raise ValueError(
            "STL file is too small."
        )

    triangle_count = struct.unpack_from(
        "<I",
        data,
        80
    )[0]

    expected_size = 84 + triangle_count * 50

    if expected_size > len(data):

        raise ValueError(
            "Invalid binary STL: "
            "triangle count exceeds file size."
        )

    vertices = []

    offset = 84

    for _ in range(triangle_count):

        # Normal
        offset += 12

        # Three vertices
        for _ in range(3):

            x, y, z = struct.unpack_from(
                "<fff",
                data,
                offset
            )

            vertices.append(
                [x, y, z]
            )

            offset += 12

        # Attribute byte count
        offset += 2

    return np.asarray(
        vertices,
        dtype=float
    ), triangle_count


def read_ascii_stl(data):

    text = data.decode(
        "utf-8",
        errors="ignore"
    )

    vertices = []

    for line in text.splitlines():

        line = line.strip()

        if line.lower().startswith("vertex"):

            parts = line.split()

            if len(parts) >= 4:

                try:

                    x = float(parts[1])
                    y = float(parts[2])
                    z = float(parts[3])

                    vertices.append(
                        [x, y, z]
                    )

                except ValueError:

                    pass

    if not vertices:

        raise ValueError(
            "No vertices found in ASCII STL."
        )

    return np.asarray(
        vertices,
        dtype=float
    ), len(vertices) // 3


def read_stl(path):

    with open(
        path,
        "rb"
    ) as f:

        data = f.read()

    if is_probably_binary_stl(data):

        vertices, triangles = read_binary_stl(
            data
        )

        file_type = "binary"

    else:

        vertices, triangles = read_ascii_stl(
            data
        )

        file_type = "ascii"

    return vertices, triangles, file_type


# ============================================================
# MESH GEOMETRY
# ============================================================

def calculate_bbox(vertices):

    minimum = np.min(
        vertices,
        axis=0
    )

    maximum = np.max(
        vertices,
        axis=0
    )

    dimensions = maximum - minimum

    center = (
        minimum + maximum
    ) / 2.0

    return {
        "min": minimum,
        "max": maximum,
        "dimensions": dimensions,
        "center": center,
    }


def calculate_centroid(vertices):

    return np.mean(
        vertices,
        axis=0
    )


# ============================================================
# PCA
# ============================================================

def calculate_pca(vertices):

    """
    Geometry-derived principal directions.

    IMPORTANT:
    These are NOT necessarily the SolidWorks part axes.

    They only describe dominant geometric directions.
    """

    center = np.mean(
        vertices,
        axis=0
    )

    centered = vertices - center

    covariance = np.cov(
        centered,
        rowvar=False
    )

    eigenvalues, eigenvectors = np.linalg.eigh(
        covariance
    )

    # Largest eigenvalue first
    order = np.argsort(
        eigenvalues
    )[::-1]

    eigenvalues = eigenvalues[order]

    eigenvectors = eigenvectors[:, order]

    axes = []

    for i in range(3):

        axes.append(
            normalize(
                eigenvectors[:, i]
            )
        )

    # Ensure right-handed system
    x_axis = axes[0]
    y_axis = axes[1]
    z_axis = normalize(
        np.cross(
            x_axis,
            y_axis
        )
    )

    if np.dot(
        z_axis,
        axes[2]
    ) < 0:

        z_axis = -z_axis

    return {
        "eigenvalues": eigenvalues,
        "x": x_axis,
        "y": y_axis,
        "z": z_axis,
    }


# ============================================================
# DIMENSION / UNIT ANALYSIS
# ============================================================

def estimate_unit_scale(dimensions):

    """
    Our CAD API values are in meters.

    STL itself is unitless.

    We therefore do NOT automatically convert anything.

    This function only provides dimensional hints.
    """

    max_dimension = float(
        np.max(dimensions)
    )

    if max_dimension < 0.01:

        guess = "possibly_meters"

    elif max_dimension < 10:

        guess = "possibly_centimeters"

    elif max_dimension < 10000:

        guess = "possibly_millimeters"

    else:

        guess = "possibly_large_or_other_units"

    return {
        "largest_dimension_raw": max_dimension,
        "heuristic": guess,
        "warning":
            "Heuristic only. STL has no inherent unit."
    }


# ============================================================
# CAD DATA
# ============================================================

def load_cad_data():

    if not CAD_DATA_FILE.exists():

        print()
        print(
            "[WARNING] CAD data file not found:"
        )

        print(
            f"          {CAD_DATA_FILE}"
        )

        return None

    try:

        with open(
            CAD_DATA_FILE,
            "r",
            encoding="utf-8"
        ) as f:

            data = json.load(f)

        print(
            f"[CAD] Loaded: {CAD_DATA_FILE.name}"
        )

        return data

    except Exception as e:

        print()
        print(
            "[WARNING] Could not load CAD data:"
        )

        print(
            f"          {e}"
        )

        return None


def extract_cad_components(cad_data):

    if not cad_data:
        return {}

    components = cad_data.get(
        "components",
        []
    )

    result = {}

    for component in components:

        name = component.get(
            "name"
        )

        transform = component.get(
            "transform"
        )

        if not name:
            continue

        entry = {
            "name": name,
            "path": component.get(
                "path"
            ),
            "fixed": component.get(
                "fixed"
            ),
            "suppressed": component.get(
                "suppressed"
            ),
        }

        if transform and len(transform) >= 12:

            try:

                R, t = rotation_matrix_from_transform(
                    transform
                )

                entry["rotation_matrix"] = R
                entry["translation"] = t

            except Exception as e:

                entry["transform_error"] = str(e)

        result[name] = entry

    return result


# ============================================================
# JSON CONVERSION
# ============================================================

def vector_to_list(v):

    return [
        float(x)
        for x in np.asarray(v)
    ]


def matrix_to_list(M):

    return [
        [
            float(x)
            for x in row
        ]
        for row in np.asarray(M)
    ]


# ============================================================
# ANALYZE ONE MESH
# ============================================================

def analyze_mesh(
    component_name,
    mesh_filename,
    cad_component=None
):

    path = MESH_DIR / mesh_filename

    print()
    print(
        "-" * 72
    )

    print(
        f"COMPONENT: {component_name}"
    )

    print(
        f"STL      : {mesh_filename}"
    )

    print(
        "-" * 72
    )

    if not path.exists():

        print(
            "[ERROR] STL not found:"
        )

        print(
            f"        {path}"
        )

        return {
            "component": component_name,
            "mesh": mesh_filename,
            "status": "MISSING",
        }

    # --------------------------------------------------------
    # Read STL
    # --------------------------------------------------------

    vertices, triangle_count, file_type = read_stl(
        path
    )

    print(
        f"STL type       : {file_type}"
    )

    print(
        f"Triangles      : {triangle_count}"
    )

    print(
        f"Vertices       : {len(vertices)}"
    )

    # --------------------------------------------------------
    # Bounding box
    # --------------------------------------------------------

    bbox = calculate_bbox(
        vertices
    )

    centroid = calculate_centroid(
        vertices
    )

    pca = calculate_pca(
        vertices
    )

    units = estimate_unit_scale(
        bbox["dimensions"]
    )

    print()

    print(
        "Bounding box:"
    )

    print(
        f"  Min: {vector_to_list(bbox['min'])}"
    )

    print(
        f"  Max: {vector_to_list(bbox['max'])}"
    )

    print(
        f"  Size: {vector_to_list(bbox['dimensions'])}"
    )

    print(
        f"  Center: {vector_to_list(bbox['center'])}"
    )

    print()

    print(
        "Geometric centroid:"
    )

    print(
        f"  {vector_to_list(centroid)}"
    )

    print()

    print(
        "PCA principal directions:"
    )

    print(
        f"  X': {vector_to_list(pca['x'])}"
    )

    print(
        f"  Y': {vector_to_list(pca['y'])}"
    )

    print(
        f"  Z': {vector_to_list(pca['z'])}"
    )

    print()

    print(
        f"Unit hint: {units['heuristic']}"
    )

    # --------------------------------------------------------
    # CAD comparison
    # --------------------------------------------------------

    cad_result = None

    if cad_component:

        cad_result = {
            "component_name":
                cad_component["name"],

            "path":
                cad_component.get(
                    "path"
                ),

            "fixed":
                cad_component.get(
                    "fixed"
                ),

            "suppressed":
                cad_component.get(
                    "suppressed"
                ),
        }

        if "translation" in cad_component:

            cad_result["assembly_translation"] = \
                vector_to_list(
                    cad_component[
                        "translation"
                    ]
                )

        if "rotation_matrix" in cad_component:

            cad_result["assembly_rotation"] = \
                matrix_to_list(
                    cad_component[
                        "rotation_matrix"
                    ]
                )

        print()

        print(
            "CAD component transform:"
        )

        if "translation" in cad_component:

            print(
                "  Translation:"
            )

            print(
                f"    {vector_to_list(cad_component['translation'])}"
            )

        if "rotation_matrix" in cad_component:

            print(
                "  Rotation:"
            )

            for row in cad_component[
                "rotation_matrix"
            ]:

                print(
                    f"    {vector_to_list(row)}"
                )

        print()

        print(
            "NOTE:"
        )

        print(
            "  STL geometry and CAD component transform"
        )

        print(
            "  are being reported separately."
        )

        print(
            "  No automatic mesh correction is being applied."
        )

    return {
        "component": component_name,
        "mesh": mesh_filename,
        "path": str(path),

        "status": "ANALYZED",

        "stl": {
            "type": file_type,
            "triangle_count":
                int(triangle_count),
            "vertex_count":
                int(len(vertices)),
        },

        "geometry": {
            "bounding_box": {
                "min":
                    vector_to_list(
                        bbox["min"]
                    ),
                "max":
                    vector_to_list(
                        bbox["max"]
                    ),
                "dimensions":
                    vector_to_list(
                        bbox["dimensions"]
                    ),
                "center":
                    vector_to_list(
                        bbox["center"]
                    ),
            },

            "centroid":
                vector_to_list(
                    centroid
                ),

            "pca": {
                "x":
                    vector_to_list(
                        pca["x"]
                    ),
                "y":
                    vector_to_list(
                        pca["y"]
                    ),
                "z":
                    vector_to_list(
                        pca["z"]
                    ),

                "eigenvalues":
                    vector_to_list(
                        pca["eigenvalues"]
                    ),
            },

            "unit_analysis": units,
        },

        "cad_component":
            cad_result,

        "frame_status": {
            "stl_frame_known":
                False,

            "cad_frame_known":
                cad_component is not None,

            "exact_mesh_to_cad_transform":
                False,

            "note":
                "V13A is diagnostic only. "
                "PCA directions describe geometry, "
                "not the actual SolidWorks part coordinate frame."
        }
    }


# ============================================================
# MAIN
# ============================================================

def main():

    print()
    print(
        "=" * 72
    )

    print(
        "SOLIDWORKS → URDF V13A"
    )

    print(
        "STL / CAD FRAME DIAGNOSTIC"
    )

    print(
        "=" * 72
    )

    print()

    print(
        f"Project : {PROJECT_DIR}"
    )

    print(
        f"Meshes  : {MESH_DIR}"
    )

    print(
        f"CAD data: {CAD_DATA_FILE}"
    )

    # --------------------------------------------------------
    # Load CAD data
    # --------------------------------------------------------

    cad_data = load_cad_data()

    cad_components = extract_cad_components(
        cad_data
    )

    # --------------------------------------------------------
    # Analyze meshes
    # --------------------------------------------------------

    results = []

    for component_name, mesh_filename in MESH_MAP.items():

        cad_component = cad_components.get(
            component_name
        )

        try:

            result = analyze_mesh(
                component_name,
                mesh_filename,
                cad_component
            )

        except Exception as e:

            print()
            print(
                f"[ERROR] Failed to analyze "
                f"{mesh_filename}"
            )

            print(
                f"        {type(e).__name__}: {e}"
            )

            result = {
                "component": component_name,
                "mesh": mesh_filename,
                "status": "ERROR",
                "error": str(e),
            }

        results.append(
            result
        )

    # --------------------------------------------------------
    # Build output
    # --------------------------------------------------------

    output = {
        "format":
            "sw2urdf_mesh_frame_analysis",

        "version":
            "13A.0",

        "stage":
            "diagnostic_only",

        "coordinate_system": {
            "note":
                "No mesh coordinate transformation "
                "has been applied.",

            "cad_units":
                "SolidWorks API values are expected "
                "to be meters.",

            "stl_units":
                "STL is unitless."
        },

        "meshes":
            results,

        "analysis_notes": [
            "Bounding boxes are measured directly "
            "from STL vertices.",

            "Centroids are geometric vertex averages "
            "and are not mass centroids.",

            "PCA directions are geometry-derived and "
            "must not be treated as SolidWorks part axes.",

            "No mesh rotation or translation has been applied.",

            "No URDF file has been modified.",

            "Exact STL-to-part-frame correspondence "
            "requires SolidWorks part coordinate-system "
            "information or an equivalent exported frame."
        ]
    }

    # --------------------------------------------------------
    # Write JSON
    # --------------------------------------------------------

    with open(
        OUTPUT_FILE,
        "w",
        encoding="utf-8"
    ) as f:

        json.dump(
            output,
            f,
            indent=2
        )

    # --------------------------------------------------------
    # Summary
    # --------------------------------------------------------

    analyzed = sum(
        1
        for r in results
        if r.get("status") == "ANALYZED"
    )

    missing = sum(
        1
        for r in results
        if r.get("status") == "MISSING"
    )

    errors = sum(
        1
        for r in results
        if r.get("status") == "ERROR"
    )

    print()
    print(
        "=" * 72
    )

    print(
        "V13A COMPLETE"
    )

    print(
        "=" * 72
    )

    print(
        f"Meshes analyzed : {analyzed}"
    )

    print(
        f"Meshes missing  : {missing}"
    )

    print(
        f"Errors          : {errors}"
    )

    print()

    print(
        f"Analysis file:"
    )

    print(
        f"  {OUTPUT_FILE}"
    )

    print()

    print(
        "IMPORTANT:"
    )

    print(
        "This stage did NOT modify the URDF."
    )

    print(
        "This stage did NOT guess mesh transforms."
    )


if __name__ == "__main__":
    main()