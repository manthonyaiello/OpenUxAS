"""Common path definitions for anod."""
import os

__ROOT_DIR = os.path.dirname(os.path.abspath(__file__))

REPO_DIR = os.path.realpath(
    os.path.join(
        __ROOT_DIR,
        "..",
        "..",
        "..",
    )
)

SUPPORT_DIR = os.path.join(
    REPO_DIR,
    "support",
)

SPEC_DIR = os.path.join(
    REPO_DIR,
    SUPPORT_DIR,
    "specs",
)

SBX_DIR = os.path.join(
    REPO_DIR,
    SUPPORT_DIR,
    "sbx",
)
