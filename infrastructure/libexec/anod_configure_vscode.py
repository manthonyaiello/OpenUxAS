#!/usr/bin/env python3

"""Anod vscode configurator."""

from __future__ import annotations

from anod.util import check_common_tools, create_anod_context, create_anod_sandbox
from anod.paths import SPEC_DIR, SBX_DIR, REPO_DIR

from e3.main import Main
from e3.env import BaseEnv

import logging
import os
import pathlib
from string import Template

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from typing import Optional


# Template for the c_cpp_properties.json file we generate
C_CPP_PROPERTIES_JSON = Template(
    """\
{
    "configurations": [
        {
            "name": "OpenUxAS",
            "includePath": [
                "$${workspaceFolder}/**",
                "${paths}"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "gnu11",
            "cppStandard": "gnu++11",
            "intelliSenseMode": "gcc-x64"
        }
    ],
    "version": 4
}
"""
)

TASKS_JSON = Template(
    """\
{
    // See https://go.microsoft.com/fwlink/?LinkId=733558
    // for the documentation about the tasks.json format
    "version": "2.0.0",
    "tasks": [
        {
            "label": "Build OpenUxAS C++",
            "type": "shell",
            "command": "PATH=\"${bootstrap}/vpython/bin\":\"$$PATH\" make -j all",
            "problemMatcher": [],
            "group": {
                "kind": "build",
                "isDefault": true
            }
        },

        {
            "label": "Clean OpenUxAS C++",
            "type": "shell",
            "command": "PATH=\"${bootstrap}/vpython/bin\":\"$$PATH\" make clean",
            "problemMatcher": [],
            "group": "none",
        },
    ]
}
"""
)

DEFAULT_PATH = "develop/OpenUxAS"

VSCODE_DIR = ".vscode"
C_CPP_PROPERTIES_FILENAME = "c_cpp_properties.json"
TASKS_FILENAME = "tasks.json"


def do_configure(m: Main, uxas_dir: Optional[str] = None, set_prog: bool = True) -> int:
    """Create the configuration file for VS Code."""
    if set_prog:
        m.argument_parser.prog = m.argument_parser.prog + " configure-vscode"

    m.argument_parser.add_argument(
        "spec_name",
        nargs="?",
        help="spec for which the VS Code configuration should be generated",
        default="uxas",
    )

    m.argument_parser.add_argument("--qualifier", help="optional qualifier")
    m.argument_parser.add_argument(
        "--sandbox-dir",
        help="directory in which build artifacts are stored",
        default=SBX_DIR,
    )

    m.argument_parser.add_argument(
        "--out-dir",
        help="specify the output directory that will contain the .vscode directory "
        "and generated contents",
        default=DEFAULT_PATH,
    )

    m.parse_args()

    # Disable logging messages except errors
    logging.getLogger("").setLevel(logging.ERROR)

    check_common_tools()

    ac = create_anod_context(SPEC_DIR)
    sbx = create_anod_sandbox(m.args.sandbox_dir, SPEC_DIR)

    anod_instance = ac.add_anod_action(
        name=m.args.spec_name,
        primitive="build",
        qualifier=m.args.qualifier,
        sandbox=sbx,
        upload=False,
        env=BaseEnv.from_env(),
    ).anod_instance

    if hasattr(anod_instance, "build_setenv"):
        anod_instance.build_setenv()

        if uxas_dir is not None:
            abspath = os.path.abspath(uxas_dir)
        else:
            abspath = os.path.abspath(m.args.out_dir)

        vscode_dir = os.path.join(abspath, VSCODE_DIR)
        c_cpp_file = os.path.join(vscode_dir, C_CPP_PROPERTIES_FILENAME)
        task_file = os.path.join(vscode_dir, TASKS_FILENAME)

        if not os.path.exists(vscode_dir):
            pathlib.Path(vscode_dir).mkdir(parents=True)

        c_cpp_content = C_CPP_PROPERTIES_JSON.substitute(
            paths='",\n                "'.join(
                os.environ["CPLUS_INCLUDE_PATH"].split(os.pathsep)
            )
        )
        open(c_cpp_file, "w").write(c_cpp_content)

        if m.args.spec_name == "uxas":
            task_content = TASKS_JSON.substitute(bootstrap=REPO_DIR)
            open(task_file, "w").write(task_content)

        return 0
    else:
        logging.error(
            f"Cannot generate a VS Code configuration for {m.args.spec_name} "
            "because it does not export a build_setenv"
        )

        return 1
