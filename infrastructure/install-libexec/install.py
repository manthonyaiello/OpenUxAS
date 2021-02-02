#! /usr/bin/env python3

"""
Install front-end script.

This script provides a front-end for installing components needed to be able to
run the anod build of OpenUxAS.

Run this script from the root of bootstrap, like this:

    python3 util/install

To get more information and to control the script's behavior, run:

    python3 util/install --help
"""

from __future__ import annotations

import os
import sys

from support.arguments import (
    add_logging_group,
    add_interactive_group,
    add_force_argument,
    add_dry_run_argument,
)
from support.commands import Command, run_command_and_exit_on_fail
from support.log import configure_logging
from support.paths import INSTALL_LIBEXEC_DIR


GNAT_INSTALL = Command(
    cmd=[sys.executable, os.path.join(INSTALL_LIBEXEC_DIR, "install-gnat.py")],
    description="Install gnat",
)

GNAT_ENV = Command(
    cmd=GNAT_INSTALL.cmd + ["--printenv"],
)

VENV_INSTALL = Command(
    cmd=[sys.executable, os.path.join(INSTALL_LIBEXEC_DIR, "install-anod-venv.py")],
    description="Install anod venv",
)


def pass_args(command: Command) -> Command:
    """Add all arguments provided to this script to a command."""
    command.cmd += sys.argv[1:]
    return command


DESCRIPTION = """\
This script automates the installation of components needed to be able to run
the anod build of OpenUxAS. You should generally run this script from the root
of bootstra, like this:

    `python3 util/install`
"""

if __name__ == "__main__":
    from argparse import ArgumentParser

    argument_parser = ArgumentParser(
        description=DESCRIPTION,
    )

    add_dry_run_argument(argument_parser)
    add_force_argument(argument_parser)

    add_interactive_group(argument_parser)

    meta_gnat_group = argument_parser.add_argument_group("GNAT Community installation")
    gnat_group = meta_gnat_group.add_mutually_exclusive_group()
    gnat_group.add_argument(
        "--gnat",
        dest="install_gnat",
        action="store_true",
        default=True,
        help="install GNAT community; run `util/install-gnat --help` for "
        "additional gnat options",
    )
    gnat_group.add_argument(
        "--no-gnat",
        dest="install_gnat",
        action="store_false",
        help="do not install GNAT community",
    )

    meta_anod_group = argument_parser.add_argument_group(
        "anod virtual environment installation"
    )
    anod_group = meta_anod_group.add_mutually_exclusive_group()
    anod_group.add_argument(
        "--anod",
        dest="install_anod_venv",
        action="store_true",
        default=True,
        help="install anod virtual environment; run `util/install-anod --help`"
        " for additional anod options",
    )
    anod_group.add_argument(
        "--no-anod",
        dest="install_anod_venv",
        action="store_false",
        help="do not install anod virtual environment",
    )

    add_logging_group(argument_parser)

    (args, _) = argument_parser.parse_known_args()

    configure_logging(args)

    set_no_update = False
    installed_gnat = False
    installed_venv = False

    if args.install_gnat and (
        not args.interactive
        or input(
            "Install GNAT community (optional; needed to build Ada services)? [Y/n] "
        )
        != "n"
    ):
        command = pass_args(GNAT_INSTALL)

        if set_no_update:
            command.cmd.append("--no-update")

        run_command_and_exit_on_fail(command)
        set_no_update = True
        installed_gnat = True

    if args.install_anod_venv and (
        not args.interactive or input("Install anod virtual environment? [Y/n] ") != "n"
    ):
        command = pass_args(VENV_INSTALL)

        if set_no_update:
            command.cmd.append("--no-update")

        run_command_and_exit_on_fail(command)
        set_no_update = True
        installed_venv = True
