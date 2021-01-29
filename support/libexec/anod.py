#!/usr/bin/env python3

"""Simple front-end to the task-specific anod scripts contained in lib."""

from e3.main import Main

from argparse import SUPPRESS, ArgumentParser, _HelpAction


class CustomHelpAction(_HelpAction):
    """
    Custom help action for ArgumentParser.

    This action will defer printing help if it sees that the command positional
    argument has already been given and it is on the first pass. This allows
    more specific help to be retrieved by placing the the help option after the
    command argument.

    Once the first round of argument parsing is completely, call increment_pass
    """

    def __init__(self, option_strings, dest=SUPPRESS, default=SUPPRESS, help=None):
        super(CustomHelpAction, self).__init__(
            option_strings=option_strings,
            dest=dest,
            default=default,
            help=help,
        )
        self.current_pass = 1

    def __call__(
        self,
        parser,
        namespace,
        values,
        option_string=None,
    ):
        if self.current_pass > 1 or getattr(namespace, "command", None) is None:
            parser.print_help()
            parser.exit()

    def increment_pass(self) -> None:
        """Increment the current pass number."""
        self.current_pass += 1


if __name__ == "__main__":
    m = Main(argument_parser=ArgumentParser(add_help=False))

    command_arg = m.argument_parser.add_argument(
        "command",
        choices=["build", "printenv", "devel-setup", "configure-vscode"],
        help="the subcommand to be run.",
    )

    help_arg = m.argument_parser.add_argument(
        "-h",
        "--help",
        action=CustomHelpAction,
        help="show this help message and exit",
    )

    m.parse_args(known_args_only=True)

    # Now that we've parsed the command, we don't want it showing up in help
    # for the subcommands
    command_arg.help = SUPPRESS
    help_arg.increment_pass()

    if m.args.command == "build":
        from anod_build import do_build

        exit(do_build(m))

    elif m.args.command == "printenv":
        from anod_printenv import do_printenv

        exit(do_printenv(m))

    elif m.args.command == "devel-setup":
        from anod_devel_setup import do_devel_setup

        exit(do_devel_setup(m))

    else:
        # cannot happen
        exit(4)
