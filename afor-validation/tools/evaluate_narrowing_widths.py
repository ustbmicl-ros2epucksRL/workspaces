"""Evaluate an explicit G actor across the registered opening widths."""
import sys
from afor_local.__main__ import main as project_main


def main(argv=None):
    options = list(sys.argv[1:] if argv is None else argv)
    if not any(option == '--widths' or option.startswith('--widths=') for option in options):
        options += ['--widths', '1.1', '1.3', '1.4', '1.5', '1.6']
    if not any(option == '--jitters' or option.startswith('--jitters=') for option in options):
        options += ['--jitters', '0']
    return project_main(['evaluate', *options])


if __name__ == '__main__':
    main()
