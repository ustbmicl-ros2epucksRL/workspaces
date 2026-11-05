"""Train through the project's single G-policy command interface."""
import sys
from afor_local.__main__ import main as project_main


def main(argv=None):
    return project_main(['train', *(sys.argv[1:] if argv is None else argv)])


if __name__ == '__main__':
    main()
