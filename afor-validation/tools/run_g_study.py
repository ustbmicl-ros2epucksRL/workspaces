"""Run the registered G-only three-formation training and independent test."""
from afor_local.__main__ import main


if __name__ == '__main__':
    import sys
    main(['study', '--execute', *sys.argv[1:]])
