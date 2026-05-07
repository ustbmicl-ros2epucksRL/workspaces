"""Workspace-local Python compatibility fixes for ROS catkin builds.

This only affects interpreters launched with PYTHONPATH including this folder.
It patches an older ``importlib_metadata`` so newer ``setuptools`` can import.
"""

import configparser
import importlib


def _patch_distutils():
    try:
        import distutils
    except Exception:
        return

    if not hasattr(distutils, "core"):
        try:
            distutils.core = importlib.import_module("distutils.core")
        except Exception:
            return


def _patch_importlib_metadata():
    try:
        import importlib_metadata
    except Exception:
        return

    if hasattr(importlib_metadata, "EntryPoints"):
        return

    class EntryPoints(tuple):
        __slots__ = ()

        def __new__(cls, iterable=()):
            return super().__new__(cls, tuple(iterable))

        def __getitem__(self, item):
            if isinstance(item, str):
                for entry_point in self:
                    if entry_point.name == item:
                        return entry_point
                raise KeyError(item)
            return super().__getitem__(item)

        def select(self, **params):
            return EntryPoints(
                entry_point
                for entry_point in self
                if all(getattr(entry_point, key, None) == value for key, value in params.items())
            )

        @property
        def names(self):
            return {entry_point.name for entry_point in self}

        @property
        def groups(self):
            return {entry_point.group for entry_point in self}

        @staticmethod
        def _from_text(text):
            parser = configparser.ConfigParser()
            parser.optionxform = str
            parser.read_string(text or "")
            return EntryPoints(
                importlib_metadata.EntryPoint(name=name, value=value, group=group)
                for group in parser.sections()
                for name, value in parser.items(group)
            )

    original_entry_points = importlib_metadata.entry_points

    def entry_points(**params):
        result = original_entry_points()
        if isinstance(result, dict):
            flattened = EntryPoints(
                entry_point
                for entry_points_in_group in result.values()
                for entry_point in entry_points_in_group
            )
        else:
            flattened = EntryPoints(result)
        return flattened.select(**params) if params else flattened

    importlib_metadata.EntryPoints = EntryPoints
    importlib_metadata.entry_points = entry_points


_patch_distutils()
_patch_importlib_metadata()
