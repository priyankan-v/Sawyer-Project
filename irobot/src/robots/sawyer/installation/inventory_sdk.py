"""Index public SDK methods without importing ROS or executing SDK code."""

# Standalone development tool.
# ruff: noqa: INP001
from __future__ import annotations

import argparse
import ast
import hashlib
import json
from pathlib import Path

PACKAGES = ('intera_interface', 'intera_io', 'intera_motion_interface')


def inventory(sdk):
    files, operations = {}, []
    for package in PACKAGES:
        directories = sorted(sdk.glob(f'*/src/{package}'))
        for path in sorted(
            path
            for directory in directories
            for path in directory.rglob('*.py')
        ):
            directory = next(
                root for root in directories if path.is_relative_to(root)
            )
            relative = path.relative_to(sdk).as_posix()
            files[relative] = hashlib.sha256(path.read_bytes()).hexdigest()
            module = path.relative_to(directory.parent).with_suffix('')
            module_name = '.'.join(module.parts)
            tree = ast.parse(path.read_text())
            for item in tree.body:
                if not isinstance(item, ast.ClassDef):
                    continue
                for method in item.body:
                    if not isinstance(
                        method, ast.FunctionDef
                    ) or method.name.startswith('_'):
                        continue
                    operations.append(
                        {
                            'operation': (
                                f'{module_name}.{item.name}.{method.name}'
                            ),
                            'arguments': ast.unparse(method.args),
                            'source': relative,
                            'line': method.lineno,
                        }
                    )
    return {
        'scope': 'Public class methods; source index, not a parity claim',
        'files': files,
        'operations': operations,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output.write_text(json.dumps(inventory(args.sdk), indent=2) + '\n')


if __name__ == '__main__':
    main()
