"""Parse a TOML configuration file and generate corresponding Python constants file."""

import re
from datetime import datetime
from pathlib import Path

import click
import toml

import lib.paths as paths


def to_snake_case(name):
    """
    Convert a given CamelCase string to snake_case.

    Args:
        name (str): The CamelCase string to convert.

    Returns
    -------
        str: The converted snake_case string.
    """
    pattern = re.compile(r'(?<!^)(?=[A-Z])')
    return pattern.sub('_', name).lower()

@click.command()
@click.argument('toml_path', type=click.Path(exists=True), required=True)
@click.argument('py_path',
                type=click.Path(),
                required=False,
                default=str(paths.REPO_PATH / 'lib' / 'constants.py'))
def main(toml_path, py_path):
    """
    Parse a TOML configuration file and generate corresponding Python constant file.

    Args:
        toml_path (str): Path to the TOML configuration file.
        py_path (str): Path to the Python constants file.
    """
    # Load the TOML file
    with open(toml_path, 'r') as file:
        constants = toml.load(file)

    # Define path for the .py file
    py_file_path = Path(py_path)
    if not py_file_path.exists():
        py_file_path.parent.mkdir(parents=True, exist_ok=True)
        with open(py_file_path, 'w') as py_file:
            py_file.write('"""' + "Auto-generated Python constants file" + '"""' + '\n\n')
    py_file_path = Path(py_path).with_suffix('.py')
    py_file_path.unlink()
    py_file_path = py_file_path.as_posix()
    # Start writing Python file
    with open(py_file_path, 'w') as py_file:
        file_header_msg = f"Auto-generated constants from TOML configuration {toml_path}."
        py_file.write('"""' + file_header_msg + '"""' + '\n\n')

        # Write initial Python import statements

        # path to original config toml for back reference
        key = "TOML_PATH"
        py_file.write(f'{key.upper()} = "{toml_path}"\n')

        # Iterate through each section and generate definitions
        for section, values in constants.items():

            assert isinstance(values, dict)
            for key, value in values.items():
                _full_key_name = f"CFG_{section.upper()}_{key}".upper()
                py_key_name = f"{section.upper()}_{key}".upper()
                if isinstance(value, list):
                    if all(isinstance(x, str) for x in value):
                        value_str = ', '.join(f'"{x}"' for x in value)
                        py_value_str = ', '.join(f'"{x}"' for x in value)
                    elif all(isinstance(x, (int, float)) for x in value):
                        value_str = ', '.join(f"{x}" for x in value)
                        py_value_str = value_str
                    py_file.write(f"{py_key_name} = [{py_value_str}]\n")
                elif isinstance(value, str):
                    py_file.write(f'{py_key_name} = "{value}"\n')
                elif isinstance(value, (int, float)):
                    py_file.write(f'{py_key_name} = {value}\n')

if __name__ == '__main__':
    main()