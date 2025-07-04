import types
import sys
import pathlib
from unittest import mock

# Utility to load only the find_esp32_port function without running the script

def load_find_esp32_port():
    file_path = pathlib.Path(__file__).resolve().parents[1] / 'Joystick' / 'joystick_to_serial.py'
    with open(file_path, 'r') as f:
        source = f.read()
    # Keep only the part before the example usage and runtime code
    source = source.split('# Example usage')[0]

    # Ensure stub modules exist for imports not installed in test env
    sys.modules.setdefault('pygame', types.ModuleType('pygame'))
    serial_mod = types.ModuleType('serial')
    tools_mod = types.ModuleType('serial.tools')
    serial_mod.tools = tools_mod
    list_ports = types.ModuleType('serial.tools.list_ports')
    list_ports.comports = lambda: []
    tools_mod.list_ports = list_ports
    sys.modules.setdefault('serial', serial_mod)
    sys.modules.setdefault('serial.tools', tools_mod)
    sys.modules.setdefault('serial.tools.list_ports', list_ports)

    module = types.ModuleType('joystick_stub')
    exec(source, module.__dict__)
    return module.find_esp32_port


def test_find_esp32_port_none():
    find_esp32_port = load_find_esp32_port()
    with mock.patch('platform.system', return_value='Linux'), \
         mock.patch('os.path.exists', return_value=False), \
         mock.patch('serial.tools.list_ports.comports', return_value=[]):
        assert find_esp32_port() is None
