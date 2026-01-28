from rosidl_runtime_py import get_interface_path
import sys, json

interface_name = sys.argv[1]
path = get_interface_path(interface_name=interface_name)
print(json.dumps({"interface_path": path}))
