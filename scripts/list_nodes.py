import sys, os, json
from ros2pkg.api import get_executable_paths

package_name = sys.argv[1]

paths = get_executable_paths(package_name=package_name)

print(json.dumps([os.path.basename(p) for p in paths]))
