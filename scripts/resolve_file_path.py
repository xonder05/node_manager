import sys, json
from ros2launch.api.api import get_share_file_path_from_package; 

package_name = sys.argv[1]
file_name = sys.argv[2]

path = get_share_file_path_from_package(package_name=package_name, file_name=file_name)

print(json.dumps({"file_path": path}))
