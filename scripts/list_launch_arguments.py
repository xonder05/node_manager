import sys, json
from ros2launch.api.api import get_share_file_path_from_package; 
from launch.launch_description_sources import get_launch_description_from_python_launch_file

package_name = sys.argv[1]
file_name = sys.argv[2]

launch_file_path = get_share_file_path_from_package(package_name=package_name, file_name=file_name)

args_list = get_launch_description_from_python_launch_file(python_launch_file_path=launch_file_path).get_launch_arguments()

result = {}
for arg in args_list:
    result[arg.name] = ""

    for substitution in arg.default_value:
        string = substitution.describe().strip("\"'")
        result[arg.name] = result[arg.name] + string

print(json.dumps(result))
