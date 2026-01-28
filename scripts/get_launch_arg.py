import json
from launch.launch_description_sources import get_launch_description_from_any_launch_file
import sys

launch_file_path = sys.argv[1]

args_list = get_launch_description_from_any_launch_file(launch_file_path=launch_file_path).get_launch_arguments()

result = {}
for arg in args_list:
    result[arg.name] = ""

    for substitution in arg.default_value:
        string = substitution.describe().strip("\"'")
        result[arg.name] = result[arg.name] + string

print(json.dumps(result))