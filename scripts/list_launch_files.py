import sys, argparse, json 
from ros2launch.api.api import LaunchFileNameCompleter

package_name = sys.argv[1]

all_files = LaunchFileNameCompleter()(None, argparse.Namespace(package_name=package_name))

python_files = [file for file in all_files if file.endswith(".py")]

print(json.dumps(python_files))
