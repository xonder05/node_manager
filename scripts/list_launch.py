import argparse, json, sys; 
from ros2launch.api.api import LaunchFileNameCompleter;

package_name = sys.argv[1]
files = LaunchFileNameCompleter()(None, argparse.Namespace(package_name=package_name))
print(json.dumps(files))
