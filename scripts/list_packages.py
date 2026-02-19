import json
from ament_index_python.packages import get_packages_with_prefixes

packages = sorted(list(get_packages_with_prefixes().keys()))

print(json.dumps(packages))
