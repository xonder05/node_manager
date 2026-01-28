import sys, json

launch_file_path = sys.argv[1]

with open(launch_file_path, "r", encoding="utf-8") as f:
    content = f.read()

print(json.dumps({"file_content": content}))
