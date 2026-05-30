#!/usr/bin/env python3
import sys
import subprocess
import re

def main():
    # Pass all arguments (excluding the script name itself) to xacro
    args = ['xacro'] + sys.argv[1:]
    try:
        # Run xacro and get output
        res = subprocess.run(args, capture_output=True, text=True, check=True)
        xml = res.stdout
        # Strip comments
        cleaned = re.sub(r'<!--.*?-->', '', xml, flags=re.DOTALL)
        print(cleaned)
    except subprocess.CalledProcessError as e:
        sys.stderr.write(e.stderr)
        sys.exit(e.returncode)

if __name__ == '__main__':
    main()
