"""Example entry point.

Run on the Pi after `git push <remote> <branch>`:

    ssh <user>@<pi> "cd /var/www/<project> && .venv/bin/python main.py"
"""
#import platform
#import time
from transmitting import main

#def main() -> None:
    #print(f"Hello from {platform.node()}")
    #print(f"Python {platform.python_version()} on {platform.system()} {platform.release()}")
    #print(f"Local time: {time.strftime('%Y-%m-%d %H:%M:%S')}")


if __name__ == "__main__":
    main()
