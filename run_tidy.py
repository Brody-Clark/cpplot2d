"""
This script runs clang-tidy checks on specified C++ and Objective-C++ source files
using a predefined configuration file. It assumes that the project has been set up
with CMake and that the compile_commands.json file is available in the build directory.
Usage:
    python run_tidy.py
"""

import os
import subprocess
import sys

# 1. Project structure relative to the script
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
BUILD_DIR = os.path.join(PROJECT_ROOT, "build")
TIDY_CONFIG = os.path.join(PROJECT_ROOT, ".clang-tidy")

# 2. List of files to analyze with their specific flags
# These targets assume you have already run `cmake -B build` to generate the
# compile_commands.json file inside the 'build' directory.
TIDY_TARGETS = [
    {
        "file": "clang_tidy_cpp.cpp",
        "flags": ["-std=c++17"],
        "description": "C++ Core Logic Check"
    },
    {
        "file": "clang_tidy_objc.mm",
        "flags": [
            "-x", "objective-c++", 
            "-std=c++17",
        ],
        "description": "Objective-C++ and Framework Check"
    }
]

def run_tidy_check(target):
    """Executes clang-tidy for a single target."""
    print(f"\n========================================================")
    print(f"Running Tidy Check: {target['description']}")
    print(f"Target File: {target['file']}")
    print(f"========================================================")

    # Base command: use clang-tidy, tell it where the database is
    command = [
        "clang-tidy",
        os.path.join(PROJECT_ROOT, target['file']),
        f"--config-file={TIDY_CONFIG}",
        f"-p={BUILD_DIR}",  # Directs clang-tidy to the compile_commands.json
        "--"
    ]
    
    # Append the specific compiler flags for this target
    command.extend(target['flags'])
    
    # Execute the command
    try:
        # Use check=True to raise an exception if clang-tidy returns a non-zero exit code
        subprocess.run(command, check=True, text=True, capture_output=False)
        print(f"\nSUCCESS: {target['description']} passed.")
        return 0
    except subprocess.CalledProcessError as e:
        print(e.output)
        return e.returncode
    except FileNotFoundError:
        print("\nERROR: clang-tidy command not found. Is it installed and in your PATH?")
        return 1

def main():
    if not os.path.exists(BUILD_DIR):
        print(f"Error: Build directory '{BUILD_DIR}' not found.")
        print("Please run 'cmake -B build' first to generate 'compile_commands.json'.")
        sys.exit(1)

    all_passed = True
    
    # Run checks sequentially
    for target in TIDY_TARGETS:
        result = run_tidy_check(target)
        if result != 0:
            all_passed = False
            # Break immediately on first failure for quick feedback
            break 
            
    if all_passed:
        print("\n========================================================")
        print(f"{'\033[92m'}ALL CLANG-TIDY CHECKS PASSED SUCCESSFULLY!{'\033[0m'}")
        print("========================================================")
        sys.exit(0)
    else:
        print("\n========================================================")
        print(f"{'\033[93m'}CLANG-TIDY CHECKS FAILED. Please review errors above.{'\033[0m'}")
        print("========================================================")
        sys.exit(1)

if __name__ == "__main__":
    main()