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
import shutil

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
BUILD_DIR = os.path.join(PROJECT_ROOT, "build")
TIDY_CONFIG = os.path.join(PROJECT_ROOT, ".clang-tidy")
DATABASE = os.path.join(BUILD_DIR, "compile_commands.json")

TIDY_TARGETS = [
    {
        "file": "clang_tidy_cpp.cpp",
        "description": "C++ Core Logic Check"
    },
    {
        "file": "clang_tidy_objc.mm",
        "description": "Objective-C++ and Framework Check"
    }
]

def run_tidy_check(target):
    # Construct the full path to the file
    file_path = os.path.join(PROJECT_ROOT, target['file'])
    
    if not os.path.exists(file_path):
        print(f"ERROR: File not found: {file_path}")
        return 1

    print(f"\n{'='*60}")
    print(f"Running Tidy Check: {target['description']}")
    print(f"Target: {target['file']}")
    print(f"{'='*60}")

    # Build command
    command = [
        "clang-tidy",
        file_path,
        f"--config-file={TIDY_CONFIG}",
        f"-header-filter=.*",
        f"-p={BUILD_DIR}",
        "--use-color"
    ]
    
    try:
        # Changed capture_output to False so you see real-time streaming 
        # of clang-tidy output to terminal.
        result = subprocess.run(command, check=False, text=True)
        
        if result.returncode == 0:
            print(f"\nSUCCESS: {target['description']} passed.")
            return 0
        else:
            # Clang-tidy returns non-zero if it finds warnings/errors
            return result.returncode

    except FileNotFoundError:
        print("\nERROR: clang-tidy not found in PATH.")
        return 1

def main():
    # Check for clang-tidy installation
    if not shutil.which("clang-tidy"):
        print("ERROR: clang-tidy is not installed or not in PATH.")
        sys.exit(1)

    # Check for compile_commands.json
    if not os.path.exists(DATABASE):
        print(f"Error: {DATABASE} not found.")
        print("Run: cmake -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON")
        sys.exit(1)

    all_passed = True
    for target in TIDY_TARGETS:
        if run_tidy_check(target) != 0:
            all_passed = False
            # Don't break, see all errors at once
            
    if all_passed:
        print(f"\n\033[92mALL CHECKS PASSED\033[0m")
        sys.exit(0)
    else:
        print(f"\n\033[91mSOME CHECKS FAILED\033[0m")
        sys.exit(1)

if __name__ == "__main__":
    print(PROJECT_ROOT)
    print(DATABASE)
    print(TIDY_CONFIG)
    print(BUILD_DIR)
    main()