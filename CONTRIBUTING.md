# Contributing to cpplot2d

Table of Contents:

- 

## Code Style and Quality

### Formatting

To ensure uniform code style across the entire library, this project uses **Clang-Format**. This tool automatically adjusts spacing, indentation, line breaks, and brace styles according to a predefined configuration.

- Ensure code is formatted before submitting a pull request
- The formatting configuration is defined in the .clang-format file in the repo root

#### How to Format Your Code

You **must** run `clang-format` on your changes before submitting a Pull Request (PR).

* **Single File:**

```sh
clang-format -i path/to/your/file.cpp
```

* **Checking (CI Only):** To see what changes would be made without applying them:

```sh
clang-format --Werror --dry-run path/to/your/file.cpp
```

A handy git alias to format all staged files with a simple "format" command:

```sh
alias format="git diff --cached --name-only --diff-filter=ACMRTUXB | grep -E '\.(cpp|cc|c|h|hpp)$' | xargs clang-format -i"
```

#### Recommended Editor Setup

It is strongly encouraged that contributors set up their IDE or text editor to run Clang-Format on save.

| Editor | Extension/Method |
| :--- | :--- |
| **VS Code** | C/C++ Extension Pack (Enable "Format on Save") |
| **Vim/NeoVim** | Plugins like `vim-clang-format` or LSP integration |
| **CLion** | Configure the tool settings to use the Clang-Format executable |

### Code Quality and Static Analysis with Clang-Tidy

The checks Clang-Tidy performs are governed by the `.clang-tidy` configuration file, which defines which categories of checks are enabled (e.g., `bugprone-*`, `readability-*`, `modernize-*`).

### Integration with CMake

Clang-Tidy is integrated directly into the build process using CMake's toolchain features. When you build the project locally, Clang-Tidy runs automatically on the files that are being compiled.

To run the full suite of checks locally, use the following CMake command during configuration:

```bash
# Configure the build directory
cmake -B build -DCMAKE_CXX_CLANG_TIDY='clang-tidy;--config-file=${workspaceFolder}/.clang-tidy'
```

Then, run the build:

```bash
cmake --build build
```

Any warnings or errors reported by Clang-Tidy must be resolved before your code can be merged.

### Pull Request Requirements

The CI pipeline for every Pull Request will automatically run both Clang-Format and Clang-Tidy.

**A PR will not be merged if:**

1. The Clang-Format check fails (meaning your code is not formatted according to the `.clang-format` file).
2. The Clang-Tidy check reports any **error** (warning level may be allowed, but errors are strictly prohibited).

## Testing

Unit tests are located in the tests/ directory. (WIP — contributions welcome.)
