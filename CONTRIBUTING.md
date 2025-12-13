# Contributing to cpplot2d

Table of Contents:

- [Code Style and Quality](#code-style-and-quality)
  - [Formatting with Clang-Format](#formatting-with-clang-format)
  - [Code Quality and Static Analysis with Clang-Tidy](#code-quality-and-static-analysis-with-clang-tidy)
- [Testing](#testing)
- [How to Create a Pull Request](#how-to-create-a-pull-request)
  - [The Workflow](#the-workflow)
  - [Pull Request Requirements](#pull-request-requirements)
  - [PR Title and Summary](#pr-title-and-summary)

## Code Style and Quality

### Formatting with Clang-Format

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

The checks clang-tidy performs are governed by the `.clang-tidy` configuration file, which defines which categories of checks are enabled (e.g., `bugprone-*`, `readability-*`, `modernize-*`).

#### Running clang-tidy

For simplicity, a python script `run_tidy.py` runs clang-tidy on target files `clang_tidy_cpp.cpp` and `clang_tidy_objc.mm` to target both the cpp and objc code in the library.

To run this script, make sure you have python installed and the project is built using:

```sh
cmake --build build
```

then run the tidy script using:

```sh
python3 run_tidy.py
```

Any errors reported by clang-tidy must be resolved before your code can be merged.

**A PR will not be merged if:**

1. The Clang-Format check fails (meaning your code is not formatted according to the `.clang-format` file).
2. The Clang-Tidy check reports any **error** (warning level may be allowed, but errors are strictly prohibited).

## Testing

Unit tests are located in the tests/ directory. (WIP — contributions welcome.)

## How to Create a Pull Request

This project follows a trunk-based development strategy, meaning all changes are merged directly into the `main` branch frequently. This requires small, focused, and well-described PRs.

### The Workflow

1. **Branching:** Create a new feature branch off of the latest `main` branch.

```sh
    git checkout main
    git pull origin main
    git checkout -b feature/my-new-feature-name
```

2. **Commit:** Write clear, descriptive commit messages locally. Commit your changes frequently and squash minor work-in-progress commits before pushing, if necessary, to keep the history clean.
3. **Push:** Push your local feature branch to the remote repository.
4. **Open PR:** Open a Pull Request targeting the **`main`** branch.

### Pull Request Requirements

A PR will not be reviewed or merged until the following criteria are met:

* **Passing Checks:** All Continuous Integration (CI) checks—including **Clang-Format** and **Clang-Tidy**—must pass. (see previous sections for information on clang-tidy and clang-format)
* **Unit Tests:** All existing unit tests must pass, and new unit tests must be added to cover the new functionality or fixes.
* **Atomic Changes:** The PR must represent the smallest possible unit of work. Large PRs will be sent back for splitting.

### PR Title and Summary

The quality of the PR description is key to fast reviews. Please ensure your PR summary includes:

| Component | Requirement | Example |
| :--- | :--- | :--- |
| **Title** | Clear & concise description of the main change. | `feat: Added Color initializers` |
| **Summary** | Brief summary of the changes and why they were made. | "Refactors `Color` struct to use C++17 default member initializers and adds unit tests for common colors. Fixes issue with non-opaque default alpha." |
| **Linked Issues** | If applicable, reference to any related issue (e.g., `Fixes #123`). | `Fixes #42` |