# Contributing to cpplot2d

This project aims to be a **minimal, fast, header-only, cross-platform 2D plotting library in C++**, with no third-party runtime dependencies.

To keep the codebase clean, stable, and easy to maintain, please follow the guidelines below.

---

## Table of Contents

* [Project Philosophy](#project-philosophy)
* [Code Style and Quality](#code-style-and-quality)

  * [Formatting with Clang-Format](#formatting-with-clang-format)
  * [Code Quality and Static Analysis with Clang-Tidy](#code-quality-and-static-analysis-with-clang-tidy)
* [Testing](#testing)
* [How to Create a Pull Request](#how-to-create-a-pull-request)

  * [Branching and Workflow](#branching-and-workflow)
  * [Pull Request Requirements](#pull-request-requirements)
  * [PR Title and Summary](#pr-title-and-summary)
* [Versioning and Releases](#versioning-and-releases)

---

## Project Philosophy

Before contributing, please keep the following core principles in mind:

* **Header-only**: Public API and implementations must lives in the single header file.
* **No third-party dependencies**: Platform code uses native OS APIs only.
* **Portable C++**: Prefer standard C++17 features and idioms.
* **Small, composable changes**: This project favors many small PRs over large ones.
* **Trunk-based development**: `main` is always releasable.

If a proposed change significantly increases complexity, build requirements, or dependency surface, it may be rejected even if technically correct.

---

## Code Style and Quality

### Formatting with Clang-Format

This project uses **Clang-Format** to enforce a consistent code style.

* All submitted code **must** be formatted before review
* Formatting rules are defined in the `.clang-format` file at the repo root

#### How to Format Your Code

**Single file:**

```sh
clang-format -i path/to/your/file.cpp
```

**Check only (used in CI):**

```sh
clang-format --Werror --dry-run path/to/your/file.cpp
```

**Format all staged files (recommended git alias):**

```sh
alias format="git diff --cached --name-only --diff-filter=ACMRTUXB | grep -E '\.(cpp|cc|c|h|hpp)$' | xargs clang-format -i"
```

---

### Recommended Editor Setup

It is strongly encouraged to run Clang-Format automatically on save.

| Editor           | Setup                                   |
| ---------------- | --------------------------------------- |
| **VS Code**      | C/C++ Extension Pack + “Format on Save” |
| **Vim / NeoVim** | LSP or `vim-clang-format`               |
| **CLion**        | Configure Clang-Format executable       |

---

### Code Quality and Static Analysis with Clang-Tidy

Static analysis is enforced using **clang-tidy**, configured via `.clang-tidy`.

The provided script `run_tidy.py` runs clang-tidy against:

* `clang_tidy_cpp.cpp`
* `clang_tidy_objc.mm`

This ensures both C++ and Objective-C++ paths are analyzed.

#### Running clang-tidy

First build the project:

```sh
cmake --build build
```

Then run:

```sh
python3 run_tidy.py
```

Any **clang-tidy errors must be resolved** before a PR can be merged.

#### Merge Blocking Conditions

A PR **will not be merged** if:

1. Clang-Format fails
2. Clang-Tidy reports any **errors**
   (warnings may be acceptable, errors are not)

---

## Testing

Unit and integration tests live in the `tests/` directory.
Testing infrastructure is **work in progress**.

When adding new features or fixing bugs:

* Add tests when feasible
* Avoid breaking existing tests

---

## How to Create a Pull Request

This project uses **trunk-based development**.

The `main` branch is always expected to be:

* Buildable
* Usable
* Close to releasable

### Branching and Workflow

1. Create a feature branch from the latest `main`:

```sh
git checkout main
git pull origin main
git checkout -b feature/short-descriptive-name
```

2. Commit frequently with clear messages
3. Squash trivial WIP commits before pushing if needed
4. Open a PR targeting **`main`**

---

### Pull Request Requirements

Before review, your PR must:

* Pass all CI checks (formatting, static analysis)
* Include tests where applicable
* Be **atomic** (one clear purpose per PR)

---

### PR Title and Summary

Clear PR descriptions enable fast, high-quality reviews.

| Component   | Requirement              | Example                                                                            |
| ----------- | ------------------------ | ---------------------------------------------------------------------------------- |
| **Title**   | Short, descriptive       | `feat: Add axis-aligned grid rendering`                                            |
| **Summary** | What changed + why       | “Adds optional grid rendering to Plot2D. Improves readability for dense datasets.” |
| **Issues**  | Reference related issues if applicable | `Fixes #42`                                                          |

---

## Versioning and Releases

### Version Source of Truth

* The **version number is defined in the public header**
* **Git tags** mark stable releases
* Tagged releases are available on the repository homepage

### Release Process

1. `main` accumulates changes under a **`-dev` version**
2. When ready:

   * A release commit removes `-dev`
   * A Git tag is created (e.g. `v1.4.0`)

3. Immediately after tagging:

   * The version is bumped to the next **minor** version
   * `-dev` is re-added (e.g. `1.5.0-dev`)

### Example

```text
1.4.0-dev   - development on main
1.4.0       - tagged stable release
1.5.0-dev   - development resumes
```

### Contributor Guidelines for Versioning

* **Do not** change version numbers in PRs unless explicitly requested
* Assume your changes land in the next `-dev` version
* Breaking API changes should be clearly documented in the PR description

---

## Notes

If you’re unsure whether a change fits the project direction, open an issue or draft PR first. Discussion is encouraged!
