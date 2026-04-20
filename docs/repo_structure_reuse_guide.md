# Reusable C++ Repository Structure and CMake Design Pattern

## Purpose

This document defines a reusable repository and CMake design pattern for small-to-medium C++ projects.

It is intended to be generic enough to apply across different codebases while being concrete enough for an AI coding agent to implement without guesswork.

The pattern emphasizes:

- a thin repository root
- clear separation between public API, implementation, demos, and local configuration
- explicit target-based dependency wiring in CMake
- optional feature layering
- reproducible structure that scales without becoming ad hoc

## Core Design Principles

### 1. Organize around targets, not files

The build system should describe the project in terms of library and executable targets.

Files exist to support targets. Targets are the public build contract.

An agent should prefer:

- `add_library(...)`
- `add_executable(...)`
- `target_link_libraries(...)`
- `target_include_directories(...)`
- namespaced alias targets

An agent should avoid:

- global include path injection
- project-wide compile flag mutation unless truly intended
- wiring demos directly to raw paths instead of library targets

### 2. Keep the root shallow

The repository root should make the high-level structure obvious.

Recommended top-level layout:

```text
project-root/
|-- CMakeLists.txt
|-- README.md
|-- cmake/
|-- include/
|-- src/                 # only if compiled implementation exists
|-- demo/                # optional
|-- tests/               # optional
|-- docs/                # optional but recommended
`-- build/               # generated, not source-controlled
```

Not every project needs every folder, but the folder roles should stay consistent.

### 3. Separate public API from implementation

Public headers should live in `include/`.

Implementation files should live in `src/` if the project is not header-only.

This gives a predictable rule:

- `include/` defines what consumers are allowed to include
- `src/` contains implementation details that should not be included directly by external consumers

### 4. Separate core functionality from optional integrations

If a project has optional features such as:

- visualization
- GUI adapters
- platform backends
- format import/export helpers
- external tool integrations

they should not be mixed into the core target unless they are truly mandatory.

Instead, create layered targets:

```text
core target
    ->
optional extension target(s)
    ->
demo/test/app target(s)
```

This keeps the dependency graph explicit and prevents optional features from becoming hidden requirements.

### 5. Isolate machine-local paths

Developer-specific paths should not be hardcoded into the main build logic.

If the project relies on local checkout paths or non-installed dependencies, place those settings in a dedicated file such as:

- `cmake/local_paths.cmake`

Use cached path variables:

```cmake
set(MYPROJECT_SOME_DEP_PATH "C:/path/to/dependency" CACHE PATH "Path to dependency.")
```

This keeps the root build file reusable and reduces environment-specific edits.

### 6. Use options for optional build surface

Optional parts of the build should be controlled with `option(...)`.

Typical candidates:

- demos
- tests
- examples
- optional backends
- optional adapters

This avoids forcing every consumer to build every target.

## Recommended Folder Roles

### `CMakeLists.txt`

The root `CMakeLists.txt` should:

- define the project
- set language and standard requirements
- include local configuration files from `cmake/` when needed
- declare the primary target graph
- enable optional subgraphs behind options

It should not become a dumping ground for unrelated setup.

### `cmake/`

This folder should hold CMake helper files with focused responsibilities.

Examples:

- `local_paths.cmake`
- toolchain helpers
- reusable macros for packaging, warnings, or install logic

This keeps the root build script short and readable.

### `include/`

This folder should contain public headers only.

Recommended rules:

- headers here should be safe for downstream consumers to include
- names and layout should reflect the intended public API
- optional public helpers may also live here if they are part of the supported interface

### `src/`

Use `src/` only when compiled implementation exists.

Recommended rules:

- `.cpp` implementation files belong here
- private implementation headers may live here or in a private internal subfolder
- downstream code should not include files from `src/`

### `demo/`

This folder contains example executables or manual validation programs.

Recommended rules:

- demos should consume the public targets the same way an outside user would
- demos should not bypass the target graph
- demo code should not be compiled into the core library target

### `tests/`

This folder contains automated tests.

Recommended rules:

- tests should also consume public targets unless testing internal-only behavior intentionally
- test-only utilities should stay out of production targets

### `docs/`

This folder contains technical notes, architecture descriptions, and onboarding material.

It is the right place for:

- architecture decisions
- build conventions
- extension rules
- AI-agent-facing handoff documents

## CMake Target Design Pattern

## Base Rule

Every meaningful code layer should map to a target.

That usually means:

- one core library target
- zero or more optional extension targets
- zero or more executable targets for demos, tools, or apps

## Namespacing Rule

Use namespaced alias targets to make consumption clean and stable.

Example:

```cmake
add_library(myproject ...)
add_library(MyProject::myproject ALIAS myproject)
```

Benefits:

- downstream usage reads clearly
- the project can later support install/export conventions more naturally
- internal and external consumption look similar

## Header-Only vs Compiled Rule

An agent must inspect the codebase before choosing target type.

### Header-only pattern

Use this only when the library has no compiled implementation sources:

```cmake
add_library(myproject INTERFACE)
add_library(MyProject::myproject ALIAS myproject)

target_include_directories(myproject
    INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
```

### Compiled-library pattern

Use this when `.cpp` files exist:

```cmake
add_library(myproject
    src/file_a.cpp
    src/file_b.cpp
)
add_library(MyProject::myproject ALIAS myproject)

target_include_directories(myproject
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
```

The choice between `STATIC` and `SHARED` should be based on project requirements, not copied mechanically.

## Optional Extension Pattern

Optional functionality should be represented as a separate target.

Example:

```cmake
add_library(myproject_extra INTERFACE)
add_library(MyProject::myproject_extra ALIAS myproject_extra)

target_link_libraries(myproject_extra
    INTERFACE
        MyProject::myproject
        third_party_dep
)
```

This pattern keeps the core target minimal while still exposing richer functionality when needed.

## Executable Pattern

Executables should depend on library targets instead of duplicating usage requirements.

Example:

```cmake
add_executable(main_demo
    demo/main_demo.cpp
)

target_link_libraries(main_demo
    PRIVATE
        MyProject::myproject_extra
)
```

This validates that the target graph is correctly modeled.

## External Dependency Pattern

When using another project from source, prefer making the dependency relationship explicit.

One common local-development pattern is:

```cmake
add_subdirectory(
    "${MYPROJECT_DEP_SOURCE_DIR}"
    "${CMAKE_BINARY_DIR}/external/some_dep"
)
```

This works well when:

- multiple repositories are developed side-by-side
- the dependency is not yet packaged
- local iteration speed matters more than installation polish

If dependencies are installed or package-managed, use the appropriate alternative instead.

The important rule is not the mechanism itself. The important rule is that dependency wiring remains explicit and target-based.

## Recommended Option Pattern

Use options to guard optional targets and dependency trees.

Example:

```cmake
option(MYPROJECT_BUILD_DEMO "Build demo executable." ON)
option(MYPROJECT_ENABLE_EXTRA "Enable optional extra functionality." ON)
```

Then conditionally enable those parts:

```cmake
if(MYPROJECT_ENABLE_EXTRA)
    # define optional target(s)
endif()

if(MYPROJECT_BUILD_DEMO)
    # define demo executable(s)
endif()
```

Add validation when one option depends on another.

Example:

```cmake
if(MYPROJECT_BUILD_DEMO AND NOT MYPROJECT_ENABLE_EXTRA)
    message(FATAL_ERROR "MYPROJECT_BUILD_DEMO requires MYPROJECT_ENABLE_EXTRA to be ON.")
endif()
```

## Generic Target Graph

A clean project often looks like this:

```text
MyProject::core
    publishes:
        - public headers
        - core usage requirements

MyProject::extra
    depends on:
        - MyProject::core
        - optional third-party dependencies

main_demo
    depends on:
        - MyProject::extra

unit_tests
    depends on:
        - MyProject::core
```

The exact target names may differ. The structural relationship should stay clear.

## AI Agent Implementation Rules

An AI coding agent applying this pattern should follow these rules.

### 1. Inspect before restructuring

The agent should determine:

- whether the project is header-only or compiled
- which headers are actually public
- whether optional integrations already exist
- whether demos and tests are currently mixed into production code

The agent should not assume the correct target type or folder layout without checking the codebase.

### 2. Preserve behavior while changing structure

Structural cleanup must not silently remove working features.

The agent should move code carefully so that:

- include paths remain correct
- target dependencies remain complete
- demos and tests still build when enabled

### 3. Avoid fake modularity

Do not create extra targets or folders unless they represent real architectural boundaries.

A simple project should stay simple.

### 4. Keep demos and tests consumer-like

Demos and tests should prove that the library can be consumed through its published targets.

They should not rely on internal path hacks unless explicitly testing internals.

### 5. Keep local configuration isolated

Machine-local paths belong in dedicated configuration files, not mixed throughout target definitions.

### 6. Prefer stable public names

If the project is intended for reuse, use predictable alias target names and public include layout so downstream usage remains stable.

## Generic Restructuring Template

An AI agent can use this as its default migration target:

```text
project-root/
|-- CMakeLists.txt
|-- README.md
|-- cmake/
|   `-- local_paths.cmake              # only if local dependency paths are needed
|-- include/
|   `-- <public headers>
|-- src/
|   `-- <implementation sources>       # only if not header-only
|-- demo/
|   `-- main_demo.cpp                  # optional
|-- tests/
|   `-- <test files>                   # optional
|-- docs/
|   `-- architecture.md                # optional but recommended
`-- build/
```

## Generic CMake Skeleton

```cmake
cmake_minimum_required(VERSION 3.16)
project(MyProject LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/local_paths.cmake" OPTIONAL)

option(MYPROJECT_BUILD_DEMO "Build demo executable." ON)
option(MYPROJECT_BUILD_TESTS "Build tests." OFF)
option(MYPROJECT_ENABLE_EXTRA "Enable optional extra functionality." ON)

add_library(myproject
    src/file_a.cpp
    src/file_b.cpp
)
add_library(MyProject::myproject ALIAS myproject)

target_include_directories(myproject
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

if(MYPROJECT_ENABLE_EXTRA)
    add_library(myproject_extra INTERFACE)
    add_library(MyProject::myproject_extra ALIAS myproject_extra)

    target_link_libraries(myproject_extra
        INTERFACE
            MyProject::myproject
    )
endif()

if(MYPROJECT_BUILD_DEMO)
    add_executable(main_demo demo/main_demo.cpp)
    target_link_libraries(main_demo
        PRIVATE
            MyProject::myproject
    )
endif()
```

This skeleton is illustrative. An agent should adapt it to the actual codebase rather than forcing it literally.

## Prompt Template for an AI Coding Agent

```text
Restructure this C++ repository to follow a clean target-based CMake architecture.

Requirements:
- Keep the repo root shallow and predictable.
- Put public headers in include/.
- Put implementation in src/ only if compiled sources exist.
- Use explicit CMake library/executable targets with namespaced ALIAS targets.
- Separate core functionality from optional integrations or extensions.
- Make demos and tests depend on public targets rather than raw include paths.
- Put machine-local dependency paths in cmake/local_paths.cmake when needed.
- Use CMake options for demos, tests, and optional features.
- Preserve existing behavior while improving structure.
- Do not choose INTERFACE unless the library is actually header-only.

Deliverables:
- updated folder layout
- updated root CMakeLists.txt
- optional cmake helper files
- demo/test wiring through the public target graph
- short architecture documentation
```

## Non-Goals

This pattern does not require:

- a specific project name
- a specific dependency manager
- a specific testing framework
- a specific packaging approach
- a specific target naming scheme beyond consistent namespacing

It defines structure and layering discipline, not one mandatory implementation style.

## One-Sentence Rule

Build the repository around a clear target graph where public API lives in `include/`, implementation lives in `src/` when needed, optional features are layered in separate targets, and machine-local configuration stays isolated from the main build logic.
