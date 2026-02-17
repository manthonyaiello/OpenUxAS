# OpenUxAS Project Overview

OpenUxAS is an AFRL research platform for cooperative autonomy, consisting of modular services that communicate via ZeroMQ message-passing using the LMCP (Lightweight Message Control Protocol) format.

## Architecture

**Hybrid C++/Ada System**: OpenUxAS consists of two cooperating executables:
- **C++ OpenUxAS** (`src/cpp/`): Complete implementation with ~30 services for autonomous vehicle operations
- **Ada OpenUxAS** (`src/ada/`): Selected subset reimplemented in SPARK/Ada for high-assurance core functionality

The C++ executable runs without Ada-implemented services, then the Ada executable launches and both cooperate to provide the full service set.

## Key Directories

### Source Code
- `src/cpp/`: C++ implementation
  - `Services/`: ~25 services (route planning, automation request validation, sensor management, etc.)
  - `Communications/`: ZeroMQ-based messaging infrastructure
  - `Tasks/`, `Plans/`, `Utilities/`: Supporting functionality
  - `UxAS_Main.cpp`: C++ entry point

- `src/ada/`: SPARK/Ada implementation
  - `afrl_ada_dev.gpr`: Main Ada project file
  - `sparklib.gpr`: SPARK library project
  - `src/services/`: Ada services (ARV, ATBB, route_aggregator, waypoint_manager, etc.)
  - `src/comms/`, `src/common/`, `src/utils/`: Supporting packages
  - Main: `uxas_ada.adb`

### Build System
- `infrastructure/`: AdaCore e3-core based build system with sandboxes
  - `specs/`: Build specifications for components
  - `sbx/`: Sandboxes for isolated builds
  - `software/`: Installed build tools (Alire, GNATprove)
  - `paths.sh`: Environment setup script
- `Makefile`: Incremental C++ builds (use after initial `anod` build)
- `anod`: Main build orchestration script

### Testing
- `tests/cpp/`: Python-based C++ test suite (uses pylmcp for synthetic messages)
  - Focus: AutomationRequestValidator service coverage
  - Run: `./run-tests` from `tests/cpp/`
- `tests/proof/`: SPARK proof replay infrastructure
  - Run: `./run-proofs` from `tests/proof/`

### Other
- `examples/`: Example configurations and scenarios (XML-based)
- `run-example`: Script to launch examples with OpenAMASE simulation
- `resources/`: Documentation build scripts and data
- `doc/`: Doxygen C++ reference and LaTeX user manual sources
- `mdms/`: LMCP message definition files

## Build Workflow

1. **Initial setup**: `./anod build uxas` (fetches dependencies, builds everything)
2. **Incremental C++ builds**: `make -j all`
3. **Run examples**: `./run-example 02_Example_WaterwaySearch`
4. **Run C++ tests**: `cd tests/cpp && ./run-tests`
5. **Replay SPARK proofs**: `cd tests/proof && ./run-proofs`

## Key Technologies
- **C++**: Core implementation language
- **Ada/SPARK**: High-assurance reimplementation of critical services
- **ZeroMQ**: Message bus for service communication
- **LMCP**: Lightweight message protocol (XML-defined, auto-generated classes)
- **GPR/Alire**: Ada project management and dependencies
- **e3-core**: Python-based build orchestration
- **GNATprove**: SPARK formal verification

## Development Notes

### C++ Development
- Source changes in `src/cpp/`
- Rebuild with `make -j all`
- VS Code configured with IntelliSense paths

### Ada/SPARK Development
- Source changes in `src/ada/src/`
- Project files: `afrl_ada_dev.gpr` (main), `sparklib.gpr` (library)
- Build modes: debug, release, gcov (set via `APP_MODE` environment variable)
- SPARK proofs in `src/ada/proof/` and `src/ada/gnatprove/`
- Test with proof replay: `tests/proof/run-proofs`
- **New services**: Use template at `src/ada/src/services/template/` (see its README.md)

### Ada Services
Current Ada services include:
- **ARV** (Automation Request Validator): Validates automation requests
- **ATBB** (Assignment Tree Branch & Bound): Task assignment optimization
- **Route Aggregator**: Combines routing information
- **Waypoint Manager**: Manages waypoint execution

### Important Patterns
- All services communicate via LMCP messages over ZeroMQ
- Services subscribe to message types and respond to queries
- Configuration uses XML files (see `examples/`)
- C++ and Ada executables run concurrently, sharing the message bus

### Modified Files Context
Current git status shows modifications to:
- `.vscode/settings.json`: IDE configuration
- `src/ada/afrl_ada_dev.gpr`: Ada build configuration
- `src/ada/sparklib.gpr`: SPARK library configuration

## Documentation
- README.md: Quick start and usage guide
- `doc/reference/`: LaTeX user manual
- `doc/doxygen/`: C++ API reference (generated)
- Build docs: `resources/build_documentation.sh`

## Documentation Style
- No emojis in documentation files
- Use clear, professional technical writing
- Code examples should be practical and concise
- Link to existing detailed resources rather than duplicating content
