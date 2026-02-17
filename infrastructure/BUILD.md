# OpenUxAS Build Infrastructure Guide

This directory contains the AdaCore e3-core based build system for OpenUxAS. The build system provides hermetic, reproducible builds through sandboxes and separates build phases (source packaging, build, test, install).

## Overview

**Build Orchestrator**: `anod` (wrapper script at repository root)
**Build Technology**: AdaCore e3-core (Python-based build framework)
**Build Isolation**: Sandboxes in `infrastructure/sbx/`
**Dependency Management**: Alire + custom anod specs

## Directory Structure

```
infrastructure/
├── bootstrap          # Initial e3-core setup
├── install            # Installation scripts
├── install-libexec/   # Helper scripts for installation
├── paths.sh           # Global path definitions and helper functions
├── run_example.py     # Example runner implementation
├── sbx/               # Build sandboxes
│   └── x86_64-linux/  # Platform-specific builds
│       ├── boost/
│       ├── zeromq/
│       ├── xmlada/
│       └── ...        # All dependencies
├── software/          # Installed build tools
│   └── alr/           # Alire package manager
│       ├── gnatprove/ # SPARK verification tool
│       └── ...
├── specs/             # Anod build specifications
│   ├── uxas.anod
│   ├── uxas-ada.anod
│   ├── uxas-lmcp.anod
│   ├── amase.anod
│   ├── zeromq.anod
│   └── ...
└── uxas/              # UxAS-specific build artifacts
```

## Key Concepts

### Sandboxes (sbx/)

Sandboxes provide **isolated build environments** for each component:
- Source code extracted in `src/`
- Build outputs in `build/`
- Installation artifacts in `install/`
- Separate sandbox per platform (e.g., `x86_64-linux`)

**Benefits**:
- Reproducible builds across machines
- No interference from system libraries
- Clean dependency tracking

### Anod Specifications (specs/)

Each `.anod` file defines a component's build recipe:
- **Dependencies**: What other components are required
- **Sources**: Where to get source code
- **Build steps**: How to compile
- **Install steps**: Where to place artifacts

**Example**: `specs/uxas.anod` (C++ UxAS)
```python
class Uxas(spec('common')):
    @property
    def build_deps(self):
        return [
            Anod.Dependency('zeromq'),
            Anod.Dependency('boost'),
            Anod.Dependency('uxas-lmcp', qualifier='lang=cpp'),
            ...
        ]
```

### Qualifiers

Qualifiers customize builds without separate spec files:
- `scenario=debug`: Debug build vs. release
- `lang=cpp`: C++ vs. Ada language binding
- Custom qualifiers per component

## Primary Commands

### Build Commands

```bash
# Build C++ UxAS and all dependencies
./anod build uxas

# Build Ada UxAS and all dependencies
./anod build uxas-ada

# Build OpenAMASE (simulation environment)
./anod build amase

# Build with debug qualifier
./anod build uxas --qualifier=scenario=debug

# Build LMCP code generation
./anod build lmcpgen

# Build LMCP generated messages (C++)
./anod build uxas-lmcp --qualifier=lang=cpp

# Build LMCP generated messages (Ada)
./anod build uxas-lmcp --qualifier=lang=ada
```

### Environment Commands

```bash
# Print build environment variables
./anod printenv uxas

# Set environment in current shell (requires sourcing anod first)
source ./anod
anod setenv uxas

# Print build-specific environment (for Makefile, etc.)
./anod printenv uxas --build-env

# Inline format (for eval)
eval "$( ./anod printenv uxas --build-env --inline )"
```

### Development Commands

```bash
# Setup LmcpGen for local development
./anod devel-setup lmcp

# Setup OpenAMASE for local development
./anod devel-setup amase

# After devel-setup, anod uses the cloned repo in develop/ directory
```

### Utility Commands

```bash
# Clean all build artifacts (WARNING: destructive)
./anod reset

# Show dependency graph
./anod show-dep uxas

# Verbose output (show all commands)
./anod -vv build uxas
```

## Build Workflow

### Initial Build (from clean checkout)

1. **Bootstrap**: First run of `./anod` auto-installs e3-core Python environment
2. **Fetch dependencies**: `anod build uxas` downloads and builds all dependencies
3. **Build UxAS**: Compiles C++ UxAS using sandboxed dependencies
4. **Install**: Places binary in `sbx/x86_64-linux/uxas/install/bin/uxas`

**Timeline**: 20-40 minutes (depends on machine, downloads)

### Incremental Development

After initial anod build, use **Makefile** for faster C++ rebuilds:

```bash
# Incremental C++ build (seconds, not minutes)
make -j all

# The Makefile knows how to find sandboxed dependencies
# Builds to obj/cpp/uxas (not sandbox location)
```

For Ada changes:
```bash
cd src/ada
gprbuild -P afrl_ada_dev.gpr -XAPP_MODE=debug
```

### Clean Builds

```bash
# Clean C++ incremental builds
make clean

# Clean Ada builds
rm -rf src/ada/objs

# Nuclear option: clean all sandboxes (requires full rebuild)
./anod reset
```

## Dependency Management

### System vs. Sandboxed Dependencies

OpenUxAS builds dependencies from source in sandboxes to ensure:
- Consistent versions across environments
- Control over build flags
- No system library conflicts

**Dependencies include**:
- **boost**: C++ libraries
- **zeromq**: Message bus
- **pugixml**: XML parsing
- **sqlite**: Database
- **xmlada**: Ada XML parsing
- **zeromqada**: Ada ZeroMQ bindings
- And ~10 more

### Alire (Ada Package Manager)

Ada dependencies use Alire (`infrastructure/software/alr/`):
- **gnatprove**: SPARK verification
- **gprbuild**: Ada build tool
- Installed on-demand by anod

Alire stores toolchains in:
```
infrastructure/software/alr/settings/cache/toolchains/
```

### LmcpGen (LMCP Code Generation)

LMCP message classes are auto-generated:
1. XML message definitions in `mdms/` (or external LmcpGen repo)
2. `./anod build lmcpgen` builds Java-based code generator
3. `./anod build uxas-lmcp` generates C++/Ada bindings
4. Generated code linked into UxAS executables

## Build Modes and Scenarios

### C++ Scenarios
- `release` (default): Optimized, production build
- `debug`: Unoptimized, with debug symbols

### Ada Build Modes
Set via GPR variable `APP_MODE`:
- `release`: Optimized (`-O2 -gnatp`)
- `debug`: Debug symbols, assertions (`-O0 -g -gnata`)
- `gcov`: Coverage instrumentation (`-fprofile-arcs -ftest-coverage`)

## Environment Variables

Key variables set by anod (see `paths.sh`):

```bash
OPENUXAS_ROOT          # Repository root
SBX_DIR                # infrastructure/sbx
UXAS_BIN               # obj/cpp/uxas (Makefile output)
UXAS_ADA_BIN           # src/ada/uxas-ada
UXAS_BUILD_DIR         # Sandbox build directory
UXAS_SOURCE_DIR        # Sandbox source directory
UXAS_INSTALL_DIR       # Sandbox install directory
```

## Troubleshooting

### Build Failures

**Symptom**: `./anod build uxas` fails

**Debug steps**:
1. Run with verbose output: `./anod -vv build uxas`
2. Check specific command that failed
3. Look for missing system dependencies (git, curl, compiler)
4. Try clean rebuild: `./anod reset && ./anod build uxas`

**Common issues**:
- Network timeouts during dependency download
- Insufficient disk space (builds require ~5GB)
- Missing system packages (Python 3, build-essential)

### Dependency Issues

**Symptom**: "Cannot find library" at link time

**Solutions**:
- Ensure `anod build uxas` completed successfully
- Check that Makefile uses anod environment:
  ```bash
  grep ANODENV Makefile  # Should see ANODENV variable
  ```
- Manually set environment:
  ```bash
  eval "$( ./anod printenv uxas --build-env --inline )"
  make -j all
  ```

### Stale Sandboxes

**Symptom**: Changes to dependencies not reflected

**Solution**:
```bash
# Remove specific sandbox
rm -rf infrastructure/sbx/x86_64-linux/zeromq

# Rebuild that component
./anod build zeromq

# Then rebuild dependent components
./anod build uxas
```

### Python Environment Issues

**Symptom**: "No module named 'e3'" or venv errors

**Solution**:
```bash
# Remove venv and re-bootstrap
rm -rf .vpython
./anod build uxas  # Will recreate venv
```

### Ada Toolchain Issues

**Symptom**: "gnatprove not found" or "gprbuild not found"

**Solution**:
Anod installs GNAT via Alire on-demand. Ensure it's on PATH:
```bash
eval "$( ./anod printenv uxas-ada --build-env )"
which gnatprove  # Should show path in infrastructure/software/alr/
```

## Advanced Usage

### Building with Local Modifications

To develop on dependencies (e.g., LmcpGen):
```bash
./anod devel-setup lmcp
cd develop/lmcpgen
# Make changes
cd ../..
./anod build uxas  # Uses local lmcpgen
```

Anod detects `develop/<component>` and uses it instead of fetching.

### Custom Build Flags

For C++ development, modify `Makefile` directly (not anod specs):
```make
CXXFLAGS += -DMY_CUSTOM_FLAG
```

For Ada, modify GPR files or override on command line:
```bash
gprbuild -P afrl_ada_dev.gpr -XAPP_MODE=debug -cargs -gnatG
```

### Parallel Builds

Anod builds dependencies in parallel by default. Control with:
```bash
./anod build uxas --jobs=4
```

Makefile also supports parallel builds:
```bash
make -j8 all
```

## Integration with IDEs

### VS Code
- IntelliSense paths configured in `.vscode/c_cpp_properties.json`
- Build task: Calls `make all`
- Paths include sandboxed dependencies

### Command-line Development

Source `anod` to get shell function:
```bash
source ./anod
anod printenv uxas  # Now works as shell function
```

Useful for setting env without `eval`:
```bash
export PATH=$(./anod printenv uxas | grep '^export PATH' | cut -d'=' -f2-)
```

## Relationship to Makefile

- **Anod**: Full dependency builds, sandboxes, first-time setup
- **Makefile**: Incremental C++ builds, faster iteration
- **Workflow**: Run `anod build uxas` once, then use `make` for development

The Makefile imports anod environment at the top:
```make
ANODENV:=$(shell NO_INSTALL_VENV=1 $(ANOD_BIN) printenv uxas --build-env --inline)
CXX=$(ANODENV)g++
```

## CI/CD Integration

GitHub Actions workflows use anod for builds:
```yaml
- name: Build UxAS
  run: ./anod build uxas

- name: Run tests
  run: cd tests/cpp && ./run-tests
```

All CI builds start from clean sandboxes (no caching currently).

## Further Reading

- **e3-core documentation**: https://github.com/AdaCore/e3-core
- **Anod specifications**: See files in `specs/*.anod`
- **paths.sh**: Read for all environment variable definitions
- **Makefile**: See for C++ build details
