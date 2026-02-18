# OpenUxAS Testing Guide

This directory contains the test infrastructure for OpenUxAS.

## Test Organization

```
tests/
├── cpp/               # C++ service tests (Python-based)
│   ├── run-tests      # Test runner script
│   ├── tests/
│   │   └── arv/       # AutomationRequestValidator test cases
│   └── README.md      # Detailed C++ test documentation
└── proof/             # SPARK proof replay
    ├── run-proofs     # Proof runner script
    └── run-proofs.py  # Python implementation
```

## Test Types

### 1. C++ Integration Tests (tests/cpp/)

**Purpose**: Exercise C++ and Ada services in isolation using synthetic LMCP messages
**Technology**: Python + pylmcp module
**Target**: AutomationRequestValidator (ARV) service (goal: 100% coverage)

#### Running C++ Tests

```bash
# From repository root
cd tests/cpp
./run-tests

# Or run specific test
./run-tests tests/arv/correct_angled_area_search_task_request

# Run with locally-built uxas binary
PATH="${OPENUXAS_ROOT}/obj/cpp:$PATH" ./run-tests

# Run with specific qualifier
./run-tests --qualifier=scenario=debug
```

#### Test Structure

Each test is a directory under `tests/cpp/tests/arv/` containing:
- `test.py`: Python script that orchestrates the test

**Example test pattern** (`test.py`):
```python
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import AutomationRequestValidator, UxASConfig

# Configure UxAS to run only ARV service
bridge_cfg = UxASConfig()
bridge_cfg += AutomationRequestValidator()

with Server(bridge_cfg=bridge_cfg) as server:
    # Send setup messages (configs, states, zones, tasks)
    server.send_msg(Object(class_name='AirVehicleConfiguration', ID=400, ...))
    server.send_msg(Object(class_name='KeepInZone', ZoneID=1, ...))

    # Send test message
    server.send_msg(Object(class_name='ImpactAutomationRequest', RequestID=50, ...))

    # Wait for expected response
    msg = server.wait_for_msg(
        descriptor='uxas.messages.task.UniqueAutomationRequest',
        timeout=10.0)

    # Assert expected behavior
    assert msg.descriptor == "uxas.messages.task.UniqueAutomationRequest"
    assert msg.obj['OriginalRequest'] == expected_value
    print("OK")
```

#### Key Testing Components

- **pylmcp.Server**: Manages UxAS instance and ZeroMQ communication
- **pylmcp.Object**: Creates LMCP messages with randomized or specified fields
- **UxASConfig**: Specifies which services to run (single service for isolation)
- **wait_for_msg()**: Blocks until expected message arrives or timeout

#### Adding New C++ Tests

1. Create test directory: `tests/cpp/tests/arv/my_test_case/`
2. Write `test.py` following the pattern above
3. Run to verify: `cd tests/cpp && ./run-tests tests/arv/my_test_case`

Key considerations:
- **Test isolation**: Only enable the service under test
- **Determinism**: Use fixed IDs and values where possible
- **Coverage**: Design tests to exercise different code paths
- **Timeout**: Default 10 seconds; increase for complex scenarios

### 2. SPARK Proof Replay (tests/proof/)

**Purpose**: Verify Ada/SPARK code correctness via formal proof
**Technology**: GNATprove (SPARK formal verification toolchain)
**Target**: All SPARK packages in `src/ada/src/`

#### Running SPARK Proofs

```bash
# From repository root
cd tests/proof
./run-proofs

# Or via MCP server (when in src/ada/)
# Use gnatprove MCP server with project file afrl_ada_dev.gpr
```

#### Proof Artifacts

- `src/ada/proof/`: Session directories for each project
- `src/ada/gnatprove/`: GNATprove session data (analysis results)
- Baseline results stored in git for regression checking

#### Proof Expectations

All SPARK code should:
- **Pass at level 0** (default proof level)
- Have no `medium` or `high` severity unproved checks
- Warnings are acceptable but should be reviewed

#### Proof Results

Output indicates:
- **Proved**: All verification conditions discharged
- **Not proved**: Some checks could not be verified
- **Warning**: Potential issues (not blocking)

Failed proofs indicate:
- Missing preconditions or postconditions
- Missing loop invariants
- Potential runtime errors (overflow, array bounds, etc.)

#### Re-running After Changes

After modifying SPARK code:
```bash
cd tests/proof
./run-proofs

# Or clean and re-prove
cd src/ada
rm -rf gnatprove proof
cd ../../tests/proof
./run-proofs
```

### 3. Coverage Analysis (tests/cpp/ with gcov)

**Purpose**: Measure statement coverage of ARV service
**Technology**: gcov + gcc instrumentation
**Goal**: 100% coverage of AutomationRequestValidator

#### Running with Coverage

The recommended approach is to use the `anod` build system with the gcov qualifier:

```bash
# From repository root
# Build UxAS with gcov instrumentation
./anod build uxas --qualifier=scenario=gcov

# Run tests with gcov instrumentation
tests/cpp/run-tests --qualifier=scenario=gcov
```

**Note**: If you encounter issues with patch failures (for `serial` or `pugixml`), you may need to reset the sandbox first:

```bash
./anod reset
```

#### Alternative: Building Ada with gcov directly

For Ada-specific coverage, you can also build using gprbuild directly:

```bash
# Build Ada with gcov instrumentation
cd src/ada
gprbuild -P afrl_ada_dev.gpr -XAPP_MODE=gcov

# Run tests
cd ../../tests/cpp
./run-tests

# Generate coverage reports
cd ../../src/ada
gcov objs/gcov/*.gcda
```

#### Coverage Metrics

Coverage is measured for:
- Statement coverage (every line executed)
- Branch coverage (every decision path taken)

The gcov-instrumented build generates `.gcda` files during test execution, which can be analyzed using gcov to produce coverage reports showing which lines were executed and how many times.

## Test Infrastructure Details

### Python Test Harness (pylmcp)

The `pylmcp` module provides:
- **LMCP serialization/deserialization**: Convert between Python objects and LMCP wire format
- **UxAS process management**: Launch/terminate UxAS instances
- **ZeroMQ bridge**: Send/receive messages via TCP sockets
- **Message factories**: Create LMCP objects with randomized or explicit fields

Configuration:
- Uses ZeroMQ `PUB/SUB` sockets for message passing
- UxAS instances run with minimal configuration (single service)
- Timeout mechanism prevents hanging on failures

### Test Execution Flow

1. **Setup**: `Server()` launches UxAS with specified configuration
2. **Configure**: Send configuration messages (entities, zones, regions)
3. **Act**: Send test stimulus message
4. **Assert**: Wait for expected response and verify fields
5. **Cleanup**: Server context manager terminates UxAS

### Environment Setup

Tests use `infrastructure/paths.sh` to locate:
- `UXAS_BIN`: Compiled C++ executable
- `UXAS_ADA_BIN`: Compiled Ada executable
- Python virtual environment for pylmcp

The `run-tests` script automatically:
- Activates Python venv
- Sets PATH to find locally-built binaries
- Invokes `run-tests.py` with arguments

## Testing Best Practices

### For C++ Tests

1. **One behavior per test**: Each test should verify a single scenario
2. **Descriptive names**: `correct_angled_area_search_task_request` over `test_1`
3. **Comprehensive setup**: Send all prerequisite messages (configs, states, zones)
4. **Explicit assertions**: Check specific fields, not just message type
5. **Clear output**: Print "OK" on success, detailed diffs on failure

### For SPARK Proofs

1. **Annotate loops**: Always include loop invariants
2. **Contract completeness**: Add Pre/Post conditions to aid provers
3. **Bounded data**: Use formal containers with explicit capacity
4. **Incremental proving**: Prove subprograms bottom-up
5. **Justify assumptions**: Document why proofs at higher levels may be needed

### Coverage Goals

- **ARV service**: Target 100% statement coverage
- **Other services**: Best-effort coverage
- **SPARK packages**: 100% proof coverage (all VCs discharged)

## Continuous Integration

GitHub Actions workflows (`.github/workflows/`):
- **uxas-cpp.yaml**: Builds C++ UxAS and runs C++ tests
- **uxas-ada.yaml**: Builds Ada UxAS and replays SPARK proofs

Both run on:
- Every push to `develop`
- Every pull request
- Ubuntu 22.04 runners

## Debugging Failed Tests

### C++ Test Failures

1. **Enable verbose output**: Check test script for debug options
2. **Run test individually**: `./run-tests tests/arv/failing_test`
3. **Check UxAS logs**: Server output may show service errors
4. **Verify message sequence**: Print sent/received messages
5. **Check timeouts**: Increase if test is timing out

### SPARK Proof Failures

1. **Read GNATprove output**: Identifies unproved VCs
2. **Check proof level**: Ensure running at appropriate level
3. **Review contracts**: Missing Pre/Post conditions?
4. **Inspect loop invariants**: Are they strong enough?
5. **Consult MCP resources**: `file://proof.md`, `file://loops.md`

### Common Issues

- **Port conflicts**: Another UxAS instance running? (`killall uxas`)
- **Missing dependencies**: Re-run `./anod build uxas` and `./anod build uxas-ada`
- **Stale build artifacts**: Clean with `make clean` or `rm -rf obj/`
- **Python environment**: Ensure venv activated (run-tests does this automatically)

## Future Work

Potential test expansions:
- Additional service tests (RoutePlanner, WaypointManager, etc.)
- Multi-service integration tests
- Performance benchmarks
- Fuzz testing for message parsing
- Automated regression testing for proofs
