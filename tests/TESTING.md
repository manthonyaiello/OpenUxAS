# OpenUxAS Testing Guide

This directory contains the test infrastructure for OpenUxAS.

## Test Organization

```
tests/
├── cpp/               # C++ and Ada service tests (Python-based)
│   ├── run-tests      # Test runner script
│   ├── run-tests.py   # Python test driver
│   ├── pylmcp/        # LMCP serialization and test support library
│   ├── tests/
│   │   ├── arv/                    # AutomationRequestValidator test cases
│   │   │   ├── <test-name>/
│   │   │   │   ├── test.py         # Test script
│   │   │   │   └── b2b.yaml        # (optional) per-test B2B comparison config
│   │   │   └── ...
│   │   └── sensor-manager/         # SensorManagerService test cases
│   │       └── <test-name>/
│   │           └── test.py
│   └── results/       # Test output files (generated, not committed)
└── proof/             # SPARK proof replay
    ├── run-proofs     # Proof runner script
    └── run-proofs.py  # Python implementation
```

## Test Types

### 1. C++ Integration Tests (tests/cpp/)

**Purpose**: Exercise C++ and Ada services in isolation using synthetic LMCP messages
**Technology**: Python + pylmcp module
**Targets**: AutomationRequestValidator (ARV), SensorManagerService

#### Running C++ Tests

```bash
cd tests/cpp

# Run all tests against the C++ implementation (default)
./run-tests

# Run all tests for a specific service
./run-tests arv
./run-tests sensor-manager

# Run a single test by its full UID
./run-tests arv.correct_automation_request

# Run against the Ada implementation
./run-tests arv --impl=ada

# Back-to-back comparison of C++ and Ada (see section below)
./run-tests arv --impl=both

# Run a gcov-instrumented build for coverage reporting
./run-tests arv --qualifier=scenario=gcov \
    --source-dir=/path/to/OpenUxAS \
    --build-dir=/path/to/OpenUxAS/obj/gcov

# Limit parallelism
./run-tests --jobs=1
```

The `--qualifier=scenario=gcov` flag tells the shell wrapper to activate the
anod environment for a gcov-instrumented build before running the tests.
`--source-dir` and `--build-dir` tell the driver where to find source and
object files so it can print a coverage summary after the run.

#### Test Structure

Each test is a directory under `tests/cpp/tests/<service>/` containing:
- `test.py`: Python script that orchestrates the test
- `b2b.yaml` (optional): per-test back-to-back comparison rules (see below)

**Example test pattern** (`test.py`):
```python
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import AutomationRequestValidator, UxASConfig

# Configure UxAS to run only the ARV service
bridge_cfg = UxASConfig()
bridge_cfg += AutomationRequestValidator()

with Server(bridge_cfg=bridge_cfg) as server:
    # Send setup messages (configs, states, zones, tasks)
    server.send_msg(Object(class_name='AirVehicleConfiguration', ID=400,
                           randomize=True))
    server.send_msg(Object(class_name='KeepInZone', ZoneID=1, randomize=True))

    # Send test stimulus
    server.send_msg(Object(class_name='cmasi.AutomationRequest',
                           TaskList=[1000], EntityList=[400],
                           OperatingRegion=3, randomize=True))

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

- **pylmcp.Server**: Manages a UxAS instance and its ZeroMQ communication
- **pylmcp.Object**: Creates LMCP messages with randomized or specified fields
- **UxASConfig**: Specifies which services to run (single service for isolation)
- **wait_for_msg()**: Blocks until an expected message arrives or times out

### 2. Back-to-Back (B2B) Testing

Back-to-back testing runs the C++ and Ada implementations simultaneously with
identical inputs and compares their outputs message-by-message.  One
implementation is the **oracle** (its outputs are authoritative and returned to
the test's assertion logic); the other is the **challenger** (its outputs are
compared against the oracle's).  Mismatches accumulate and are raised as a
`BackToBackMismatchError` when the test exits.

#### Running B2B Tests

```bash
cd tests/cpp

# C++ oracle vs Ada challenger (default)
./run-tests arv --impl=both

# Ada oracle vs C++ challenger
./run-tests arv --impl=both --oracle=ada

# Sanity check: C++ vs C++ (should always pass)
./run-tests arv --impl=both --oracle=cpp --challenger=cpp

# Adjust floating-point tolerance
./run-tests arv --impl=both --tolerance=1e-4

# Suppress a field globally across all tests
./run-tests arv --impl=both --ignore-fields=SourceServiceID
```

When `--impl=both` is used, `run-tests.py` reserves four ZeroMQ ports per test
(two for the oracle, two for the challenger) and sets the `UXAS_ORACLE`,
`UXAS_CHALLENGER`, `UXAS_B2B_TOLERANCE`, and `UXAS_B2B_IGNORED_FIELDS`
environment variables for each test subprocess.  Tests written to use `Server`
transparently upgrade to `BackToBackServer` when `UXAS_IMPL=both` is set.

#### Round-trip Request/Response Handling

The Ada ARV intentionally starts its internal request-ID counter at 10,000 to
avoid conflicts in multi-process deployments; the C++ ARV starts at small
sequential values.  `BackToBackServer` tracks the oracle-to-challenger ID
mapping when a `UniqueAutomationRequest` is observed and remaps the
`ResponseID` in any subsequent `UniqueAutomationResponse` so that the
challenger processes it correctly.

#### Per-test Comparison Rules (b2b.yaml)

When the C++ and Ada implementations differ in ways that are benign by design
(e.g., the internal request-ID counter appearing in an error string), a test
directory may contain a `b2b.yaml` file that customises the comparison for that
test only.

Supported keys:

**`ignore_fields`** — suppress specific field names at any nesting level for
this test, supplementing the global `--ignore-fields` flag:

```yaml
ignore_fields:
  - RequestID
```

**`field_rules`** — apply per-path comparison rules.  Path keys use the same
dotted notation produced by the diff output (e.g. `Info[0].Value`).  `[*]` is
a wildcard that matches any list index.  When a path matches, the rule is
applied instead of the default equality check.

The `normalize` action applies a sequence of `re.sub` substitutions to both
the oracle and challenger values (converted to strings) before comparing them.
Values that are equal after normalization are not reported as a mismatch:

```yaml
field_rules:
  "Info[*].Value":
    normalize:
      - pattern: 'Automation Request ID\[\d+\]'
        replacement: 'Automation Request ID[*]'
```

This example suppresses the request-ID difference that appears in ARV error
strings when the same logical error is reported with different counter values
by C++ and Ada.

#### Known B2B Limitations

Some tests are expected to fail in back-to-back mode due to genuine
implementation differences that are not bugs:

- **`automation_request_any`**: Ada expands an empty `EntityList` to all known
  entities; C++ does not.  This is a semantic difference that has not yet been
  resolved.
- **`timeout_automation_response`**, **`timeout_task_initialization`**: Ada
  does not yet implement timer-callback driven timeouts, so the challenger
  times out waiting for a response that only the C++ oracle produces.

### 3. SPARK Proof Replay (tests/proof/)

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

### 4. Coverage Analysis (tests/cpp/ with gcov)

**Purpose**: Measure statement coverage of C++ services
**Technology**: gcov + gcc instrumentation

#### Running with Coverage

Build with gcov instrumentation using the anod qualifier, then run the tests
with `--source-dir` and `--build-dir` to get a coverage summary:

```bash
# Build with gcov instrumentation via anod
./anod build uxas --qualifier=scenario=gcov

# Run tests and display coverage summary
cd tests/cpp
./run-tests arv --qualifier=scenario=gcov \
    --source-dir="${OPENUXAS_ROOT}" \
    --build-dir="${OPENUXAS_ROOT}/obj/gcov"
```

The `--qualifier=scenario=gcov` flag tells the `run-tests` shell wrapper to
activate the anod environment for the gcov build before invoking the Python
driver.  `--source-dir` and `--build-dir` enable the post-run gcov summary.

**Note**: If you encounter patch failures (for `serial` or `pugixml`), reset
the sandbox first with `./anod reset`.

#### Coverage Metrics

Coverage is measured per statement (every line executed).  The gcov-
instrumented build generates `.gcda` files during test execution, which the
driver collects and processes with `gcov` to produce a per-file summary.

## Test Infrastructure Details

### Python Test Harness (pylmcp)

The `pylmcp` module provides:
- **LMCP serialization/deserialization**: Convert between Python objects and the LMCP wire format
- **UxAS process management**: Launch/terminate UxAS (C++) and uxas-ada (Ada) instances
- **ZeroMQ bridge**: Send/receive messages via TCP sockets
- **Message factories**: Create LMCP objects with randomized or explicit fields
- **BackToBackServer**: Run oracle and challenger implementations in parallel and compare outputs

Configuration:
- Uses ZeroMQ `PUB/SUB` sockets for message passing
- UxAS instances run with minimal configuration (single service)
- Timeout mechanism prevents hanging on failures

### Test Execution Flow

1. **Setup**: `Server()` (or `BackToBackServer()` in B2B mode) launches UxAS with the specified configuration
2. **Configure**: Send configuration messages (entities, zones, regions, tasks)
3. **Act**: Send test stimulus message
4. **Assert**: Wait for expected response and verify fields
5. **Cleanup**: Server context manager terminates UxAS

### Environment Setup

The `run-tests` shell script:
- Activates the Python venv
- Sets `PATH` to include the locally-built C++ binary (if present)
- When `--impl=ada` or `--impl=both`: runs `anod printenv uxas-ada` to set `UXAS_ADA_INSTALL_DIR` and locate `uxas-ada`
- When `--qualifier=...`: runs `anod printenv uxas <qualifier>` to activate the corresponding build environment
- Propagates `UXAS_IMPL`, `UXAS_ORACLE`, `UXAS_CHALLENGER`, `UXAS_B2B_TOLERANCE`, `UXAS_B2B_IGNORED_FIELDS`, and (per-test) `UXAS_B2B_CONFIG` to test subprocesses

## Adding New Tests

### Adding a C++ / Ada Unit Test

1. Choose the service directory: `tests/cpp/tests/arv/` or `tests/cpp/tests/sensor-manager/`
2. Create a test directory with a descriptive name: `tests/cpp/tests/arv/my_test_case/`
3. Write `test.py` following the patterns in existing tests
4. Run to verify: `cd tests/cpp && ./run-tests arv.my_test_case`

Key considerations:
- **Test isolation**: Enable only the service under test via `UxASConfig`
- **Determinism**: Use fixed IDs and values where possible; `randomize=True` fills unspecified fields
- **Coverage**: Design tests to exercise distinct code paths
- **Timeout**: Default 10 seconds; increase for scenarios with multiple round-trips

### Adding a Back-to-Back Test

Tests automatically participate in B2B comparison when run with `--impl=both`.
No changes to `test.py` are required.

If the C++ and Ada implementations differ in a way that is benign by design,
add a `b2b.yaml` alongside `test.py`:

```bash
# Test passes with C++ only
./run-tests arv.my_test_case

# Run B2B and inspect any mismatch output
./run-tests arv.my_test_case --impl=both
cat results/arv.my_test_case.out

# Add b2b.yaml to suppress expected differences, then re-run
./run-tests arv.my_test_case --impl=both
```

See the [Per-test Comparison Rules](#per-test-comparison-rules-b2byaml) section
for the `b2b.yaml` schema.

## Testing Best Practices

### For C++ / Ada Tests

1. **One behavior per test**: Each test should verify a single scenario
2. **Descriptive names**: `correct_angled_area_search_task_request` over `test_1`
3. **Comprehensive setup**: Send all prerequisite messages (configs, states, zones)
4. **Explicit assertions**: Check specific fields, not just message type
5. **Clear output**: Print "OK" on success; assertions produce detailed diffs on failure

### For Back-to-Back Tests

1. **Run B2B early**: Add `--impl=both` from the start to catch divergences immediately
2. **Distinguish design differences from bugs**: A mismatch may be a bug in one implementation or an intentional difference (like request-ID counter starting values)
3. **Scope suppression tightly**: Prefer `field_rules` with `normalize` over broad `ignore_fields` when the meaningful content of a field should still be compared
4. **Document b2b.yaml choices**: The comment at the top of each `b2b.yaml` should explain why the difference is expected

### For SPARK Proofs

1. **Annotate loops**: Always include loop invariants
2. **Contract completeness**: Add Pre/Post conditions to aid provers
3. **Bounded data**: Use formal containers with explicit capacity
4. **Incremental proving**: Prove subprograms bottom-up
5. **Justify assumptions**: Document why proofs at higher levels may be needed

### Coverage Goals

- **ARV service**: 100% statement coverage achieved
- **SensorManagerService**: ~99% statement coverage achieved
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

### C++ / Ada Test Failures

1. **Run the test individually**: `cd tests/cpp && ./run-tests arv.failing_test`
2. **Inspect the output file**: `cat tests/cpp/results/arv.failing_test.out`
3. **Check UxAS logs**: The server output captures service warnings and errors
4. **Verify message sequence**: Trace sent/received messages in the test script
5. **Check timeouts**: Increase if the test is consistently timing out

### B2B Mismatch Failures

The output file reports each differing field with its oracle and challenger
values, using dotted-path notation (e.g. `Info[0].Value`):

```
Mismatch for afrl.cmasi.AutomationResponse:
Info[0].Value: 'Automation Request ID[12] Not Ready ...' vs
               'Automation Request ID[10001] Not Ready ...'
```

Steps to diagnose:
1. Determine whether the difference is a bug or an expected design divergence
2. If it is a bug, identify which implementation is wrong and fix it
3. If it is an expected divergence, add a `b2b.yaml` to suppress it with an appropriate `ignore_fields` entry or `field_rules` normalize rule

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
- **Python environment**: Ensure venv is activated (`run-tests` does this automatically)
- **Ada binary not found**: Run with `--impl=ada` or `--impl=both` after `./anod build uxas-ada`, or set `UXAS_ADA_BIN`

## Future Work

Potential test expansions:
- Additional service tests (RoutePlanner, WaypointManager, etc.)
- Multi-service integration tests
- Performance benchmarks
- Fuzz testing for message parsing
- Automated regression testing for proofs
- B2B coverage: resolve remaining known limitations (`automation_request_any` EntityList, timeout tests)
