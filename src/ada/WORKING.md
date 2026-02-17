# Ada/SPARK Development Guide

This directory contains the Ada/SPARK implementation of OpenUxAS services - a subset of core functionality reimplemented for high-assurance applications.

**Creating a new service?** Start with the comprehensive template and guide at:
[src/services/template/README.md](src/services/template/README.md)

## Project Structure

```
src/ada/
├── afrl_ada_dev.gpr     # Main project file (builds uxas_ada executable)
├── sparklib.gpr         # SPARK library project (for Ada services)
├── src/
│   ├── services/        # Service implementations
│   │   ├── arv/         # Automation Request Validator
│   │   ├── atbb/        # Assignment Tree Branch & Bound
│   │   ├── route_aggregator/
│   │   ├── waypoint_manager/
│   │   └── spark/       # SPARK-only service utilities
│   ├── comms/           # Communication infrastructure
│   ├── common/          # Common utilities
│   ├── main/            # Main entry point (uxas_ada.adb)
│   └── utils/           # Helper utilities
├── proof/               # SPARK proof artifacts
├── gnatprove/           # GNATprove session data
└── objs/                # Build outputs (debug/release/gcov)
```

## Build System

### Project Files

- **afrl_ada_dev.gpr**: Main executable project
  - Main: `uxas_ada.adb`
  - Depends on: xmlada, zmq, lmcp_generated_messages, sparklib
  - Build modes: debug, release, gcov (set via `APP_MODE` environment variable)

- **sparklib.gpr**: Library of SPARK services
  - Used by both the main executable and proof infrastructure
  - Contains all service implementations

### Build Commands

```bash
# Via anod (recommended for clean builds)
./anod build uxas-ada
```

Or, during active development where you want to have incremental builds:

```bash
# Set environment from anod
eval "$( ./anod printenv uxas-ada --build-env )"

# Via GPRbuild directly (from src/ada/)
gprbuild -P afrl_ada_dev.gpr -XAPP_MODE=debug
```

(The build environment need only be set once per terminal session.)

### Build Modes

- **debug**: Assertions enabled, no optimization, debug symbols (`-O0 -g -gnata`)
- **release**: Optimized, no runtime checks (`-O2 -gnatn -gnatp`)
- **gcov**: Coverage instrumentation (`-fprofile-arcs -ftest-coverage`)

## Architecture: Dual-Executable Design

The Ada OpenUxAS runs as a **separate process** alongside the C++ OpenUxAS:

1. **C++ UxAS** starts with Ada-implemented services excluded from its configuration
2. **Ada UxAS** (`uxas_ada`) launches and reads the same XML configuration
3. Both executables communicate via **ZeroMQ bridge** (`LmcpObjectNetworkPublishPullBridge`)
4. Services in either executable can subscribe/publish LMCP messages transparently

### Configuration Pattern

In XML config files, Ada services are commented out in the C++ section:
```xml
<!-- AutomationRequestValidatorService is in Ada -->
```

The Ada executable's `uxas_ada.adb` main program:
- Parses the same XML config
- Extracts bridge TCP addresses for ZeroMQ communication
- Instantiates only Ada-implemented services
- Connects to the shared message bus

## Service Implementation Pattern

Each Ada service follows a three-layer architecture:

### 1. Core Service Logic (SPARK)
**File**: `services/<name>/<name>.ads/adb`
**Example**: `automation_request_validator.ads/adb`

- Pure SPARK package with proof obligations
- Contains state, configuration, and processing logic
- Uses formal containers (Functional.Maps, Formal.Doubly_Linked_Lists)
- Entry point: `Handle_Message` or similar processing procedures

```ada
package Automation_Request_Validator with SPARK_Mode is
   type State_Data is record ... end record;
   procedure Handle_Automation_Request(...);
end Automation_Request_Validator;
```

### 2. Mailboxes (SPARK/Ada)
**File**: `services/<name>/<name>_mailboxes.ads/adb`
**Example**: `automation_request_validator_mailboxes.ads/adb`

- Manages inbound/outbound message queues
- Bridges between LMCP messages and service logic
- May use protected types for concurrency (drops out of SPARK)

### 3. Service Interfacing (Full Ada)
**File**: `services/uxas-comms-lmcp_net_client-service-<name>_interfacing.ads/adb`
**Example**: `uxas-comms-lmcp_net_client-service-automation_request_validator_interfacing.ads/adb`

- Implements the `Service_Base` interface
- Registers subscriptions and handles LMCP message dispatch
- Creates service instances and connects to mailboxes
- Full Ada (not SPARK) due to service manager interactions

## Adding a New SPARK Service

**IMPORTANT**: Use the provided template system! A comprehensive service template with detailed instructions is available at:

**[src/services/template/README.md](src/services/template/README.md)**

The template provides:
- Automated skeleton generation via the `instantiate` script
- All six required files with TODO comments marking where to add your code
- Detailed step-by-step instructions for each file
- Examples throughout showing how to implement common patterns
- Guidance on adding LMCP messages, proofs, and CI integration

### Quick Start: Using the Template

```bash
# From src/ada/src/services/
cd /data/OpenUxAS/src/ada/src/services

# Generate skeleton for your service (use Mixed_Case naming)
template/instantiate My_Service

# This creates: src/ada/src/services/my_service/ with:
#   - my_service.ads/adb                              (SPARK core logic)
#   - my_service_mailboxes.ads/adb                    (Message queues - boilerplate)
#   - uxas-comms-lmcp_net_client-service-my_service_interfacing.ads/adb  (Ada interfacing)
```

The generated files contain `__TODO__` and `__Example__` comments guiding you through:
1. Adding required LMCP messages (if needed)
2. Defining configuration and state records
3. Implementing message handlers
4. Subscribing to messages in `Configure`
5. Connecting everything in the main processing loop
6. Adding the service to `uxas_ada.adb`

**Read [src/services/template/README.md](src/services/template/README.md) for complete details.**

### Service Architecture Overview

Each service has three packages (see template README for full explanation):

1. **`<Service_Name>`** (SPARK): Core service logic
   - State and configuration types
   - Message handlers: `Handle_<MessageType>(...)`
   - Pure SPARK, formally verified
   - Uses `SPARK.Containers.Formal.*` for containers

2. **`<Service_Name>_Mailboxes`** (Ada/SPARK): Message passing
   - Protected type for ZeroMQ communication
   - Boilerplate - template generates complete implementation
   - `sendBroadcastMessage` for publishing LMCP messages

3. **`UxAS.Comms.LMCP_Net_Client.Service.<Service_Name>_Interfacing`** (Ada): Service interface
   - Inherits from `Service_Base`
   - `Configure`: Subscribe to messages, read XML config
   - `Process_Received_LMCP_Message`: Dispatch to handlers
   - Ada LMCP message handlers: `Handle_<MessageType>_Msg(...)`

### Example: Exploring Existing Services

Study existing services as references:
- **ARV** (`arv/`): Complex state management, formal containers
- **Route Aggregator** (`route_aggregator/`): Message aggregation pattern
- **Waypoint Manager** (`waypoint_manager/`): Simpler example

### Key SPARK Patterns

When filling in template TODOs:
- Use `SPARK.Containers.Formal.*` for bounded containers (lists, maps, vectors)
- Use `SPARK.Containers.Functional.*` for configuration (immutable)
- Add Pre/Post conditions to guide provers
- Include `pragma Loop_Invariant` in loops over containers
- Avoid access types; use IDs/indices instead
- Keep SPARK packages pure; Ada interfacing handles I/O

## SPARK Proof Workflow

### Running Proofs

```bash
# From repository root
cd tests/proof
./run-proofs

# Or directly via GNATprove MCP server
# See file://getting-started.md from gnatprove MCP server
```

### Proof Artifacts

- `src/ada/proof/`: Session directories for proof results
- `src/ada/gnatprove/`: GNATprove session data
- Proofs are checked in CI (`.github/workflows/uxas-ada.yaml`)

### Common Proof Patterns

1. **Loop invariants**: Required for `for` loops over containers
   ```ada
   for I in 1 .. Last loop
      pragma Loop_Invariant (...);
      -- body
   end loop;
   ```

2. **Subprogram contracts**:
   ```ada
   procedure Process (...) with
     Pre  => Is_Valid(Input),
     Post => Is_Consistent(State);
   ```

3. **Container bounds**: Use formal containers with capacity parameters
   ```ada
   package My_Lists is new SPARK.Containers.Formal.Doubly_Linked_Lists
     (Element_Type => My_Type,
      Max_Length => 1000);
   ```

### Proof Levels

- Default: Level 0 (fast, catches most issues)
- Higher levels: 1-2 for difficult proofs (use sparingly)
- **Never use levels 3-4** without explicit user permission (very slow)

### When Proofs Fail

1. Check for missing preconditions or loop invariants
2. Review GNATprove output for unproved checks
3. Consult MCP resources: `file://proof.md`, `file://loops.md`, `file://overflow.md`
4. Consider adding intermediate assertions to guide prover
5. If necessary, use `pragma Annotate (GNATprove, ...)` for justified assumptions

## Dependencies

### Ada Libraries (via Alire)
- **xmlada**: XML parsing for configuration
- **zeromqada**: ZeroMQ bindings for messaging
- **SPARK containers**: Formal verification-friendly data structures

### Generated Code
- **lmcp_generated_messages**: Auto-generated Ada bindings for LMCP (from LmcpGen)
  - Built by `anod build uxas-lmcp --qualifier=lang=ada`

## Integration with C++ UxAS

### Message Flow
1. LMCP messages arrive on ZeroMQ `subscribe` socket
2. Service interfacing layer deserializes to Ada LMCP objects
3. Mailbox queues pass messages to SPARK service logic
4. Service processes and generates response messages
5. Responses sent via ZeroMQ `publish` socket

### Debugging

```bash
# Enable verbose logging
export UXAS_LOG_LEVEL=DEBUG

# Run both executables with logging
./obj/cpp/uxas --cfgPath ... 2>&1 | tee cpp.log &
./src/ada/uxas-ada --cfgPath ... 2>&1 | tee ada.log
```

### Common Issues

- **Service not instantiating**: Check `pragma Unreferenced` in `uxas_ada.adb`
- **Messages not received**: Verify bridge TCP addresses match in XML config
- **Build failures**: Ensure `anod build uxas-ada` ran successfully (sets up dependencies)

## Code Style

- GNAT style checks enabled: `-gnaty3aehiIklnOprstux`
- GNAT 2022 features available: `-gnat2022`
- Always use `with SPARK_Mode` on SPARK packages
- Follow Ada naming: `My_Package`, `My_Variable`, `my_procedure`

## Testing

Ada services are tested via the C++ test suite (`tests/cpp/tests/arv/`):
- Python tests exercise services with synthetic LMCP messages
- Tests run against both C++ and Ada implementations
- Coverage measured via gcov (use `APP_MODE=gcov`)

See [tests/TESTING.md](../../tests/TESTING.md) for details.
