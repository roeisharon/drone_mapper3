# Assignment 3 — Drone Mapper Simulator

**Submitters:** Roei Sharon (322526864), Tomer Maizus (323020347)

A multithreaded simulator that dynamically loads MissionControl and Algorithm plugins (`.so`)
and runs them over a composition matrix of simulations × missions × drones × lidars, in two
modes: **comparative** (all mission controls × one algorithm) and **competition** (one mission
control × all algorithms).

---

## Layout

| Folder | Contents |
|---|---|
| `Simulator/` | the executable `simulator_322526864_323020347` (own CMakeLists) |
| `Algorithm/` | `Algorithm_322526864_323020347.so` (own CMakeLists) |
| `MissionControl/` | `MissionControl_322526864_323020347.so` (own CMakeLists) |
| `common/` | staff-published interfaces, used as-is, unmodified |
| `UserCommon/` | our shared code (no makefile, per spec); consumers compile its sources directly |

Namespaces are lowercase with ids preserved, per the forum clarification:
`algorithm_322526864_323020347`, `mission_control_322526864_323020347`,
`user_common_322526864_323020347`, and `simulator` for the executable-only code.

## Build

Requires CMake ≥ 3.15, Ninja, a C++20 compiler, and vcpkg (`VCPKG_ROOT` set). Dependencies
(yaml-cpp, TinyNPY, mp-units, gtest) are staff-approved and resolved via the manifest.

All three projects from the root:

```bash
cmake --preset default && cmake --build --preset default
```

Each project also builds standalone (spec: separate build per project):

```bash
cmake -S Algorithm -B build_algo -G Ninja -DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake && cmake --build build_algo
# same for MissionControl/ and Simulator/
```

Artifacts (no `lib` prefix): `Algorithm_322526864_323020347.so`,
`MissionControl_322526864_323020347.so`, `simulator_322526864_323020347`.

## Run

```bash
./simulator_322526864_323020347 -comparative simulation=<composition.yaml> mission_control_folder=<folder> algorithm=<algorithm.so> [num_threads=<n>] [-verbose]
./simulator_322526864_323020347 -competition simulation=<composition.yaml> mission_control=<mission_control.so> algorithms_folder=<folder> [num_threads=<n>] [-verbose]
```

Arguments may appear in any order. Invalid invocations print a usage message listing **all**
unsupported and **all** missing arguments together, and exit non-zero. Results are written to
`comparative_results_<time>` / `competition_<time>` directly under the given folder (9-digit
collision-free time token). Each results folder contains the mode report
(`comparative_report.yaml` / `competitive_report.yaml`), one assignment-2-format score report
**per individual run** (component stem + run id in the filename, per the forum clarification),
all output maps (`.npy`), and error logs.

## Design notes and decisions

- **Registration.** The staff registration headers are used as-is; the registration
  constructors are **defined only in the Simulator** and exported from the executable
  (`ENABLE_EXPORTS`), so plugins carry undefined references resolved at `dlopen` time.
  Registrar singletons bind factories to plugin files by registration-count delta; a delta ≠ 1
  truncates the registrar, closes the handle immediately, and reports the component in `errors:`.
- **`num_threads` reading.** Missing or `1` → everything on the main thread. `n ≥ 2` → the value
  is the number of **worker** threads in addition to main: we spawn `min(n, #tasks)` workers
  (threads that could not be utilized are not opened) and main blocks in `join` — which the spec
  explicitly permits. The total thread count is therefore 1 or ≥ 3, never 2.
- **Lock-free concurrency.** The task matrix is enumerated up front into a flat table; workers
  claim tasks via a single atomic counter; each result slot has exactly one writer; plugins are
  loaded before workers exist; each group's hidden map is loaded once and shared read-only; all
  reports are written after the join. There are no mutexes anywhere in the program, and output
  is **byte-identical for any thread count**.
- **Instances are never cached.** Factories are invoked afresh per run (spec requirement);
  `.so` files are loaded once and `dlclose`d only when no created object may be alive.
- **Report semantics** (per the forum clarifications, with stated assumptions where the forum
  has not yet ruled):
  - `total_score` sums only runs with score ≠ −1 (forum-clarified).
  - `total_steps` likewise excludes failed runs' steps, so both totals describe the same set of
    runs — *assumption, forum question pending*.
  - `errors:` lists only components that could not be loaded or could not be executed at all;
    per-run failures appear as score −1 in that run's own report — never as `errors:` entries
    (forum-clarified).
  - `same_results` groups mission controls whose runs produced identical (score, steps) on
    every run of the canonical run order — *assumption, forum question pending; isolated behind
    a single comparison function*.
- **Never crash.** Each task runs behind a catch-all; plugin failures become error results,
  scores of −1, and log entries. The simulator exits cleanly on every failure mode we could
  construct (unloadable `.so`, registration misbehavior, throwing constructors/factories,
  throwing missions, unwritable output folders).
- **Verbose.** `-verbose` propagates to MissionControl, which writes one `*_verbose.txt` per run
  **iff** the flag is present.

## Error handling

Mandatory rows implemented: a valid algorithm avoids error states by design (row 1); physical
collision with a known obstacle throws inside the movement layer, is classified by
DroneControl, fails the run with score −1, and is logged (row 5). MissionControl-reported
`Error` statuses are logged and the run continues to be scored per the report rules (row 9).
Errors are appended durably per run to per-run logs plus a `main_errors.log` for plugin-level
failures.

## Tests (developer-only, not part of the graded build)

Three gtest suites (308 tests) are included but gated behind an OFF-by-default option, so the
graded build compiles exactly the required targets:

```bash
cmake --preset default -DDRONE_BUILD_TESTS=ON && cmake --build --preset default
./build/default/Algorithm/algorithm_tests
./build/default/MissionControl/mission_control_tests
./build/default/Simulator/simulator_tests
```

Suites are hermetic (runnable from any cwd) and cover the CLI contract, the plugin lifecycle
(fixture `.so`s for every failure mode), report formats (golden files), threading semantics,
and byte-identical thread-count determinism, alongside the algorithm/mission-control logic.
The Integration tests use the course-provided `inputs/` folder (included), e.g.:

```bash
./build/default/Simulator/simulator_322526864_323020347 -comparative simulation=inputs/sim_compose.yaml mission_control_folder=<folder with MissionControl .so files> algorithm=build/default/Algorithm/Algorithm_322526864_323020347.so num_threads=4
```
