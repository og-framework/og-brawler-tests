<!-- SPDX-License-Identifier: BUSL-1.1 -->
# og-brawler-tests

Catch2 test source for [og-brawler](https://github.com/og-framework/og-brawler).

This repo is a **pure source distribution** — it does not have a root `CMakeLists.txt` and is not directly buildable on its own. Consumers compose it via their own build systems.

## Position in the og-framework graph

```
og-simulation + og-brawler  (submoduled by consumers alongside these tests)
og-simulation-tests         (sibling test repo; also submoduled by og-tests-cmake-runner)
og-brawler-tests  (this repo — pure Catch2 test source)
    ↓ consumed by
og-tests-cmake-runner  — CMake assembly; runs ctest for both test suites
og-brawler-unreal      — UE project; compiles as an LLT target
```

## Related repos

| Repo | Role |
|---|---|
| [og-brawler](https://github.com/og-framework/og-brawler) | The library under test |
| [og-simulation](https://github.com/og-framework/og-simulation) | Transitive dependency (og-brawler depends on it) |
| [og-simulation-tests](https://github.com/og-framework/og-simulation-tests) | Sibling test repo; og-tests-cmake-runner builds both |
| [og-tests-cmake-runner](https://github.com/og-framework/og-tests-cmake-runner) | CMake harness that builds and runs these tests |
| [og-brawler-unreal](https://github.com/og-framework/og-brawler-unreal) | UE project; `Source/OGBrawlerTests/` LLT target submodules this repo |

## Quickstart

To build and run these tests, clone [og-tests-cmake-runner](https://github.com/og-framework/og-tests-cmake-runner) — it submodules this repo alongside og-brawler, og-simulation, og-simulation-tests, and Catch2 and assembles the full CMake build:

```bash
git clone --recurse-submodules https://github.com/og-framework/og-tests-cmake-runner
cd og-tests-cmake-runner
cmake -S . -B build -G "Visual Studio 17 2022"
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Expected: **40 assertions in 20 test cases** from `og_brawler_tests.exe` (plus 218/11 from `og_simulation_tests.exe`).

## Layout

```
Source/OGBrawlerTests/   Catch2 test source (.cpp files + CMakeLists.txt)
```

The `CMakeLists.txt` expects the parent build to provide `og_brawler`, `og_simulation`, `Catch2`, and `glm::glm-header-only` targets — og-tests-cmake-runner supplies all of them.

## Canonical workflow

See [`og-brawler-unreal/docs/cross-repo-dev-loop.md`](https://github.com/og-framework/og-brawler-unreal/blob/main/docs/cross-repo-dev-loop.md) for the multi-repo development workflow.

## License

**[Business Source License 1.1](LICENSE)** — converts to **[MPL-2.0](LICENSES/MPL-2.0.txt)** on the Change Date printed in `LICENSE` (currently `2030-06-01`).

**What this means in practice:**

| Use case | Allowed? |
|---|---|
| Non-commercial use (personal, educational, research, hobby, open-source) | ✅ Yes |
| Commercial use in any product that is *not* a multiplayer brawler | ✅ Yes |
| Use in a software product or service whose primary gameplay is multiplayer character-vs-character melee combat (a "Competing Product") | ⛔ Please contact the maintainer to discuss |
| Modify and contribute back via PR | ✅ Yes (via [CLA](https://github.com/og-framework/og-tools/blob/main/Public/license-templates/CLA-process.md)) |

After the Change Date, the codebase converts automatically to MPL-2.0 and these restrictions lift.

**Unsure if your use is permitted? Have an interesting idea?**
Reach out to [grahnen92@gmail.com](mailto:grahnen92@gmail.com) — the Licensor welcomes such conversations and is open to case-by-case exceptions.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for the contribution decision tree and CLA signing flow.

