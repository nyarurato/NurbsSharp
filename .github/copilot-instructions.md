<!--
This file provides a concise, project-specific orientation for AI coding assistants
working on this repository. Edit carefully and verify commands/paths when changing.
-->

# NurbsSharp — Quick AI Instructions (concise)

Purpose: This repository implements core NURBS (non-uniform rational B-splines) data
structures, evaluators, I/O, topology operators (join/split/degree/knot) and
tessellators. Use the pointers below to get productive quickly.
- Basic:
  - Respond in the user's language
  - Comment style: follow existing conventions in the codebase. Basically use in English.
  - commit message style: concise, imperative mood, English. see .gitmessage file.

- Quick start (PowerShell):
  - Restore and build:
    `dotnet restore`
    `dotnet build -c Debug`
  - Run tests:
    `dotnet test -c Debug`
  - CI test command (used in GitHub Actions):
    `dotnet test --no-restore --configuration Release`

- Key settings / targets:
  - Library outputs include `netstandard2.1` and `net8.0` (see `bin/` folders).
  - Tests target `net8.0` and use **NUnit** (`src/UnitTests/UnitTests.csproj`).
  - Project conventions: `Nullable` is `enable`, `ImplicitUsings` is `disable`.
    Avoid adding implicit global usings.

- Major components (representative files):
  - Core numerics & types: `src/Core/ControlPoint.cs`, `src/Core/KnotVector.cs`,
    `src/Core/VectorDouble.cs`, `src/Core/LinAlg.cs`.
  - Geometry models: `src/Geometry/NurbsCurve.cs`, `src/Geometry/NurbsSurface.cs`,
    `src/Geometry/NurbsVolume.cs`, `src/Geometry/Mesh.cs`.
  - Evaluators: `src/Evaluation/BasicEvaluator.cs`, `src/Evaluation/CurveEvaluator.cs`,
    `src/Evaluation/SurfaceEvaluator.cs`, `src/Evaluation/VolumeEvaluator.cs`.
  - Operators: `src/Operation/DegreeOperator.cs`, `src/Operation/KnotOperator.cs`,
    `src/Operation/JoinOperator.cs`, `src/Operation/SplitOperator.cs`.
  - I/O: `src/IO/OBJExporter.cs`, `src/IO/STLExporter.cs`,
    `src/IO/IGES/Import/ImportedIgesRationalBSplineCurve.cs` (IGES parsing example).
  - Tessellation: `src/Tesselation/CurveTessellator.cs`,
    `src/Tesselation/SurfaceTessellator.cs`.
  - Test resources: `src/UnitTests/IO/resources/test_outputB.igs`.

- Architecture notes / rationale:
  - Numerical evaluation logic is centralized under `src/Evaluation/*` and
    depends heavily on geometry models in `src/Geometry/*` plus numeric helpers
    in `src/Core/*`.
  - Topology-modifying operators (join/split/degree/knot) mutate knot vectors and
    control point arrangements; such changes must be validated by associated
    evaluator tests (see `src/UnitTests/Operation/*`).

- Recommended change workflow:
  1. Identify the single responsibility of the change (e.g. evaluator vs geometry).
  2. Run unit tests: `dotnet test src/UnitTests/UnitTests.csproj -c Debug`.
  3. Add or update NUnit tests under `src/UnitTests/*` when behavior or API changes.

- Project-specific conventions:
  - Avoid breaking public API surface unless necessary—prefer internal refactors.
  - Naming: PascalCase for files and types; keep parameter ordering consistent
    across public APIs.
  - When adding packages, run `dotnet restore` and a local build to validate the
    solution (`NurbsSharp.sln`).
  - If adding a top-level public type, consider including XML documentation
    comments so library XML output remains complete.

- CI / workflow pointers:
  - GitHub Actions workflows live under `.github/workflows/` (`test.yml`,
    `ci.yml`, `document.yml`, `deploy.yml`). `test.yml` runs `dotnet test` on .NET 8.

- Quick lookup tips (symbols to search):
  - Evaluators: search for `CurveEvaluator`, `SurfaceEvaluator`, `BasicEvaluator`.
  - Geometry: search for `NurbsCurve`, `NurbsSurface`, `Mesh`.