# Lab III workspace instructions

- Use the shared `intr5100` Conda environment for course Python work. The course-root `environment.yml` is authoritative; sync its repository copy when publishing.
- The root `student_controller3.py` is the frozen lab submission. Do not retune it without a user request. Preserve the required callback signature and filename.
- Supporting files in `environment/` are supplied reference copies and must remain unchanged. Verify them against `review/supporting_file_manifest.json`.
- Keep a standalone clone functional. Tests must use repository-local supporting files and temporary GUI deployments, with no physical vehicle connections.
- Distinguish synthetic results from physical Lab III data; no physical experiment is currently recorded. Retain failed stress cases and documented limitations.
- General document names use spaces. Keep functional code and data filenames intact. Use `docs/`, `tests/`, `results/`, `review/` and `archive/` consistently.
- Do not commit environments, bytecode, caches, temporary renders, uploaded ZIPs or GUI run-directory byproducts. Preserve scientific evidence and useful historical versions.
