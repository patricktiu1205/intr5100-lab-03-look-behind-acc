# Lab III Look Behind Adaptive Cruise Control

The controlled vehicle is now in the middle: leader (0), student (1), IDM follower (2). Design at least one look-behind controller using both surrounding vehicles and the student state. Two or more controllers are encouraged for comparison.

## Before the lab

Submit `student_controller3.py` to the TA before the session begins. Modify only that file and preserve `student_control(leader_state, student_state, following_state, history=None) -> float`. Supporting experiment, IDM and map files must remain unchanged. Repository-local reference copies are in `environment/`; the original course copies are untouched. The guideline gives no calendar date or separate submission time.

Return a desired speed command, not acceleration. If using acceleration internally, convert it using a timestep in seconds; the configured default is 0.15 s. States supply velocity, x, y and heading. History includes role, elapsed time, position, heading, velocity and longitudinal route coordinates.

Student control is called only when leader and student share one of the three longest selected straights. The IDM follower operates when student and follower share a selected straight. Otherwise the corresponding vehicle receives the current leader speed command. These two activation conditions can differ.

## Unmodified supplied files

- [Controller template](<environment/student_controller3.py>)
- [Lab III GUI](<environment/ta_experiment3_gui.py>)
- [Experiment implementation](<environment/three_vehicle_experiment.py>)
- [Following vehicle IDM](<environment/idm_controller.py>)
- [Guidelines](<docs/Lab 03 Guidelines.pdf>)

The supplied shared template remains unchanged. The implemented candidate is [student_controller3.py](student_controller3.py) in this folder. It uses equilibrium-referenced Leading Cruise Control with front priority and gains selected for leader-to-follower disturbance attenuation. See [Design Notes](<docs/Design Notes.md>) for equations, paper references, scale assumptions and test limitations. Submit only this candidate file; do not submit the tests or change supporting files.

## After the lab

Retain each complete run folder: `trajectory.csv`, `commands.csv`, `leader_events.csv` and `metadata.json`. The final project report should cover controller objectives/design, all three trajectories, quantitative disturbance propagation, spacing and safety, and controller comparisons when applicable. Separate active-control and bypass intervals in analysis.

Use the shared `intr5100` Conda environment. General document names use spaces. Functional controller and recorded-data filenames retain their required names.

## Current submission

Use [student_controller3.py](student_controller3.py). This tested version is frozen for the lab; further algorithm improvements are deferred. [Controller Comparison](<docs/Disturbance Comparison.md>) summarizes the results, and [Report Discussion Notes](<docs/Report Discussion Notes.md>) records the remaining limitations.

## Verification

From this folder, use the shared course environment:

```sh
conda activate intr5100
PYTHONDONTWRITEBYTECODE=1 python -m unittest discover -s tests -v
PYTHONDONTWRITEBYTECODE=1 python tests/check_ta_integration.py
```

The integration check copies unmodified supporting files into an automatically deleted temporary deployment, uses synthetic states and forbids network connections. No physical experiment has been run by these checks.

Existing comparisons are reproducible with `tests/evaluate_refinement.py`, `tests/check_linear_response.py` and `tests/build_comparison.py`. `tests/select_refinement.py` records the development selection procedure; routine use does not require retuning. The deployed controller reads the existing supplied geometry utilities and map from `environment/` locally, or from the same folder as the submission at the TA. It requires no online optimization solver. This checkout is self-contained; activate the shared `intr5100` environment, or recreate it from `environment.yml` on another machine.

For a physical matched front-only test, use a separate copy of the submission with only the module-level instantiation changed to `LookBehindController(rear_weight=0.0)`. Keep front targets, other settings and supporting files unchanged. Only submit the requested controller file.

## Repository layout

| Location | Contents |
| --- | --- |
| `student_controller3.py` | The single file to submit to the TA |
| `tests/` | Logic, telemetry, TA GUI and synthetic comparison checks |
| `docs/` | Guideline, design notes, comparisons and report discussion |
| `environment/` | Byte-identical copies of the supplied supporting files |
| `results/` | Synthetic plots and trace CSVs |
| `review/` | Parameter selection, verification and comparison evidence |
| `archive/` | Clearly identified earlier controller versions and results |

There are no physical Lab III run data or completed experimental report in this repository yet. The synthetic results must not be presented as physical measurements.

GitHub: [intr5100-lab-03-look-behind-acc](https://github.com/patricktiu1205/intr5100-lab-03-look-behind-acc). Publication checks are recorded in `review/publication_checks.json`.
