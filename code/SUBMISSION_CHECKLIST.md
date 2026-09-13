# Experiment 2 submission checklist

| Required item | Location | Status |
|---|---|---|
| ROS 2 program | `src/mecharm_fixed_pick_sim/`, `src/mycobot_description/` | Included |
| Launch and parameter files | `src/mecharm_fixed_pick_sim/launch/`, `config/` | Included |
| Simulation model and A/B/safe-height configuration | `urdf/`, `worlds/`, `config/fixed_point_experiment.yaml` | Included |
| Simulation demo | `submission_evidence/videos/simulation_demo.webm` | Included |
| Real-machine demo | `submission_evidence/videos/real_robot_demo.mp4` | Included |
| Five-grasp simulation record | `submission_evidence/results/*.csv` | Two independent 5-cycle records; both 5/5 passed |
| Abnormal test record | `submission_evidence/docs/ERROR_LOG.md` | Included |
| Running instructions | `README.md` | Included |
| Lab report PDF | `report/experiment2_lab_report.pdf` | Included |
| Lab report source | `report/experiment2_lab_report.tex` | Included |
| Real-machine numeric five-cycle/safety record | Not present in v2 archive | Supplement if available |

The package was prepared from the v2 archive without rerunning the simulation. Generated LaTeX auxiliary files may remain in the report directory for local rebuild, while the PDF and `.tex` source are the submission files.
