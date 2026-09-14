# Experiment 2 submission materials

This directory follows the submission list in the supplied fixed-point grasping guide:

- `code/`: ROS 2 programs, Launch files, YAML parameters, robot descriptions and the one-command runner.
- `evidence/`: five-cycle grasp records, abnormal-case log and checksums.
- `report/`: English LaTeX source, compiled PDF and report figures.
- `videos/`: simulation WebM and real-machine MP4 demonstrations.

The fixed sequence is `initial → above_a → crab → pick → lift → initial_2 → release_ready → release → initial`. The two included CSV files are independent Ubuntu simulation runs, each with 5/5 passed cycles. The real-machine operation is documented by the supplied MP4; no numerical hardware table is inferred from the video.
