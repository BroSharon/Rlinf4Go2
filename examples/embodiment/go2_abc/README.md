# Go2 ABC Experiments

[简体中文](README.zh-CN.md)

Run Go2 omnidirectional movement (A), rough terrain (B), and stairs (C) on a remote Slurm A40 node. Each task has a saved PPO baseline, an ordinary PPO continuation, and an RLinf-Huber continuation.

The continuations start from the same task-specific baseline weights with a fresh optimizer, use training seed 0, and add 1,000 updates. A and B use 2,048 parallel environments; C uses 1,024. The baseline group evaluates its saved weights; each continuation trains and then evaluates. Evaluation uses seeds 23456 and 34567.

## Prepare Inputs

From the root of a clean checkout on your Slurm cluster, set the paths to its prepared runtime and shared assets.

```bash
export GO2_ABC_RUNTIME=/path/to/prepared-runtime
export GO2_ABC_SHARED_RUNTIME=/path/to/shared-runtime
export GO2_ABC_IMAGE="$GO2_ABC_SHARED_RUNTIME/containers/isaac-lab_2.3.2.sif"
export GO2_ABC_CONTAINER=singularity
```

The runtime includes Isaac Lab 2.3.2, Isaac Sim 5.1, RSL-RL 3.1.2, the benchmark, scripts, package overlays, and assets. It uses the pinned [multi-task benchmark](https://github.com/viktoriageorrg04/bsc-thesis-multi-task-RL-env) for simulator tasks and ordinary PPO. The runtime's `benchmark` and `IsaacLab` directories must remain Git checkouts with HEADs matching [manifest.json](manifest.json). On another cluster, set these variables to your own compatible runtime, shared assets, image, and container executable. The public repository alone does not reconstruct this runtime.

Prepare the A baseline from the pinned manifest.

```bash
python -m examples.embodiment.go2_abc.prepare_data --task A
```

For B or C, use the matching checkpoint and destination directory. The runner checks the selected baseline against its manifest hash.

## Submit a Job

From the remote repository root, submit an A RLinf-Huber continuation to the cluster:

```bash
sbatch --gres gpu:a40:1 --cpus-per-task 4 --mem 64G --time 06:00:00 examples/embodiment/go2_abc/run_a40.sbatch A_rlinf
```

Use `A_baseline` for baseline evaluation or `A_ppo` for ordinary PPO continuation. Replace A with B or C after preparing that task's baseline. Adjust the partition, QoS, account, and resources to match your cluster. The submitted job starts the container and runs on the allocated A40 compute node. A submitted job is not a completed run; check its state with `squeue -j <job-id>` or `sacct -j <job-id>` and inspect its logs.

The historical A and B continuations also ran two-update checks with 64 and 2,048 environments before the full run. This entry point runs the full configuration directly and does not include those checks.

Every experiment requires a fresh `data/go2_abc/runs/<experiment>/` directory. For ordinary PPO, also use a runtime with a fresh `benchmark/logs/rsl_rl/<experiment>` path. The runner refuses to overwrite existing outputs.

The run directory contains stage `console.log` files and evaluation outputs; continuations also export `policy.pt`. Ordinary PPO's original training logs and checkpoints remain under `$GO2_ABC_RUNTIME/benchmark/logs/rsl_rl/<experiment>/`. Slurm's default output is `slurm-<jobid>.out`.

## Configure Experiments

The RLinf continuations share [config/_common.yaml](config/_common.yaml), with task overrides in [A_rlinf.yaml](config/A_rlinf.yaml), [B_rlinf.yaml](config/B_rlinf.yaml), and [C_rlinf.yaml](config/C_rlinf.yaml). Ordinary PPO uses [A_ppo.json](config/A_ppo.json), [B_ppo.json](config/B_ppo.json), and [C_ppo.json](config/C_ppo.json); baseline evaluation uses [A_baseline.json](config/A_baseline.json), [B_baseline.json](config/B_baseline.json), and [C_baseline.json](config/C_baseline.json).

The configurations retain 235 observations each for actor and critic, 12 actions, ELU layers `[512, 256, 128]`, 24 rollout steps, and five PPO epochs. RLinf uses Huber value loss, learning rate `3e-4`, entropy coefficient `0.003`, and analytical KL target `0.005`; ordinary PPO uses adaptive KL target `0.01`. B retains the numerical guard; C continuations use the late posture weights.

## Results and Offline Checks

The following results come from runs on an A40 cluster: all A and B groups and C RLinf-Huber used RLinf `e8815cac`, while C baseline and PPO used `0e8129b9`. Training used seed 0; evaluation seeds 23456 and 34567 are pooled by episode count. The experiments have not been rerun on the current branch. A and B report one run per group; a single training seed does not establish stability across seeds. A dash means stair passage was not evaluated for that task.

| Task | Group | Tracking success | Simulated stair passage |
| --- | --- | --- | --- |
| A: omnidirectional | PPO baseline | 91.47% (1887/2063) | — |
| A: omnidirectional | PPO continuation | 93.33% (1917/2054) | — |
| A: omnidirectional | RLinf-Huber continuation | 96.73% (1981/2048) | — |
| B: rough terrain | PPO baseline | 99.85% (2045/2048) | — |
| B: rough terrain | PPO continuation | 99.61% (2040/2048) | — |
| B: rough terrain | RLinf-Huber continuation | 99.85% (2045/2048) | — |
| C: stairs | PPO baseline | 99.85% (2045/2048) | 21.39% (438/2048) |
| C: stairs | PPO continuation | 100% (2048/2048) | 44.82% (918/2048) |
| C: stairs | RLinf-Huber continuation | 100% (2048/2048) | 45.17% (925/2048) |

At RLinf `e8815cac`, two independent C RLinf-Huber continuations with training seed 0 produced identical tensors in all 20 saved checkpoints and identical 2,048 evaluation records across seeds 23456 and 34567. The evaluation measured 100% tracking success and 45.1660% simulated stair passage. This agreement applies to C in the recorded A40 runtime and recipe; one training seed does not establish stability across seeds. The Slurm scheduler records show that all three A jobs and all three B retry jobs completed; this only establishes their job status.

For an optional offline check of nine model weights, download them and verify their hashes, policy loading, and analytical KL with Python 3.11, PyTorch, and RLinf dependencies. The weights may come from different historical training runs and are intended for offline replay; the check does not establish results from a new training run. Downloading requires network access; the check runs on CPU and does not require an A40:

```bash
python -m examples.embodiment.go2_abc.prepare_data
python -m examples.embodiment.go2_abc.audit
```

## Motion Control

[Motion Control](MOTION.md) describes ABC policy observation assembly, motor conversion, and SDK2 integration. [Go2 six-direction hardware control](../go2_sport/README.md) uses a separate SportClient path. ABC policy deployment, sensing, and independent stopping still require hardware validation.
