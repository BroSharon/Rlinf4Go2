Go2 High-Level Control
======================

The Gymnasium task ``Go2DriveEnv-v1`` reads joint and IMU data from ``rt/lowstate`` by default. For a direction check, RLinf sends a body velocity command through Unitree's ``SportClient``. The Go2's onboard controller handles gait and joint control.

Overview
--------

.. grid:: 2 4 4 4

   .. grid-item-card:: Models

      Manual or fixed velocity commands.

   .. grid-item-card:: Algorithms

      No training recipe.

   .. grid-item-card:: Tasks

      State reception and six-direction velocity tracking.

   .. grid-item-card:: Hardware

      Go2 EDU and a Linux controller.

Tasks
~~~~~

.. list-table::
   :header-rows: 1

   * - Check
     - Requested action
   * - Read-only
     - State samples without motion commands.
   * - ``forward`` / ``backward``
     - Move forward or backward at 0.3 m/s.
   * - ``left`` / ``right``
     - Move sideways at 0.3 m/s.
   * - ``turn_left`` / ``turn_right``
     - Turn left or right at 0.5 rad/s.

Observation and Action
~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Field
     - Contract
   * - Observation
     - Orientation, angular velocity, twelve joint positions and velocities, battery voltage, and four raw foot-force readings.
   * - Action
     - Forward speed, leftward speed, and left turn rate, in that order. The respective limits are ±0.3 m/s, ±0.3 m/s, and ±0.5 rad/s.
   * - Reward
     - Zero. This check does not train a policy.

Setup and Run
-------------

**Prerequisites:** Follow the :doc:`installation guide </rst_source/start/installation>` to prepare the Linux controller. Use a dedicated network interface connected to the Go2 EDU.

.. tab-set::

   .. tab-item:: Go2 EDU

      **1. Install the SDK**

      On the Linux controller, follow the `Unitree SDK instructions <https://github.com/unitreerobotics/unitree_sdk2_python/tree/814556d15970dd2ecf1c9984e845ca02ab07e206>`_ to install native CycloneDDS 0.10.x and export ``CYCLONEDDS_HOME`` in your shell. The Python 3.11 bindings build against this library. From the RLinf repository root, install the SDK in the same shell:

      .. code-block:: bash

         bash requirements/install.sh embodied --env go2_sport
         source .venv/bin/activate

      Configure the network interface as described in the SDK instructions. The controller computer needs no GPU or policy weights.

      **2. Check state and motion**

      Replace ``<interface>`` with the network interface attached to the Go2. Preview a command without opening SDK channels:

      .. code-block:: bash

         python -m toolkits.realworld_check.check_go2_sport --interface <interface> --direction forward --dry-run

      The plan reports ``hardware_access: false``. Check state reception separately:

      .. code-block:: bash

         python -m toolkits.realworld_check.check_go2_sport --interface <interface> --readonly

      This writes ten state-validation results to ``outputs/go2_sport/``. To inspect a single checksum-verified joint and IMU sample from ``rt/lowstate``, run the read-only subscriber in a fresh process.

      .. code-block:: bash

         python -m toolkits.realworld_check.check_go2_lowstate --interface <interface>

      For motion, keep the original controller active, verify standing posture and clear space, and have the remote ready on site. Close other motion programs before running one direction:

      .. code-block:: bash

         python -m toolkits.realworld_check.check_go2_sport --interface <interface> --direction forward

      Choose one of the six direction names above and enter ``GO`` after the on-site checks. The check sends the command at 10 Hz for one second, then requests a stop. Use ``--seconds 2`` for two seconds or ``--output <path>`` to choose a new report file.

      .. warning::

         A stop request or a low SDK velocity estimate does not establish physical rest. Observe the robot directly and use the remote if motion continues. Network loss or process failure can defeat software stop requests.

      **3. Evaluate velocity tracking**

      From the repository root, run one dummy episode with the fixed-command policy. The default sends ten forward commands at 10 Hz. Choose any direction from the table above. Set ``rollout.model.command_magnitude`` to the approved speed in m/s for translation or rad/s for turns.

      .. code-block:: bash

         bash evaluations/run_eval.sh realworld realworld_go2_tracking
         bash evaluations/run_eval.sh realworld realworld_go2_tracking rollout.model.command_name=turn_left

      To check the SDK path on a development machine, run the mock evaluation. It launches the same entry point and checks the logged errors against three known offsets.

      .. code-block:: bash

         python -m pytest -q tests/e2e_tests/evaluations/test_go2_tracking.py

      On hardware, complete the on-site checks above first. In the Go2 hardware configuration, set ``interface`` to the robot NIC and ``enable_motion`` to true. Set ``env.eval.override_cfg.is_dummy=false`` to enable the SDK path. Independently verify that the first two components of ``rt/sportmodestate`` velocity measure forward and leftward body speed in m/s, and that positive yaw speed measures a left turn in rad/s. Record the source of that confirmation in ``env.eval.override_cfg.mapping_source``.

Visualization and Results
-------------------------

The report records interface errors and the SDK velocity estimate after the stop request. Record your observation of the motion and stop separately. Measure travel distance, turn angle, and stopping distance separately if your application requires them.

The tracking run writes the standard evaluation log and a ``go2_tracking_*.json`` episode record under the same timestamped directory in ``logs/``. Each successfully dispatched step uses the last finite SDK estimate received from dispatch up to, but excluding, the step end or an earlier stop. Feedback coverage is the fraction of dispatched steps with a match. The record includes the command, timestamps, estimate, feedback source and end reason.

For a completed episode with a confirmed mapping and at least one match, ``eval/go2_tracking_mae_*`` reports the mean absolute error per axis across the matched steps. The dummy episode echoes each command and should report ten valid steps and zero error. On hardware, the measurement comes from the robot's onboard estimator.
