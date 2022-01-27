# Delta Covering Openpilot

Environments
-----------------------

1. Openpilot - 0.8.10
2. Carla - 0.9.12

Setup
-----

1. place all python files in `openpilot/tools/sim/`
2. replace `lib` folder in `openpilot/tools/sim/`

Run
---

```Bash
# terminal 1
cd $CARLADIR && ./CarlaUE4.sh -nosound -quality-level=Epic -RenderOffScreen -fps=120
# terminal 2
cd $OPENPILOTDIR/tools/sim && ./launch_openpilot.sh
# terminal 3
cd $OPENPILOTDIR/tools/sim && python bridge_<scenarios>.py --low_quality

```

References
----------
[1] Weng et al. [2021a] (https://ieeexplore.ieee.org/document/9556594)

[2] Weng et al. [2021b] (https://ieeexplore.ieee.org/document/9594685)
