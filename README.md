# Openpilot in Carla

This repository provides the [Openpilot](https://github.com/commaai/openpilot) confiuration to integrate with [Carla](https://carla.org/). The setup was originally implemented for safety evaluation purposes using the <img src="https://latex.codecogs.com/svg.image?\bg_white&space;\epsilon\delta" title="\bg_white \epsilon\delta" />-almost safe set based methods, but should also be applicable for other purposes. The code base is built upon the [original Opepilot-Carla integration](https://github.com/commaai/openpilot/tree/master/tools/sim) from comma.ai with added Radar inputs to enhance the stack's performance. 

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

[3] add radar paper to here

[4] add the comma.ai blog post to here
