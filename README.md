![Screenshot from 2022-02-21 10-33-56](https://user-images.githubusercontent.com/25329580/154986207-35126565-dd34-4511-9293-75874c366955.png)
# Openpilot in Carla

This repository provides the [Openpilot](https://github.com/commaai/openpilot) confiuration to integrate with [Carla](https://carla.org/). The setup was originally implemented for safety evaluation purposes using the <img src="https://latex.codecogs.com/svg.image?\bg_white&space;\epsilon\delta" title="\bg_white \epsilon\delta" />-almost safe set based methods, but should also be applicable for other purposes. The code base is built upon the [original Opepilot-Carla integration](https://github.com/commaai/openpilot/tree/master/tools/sim) from comma.ai with added Radar inputs to enhance the stack's performance. 

Environments
-----------------------

1. Openpilot - 0.8.10
2. Carla - 0.9.12
3. SDQ_Tools - 1.0 ([source code](https://gitlab.com/Bobeye/sdq_tools))

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

[3] Zhong et al. Detecting Safety Problems of Multi-Sensor Fusion in Autonomous Driving (https://arxiv.org/pdf/2109.06404.pdf)

[4] What are the limitations of openpilot Adaptive Cruise Control? (https://comma.ai/vehicles)

[5] The EURO NCAP AEB C2C Test Protocal (https://cdn.euroncap.com/media/56143/euro-ncap-aeb-c2c-test-protocol-v302.pdf)

Citing
------
```
@misc{weng2022formal,
      title={A Formal Safety Characterization of Advanced Driver Assist Systems in the Car-Following Regime with Scenario-Sampling}, 
      author={Bowen Weng and Minghao Zhu and Keith Redmill},
      year={2022},
      eprint={2202.08935},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```
