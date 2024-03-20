# Multirotor Simulator in C++

Build examples using BUILD_EXAMPLES flag:

```bash
mkdir build
cd build
cmake -DBUILD_EXAMPLES=true ..
make
```

For fly examples, launch the executable with the following command, from the root of the repository:
```
./build/examples/multirotor_simulator_fly_example
```

For fly test using trajectory generation, launch the executable with the following command, from the root of the repository:
```
./build/examples/trajectory_fly_example/multirotor_simulator_traj_gen_example
```

It will use the configuration files `simulation_config.yaml`, that are located in `examples/simulation_config.yaml` and in `examples/trajectory_fly_example/simulation_config.yaml`.

The results will be saved in `multirotor_log.csv`.

You can plot the results with the following command:
```
python3 examples/plot_results.py
```