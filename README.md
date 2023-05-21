# AT-NDO-MPCC
an AT-NDO-MPCC control scheme for Autonomous Driving

This work is based on the track data and MPCC control scheme from [MPCC](https://github.com/alexliniger/MPCC)

## Getting started
git clone the AT-NDO-MPCC repository and the submodule hpipm solver
```
git clone https://github.com/ruidev1/AT-NDO-MPCC.git
git submodule init
git submodule update
```
follow the instruction of [hpipm] to build the solver environment, we setup the environment in Linux:
1) From the BLASFEO root folder, run `make shared_library -j 4 && sudo make install_shared`
2) From the HPIPM root folder, run `make shared_library -j 4 && sudo make install_shared`
3) In a terminal, navigate to the folder `hpipm/interfaces/matlab_octave`.
Set the needed environment flags by running `source env.sh` (you may need to change the `BLASFEO_MAIN_FOLDER`, or to make it equal to the `BLASFEO_PATH`) in that folder.
Compile the interface by running `make all -j 4` (for Octave), or `make compile_mex_with_matlab` (for Matlab).

## Run
In a terminal, navigate to the folder `MPCC_Solver/MPCC`.
Set the needed environment flags by running `source env.sh` (you may need to change the `BLASFEO_MAIN_FOLDER`, or to make it equal to the `BLASFEO_PATH`) in that folder.
Run an instance of Matlab or Octave from the same terminal.

For our NDO-MPCC, run `compare_scenarios.m`

For AT-NDO-MPCC, run `rl_train_script.m`, we load our trained agent in `/savedAgent`, you can also train agents with this script
