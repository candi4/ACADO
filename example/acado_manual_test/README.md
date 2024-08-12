# acado_manual_test

I resolve issues with some examples in the manual due to minor problems (visualization and outdated notation). 

I also organize all example code from chapters 3 to 8 (except code generation), which are in the `src` folder.

To use this code, please:

1. Add `source <your ACADO toolkit path>/build/acado_env.sh` to your `~/.bashrc` file.
2. Replace `FindACADO.cmake` in the package folder with `<your ACADO toolkit path>/build/FindACADO.cmake`.

Once done, you can build the package and run all example files in each chapter folder. (If a core dump occurs, use the `sudo` command.)

This package eliminates unnecessary C++ tasks, allowing you to focus on learning the ACADO toolkit and applying it to your personal projects.

In addition, please understand how to visualize without using the ACADO visulization.(This is the most important thing in this package)


## Build
```shell
mkdir build
rm -r build/*
cd build
cmake ..
make
```