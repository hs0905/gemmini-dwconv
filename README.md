DWConv Dataflow on Gemmini
====================================

Quick Start
==========

We provide here a quick guide to installing Gemmini's dependencies (Chipyard and Spike), building Gemmini hardware and software, and then running that software on our hardware simulators.

Dependencies
---------

Before beginning, install the [Chipyard dependencies](https://chipyard.readthedocs.io/en/latest/Chipyard-Basics/Initial-Repo-Setup.html#requirements) that are described here.

Installing Chipyard
-----------------------------

Run these steps to install Chipyard and Spike (make sure to checkout the correct Chipyard and Spike commits as shown below):

```shell
git clone https://github.com/ucb-bar/chipyard.git
cd chipyard
git checkout 757d354410adbd886b76cd79299b35d5f8a0265c
```

Modify `.gitmodules` to point to this git repository

- from:
```
...
[submodule "generators/gemmini"]
	path = generators/gemmini
	url = https://github.com/ucb-bar/gemmini
...
```

- to:
```
...
[submodule "generators/gemmini"]
        path = generators/gemmini
        url = https://github.com/dwconv/gemmini-dwconv
...
```


```shell
git submodule sync
git submodule update --init generators/gemmini/
git -C generators/gemmini/ checkout dwconv
git -C generators/gemmini/ submodule update --init --recursive software/gemmini-rocc-tests

./build-setup.sh esp-tools

git -C generators/gemmini/ checkout dwconv
git -C generators/gemmini/software/gemmini-rocc-tests checkout dwconv

source env.sh

```

Setting Up Gemmini
------------------

Run the steps below to set up Gemmini configuration files, symlinks, and subdirectories:

```shell
cd chipyard/generators/gemmini
./scripts/setup-paths.sh
```



Building Gemmini Software
-------------------------

Run the steps below to compile Gemmini programs, including large DNN models like ResNet50, and small matrix-multiplication tests.

```shell
cd chipyard/generators/gemmini/software/gemmini-rocc-tests
./build.sh
```

Afterwards, you'll find RISC-V binaries in `build/`.

we provide `conv_dw_dwconv_test` program to test DWConv dataflow.

If needed, change the following parameters in `chipyard/generators/gemmini/software/gemmini-rocc-tests/bareMetalC/conv_dw_dwconv_test.c`
```
#define IN_DIM 14
#define CHANNELS 32
#define KERNEL_DIM 3
#define PADDING 1
#define STRIDE 1
```

Building Gemmini Hardware and Cycle-Accurate Simulators
-----------------------------------------------

Run the instructions below to build a cycle-accurate Gemmini simulator using Verilator.

```shell
cd chipyard/generators/gemmini
./scripts/build-verilator.sh

# Or, if you want a simulator that can generate waveforms, run this:
# ./scripts/build-verilator.sh --debug
```

Run Simulators
---------------

Run the instructions below to run the Gemmini RISCV binaries that we built previously, using the simulators that we built above:

```shell
cd chipyard/generators/gemmini

# Run a smaller workload in baremetal mode, on a cycle-accurate simulator
./scripts/run-verilator.sh conv_dw_dwconv_test


# Or, if you want to generate waveforms in `waveforms/`:
# ./scripts/run-verilator.sh --debug conv_dw_dwconv_test
```


# ISA

This section describes the additional ISAs for DWConv on top of the original Gemmini's assembly-level ISA.

## Configuration
### `config_ex` configures the Execute pipeline. If `rs1[2]` of the command is `1`, config_ex command configures dwconv as follows
**Format:** `config_ex rs1 rs2`

- `rs1[1:0]` must be `00`
- `rs1[2]` determines if DWConv dwconv (1) or normal weight stationary
- `rs1[22:15]` = `R`
- `rs1[31:23]` = Input map tile height
- `rs1[40:32]` = Input map tile width
- `rs1[49:41]` = output map tile height
- `rs1[58:50]` = output map tile width

- `rs2[7:0]` = Frame size 
- `rs2[13:8]` = Frame height
- `rs2[21:14]` = Actual number of output rows that are processed together. Normally, same as `R*2` if multi-stream processing is used. It can be smaller than `R*2` for the last rows when the input height is not a multiple of `R*2`
- `funct` = 0

### `gemmini_loop_dwconv_dw_ws` Depthwise Conv Loop (WS Dataflow)

DWConv computation 

Gemmini also includes a CISC instruction for convolutions, implemented similarly to the matmul CISC instruction.
`gemmini_loop_dwconv_dw_ws` will perform a depthwise convolution with the DWConv dataflow

Like `gemmini_loop_ws`, the inputs to a single `gemmini_loop_dwconv_dw_ws` call must fit within half of Gemmini's private memory, to support double-buffering.
If the programmer would like to perform larger depthwise convolutions, they must tile and wrap `gemmini_loop_dwconv_dw_ws` within an outer-loop.
