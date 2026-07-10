# reactor-uc zephyr-west playground for 6LoWPAN experiments

This repository constitutes the playground and evaluation basis for the thesis
"Design and Evaluation of Federated Lingua Franca for Low-Power IoT Networks over 6LoWPAN".

## Repository overview

- `src/` contains some example Lingua Franca applications, including `PingPong.lf`.
- `apps/` contains example applications and experiments for the 6LoWPAN work.
- `drivers/` contains the custom DW3000 IEEE 802.15.4 driver implementation and related support code.
- `tools/` contains local build tooling. In particular, `tools/lfc-mono.py` wraps `lfc-dev` and then builds the generated federation through `tools/lf-federation-build.py`, which is usually faster for federated applications because it can reuse generated C code and Zephyr/reactor-uc build artifacts across federates.
- `py/` contains additional Python scripts for deploying and managing experiments on testbeds.

The Glossy implementation and related driver additions live on **this** branch! :) They may be merged into the main reactor-uc repository in the future, but for now they are kept here to avoid polluting the main repository with additional code. Furthermore, the IEEE802.15.4 driver changes needed to support coexistence with Glossy are also kept on this branch, separating them from the main (and first/original) driver implementation in reactor-uc.

They are kept on that branch because they alter driver performance and therefore change experimental results.

![Zephyr Logo](https://upload.wikimedia.org/wikipedia/commons/thumb/6/64/Zephyr_RTOS_logo_2015.svg/640px-Zephyr_RTOS_logo_2015.svg.png)

- **Git:** <https://github.com/zephyrproject-rtos/zephyr/>
- **Supported Boards:** <https://docs.zephyrproject.org/4.1.0/boards/index.html>
- **Documentation:** <https://docs.zephyrproject.org/4.1.0/>

## 1. Prerequisites

### 1.1. Basic

You must use one of the following operating systems:

- `Linux` Officially supported are Debian & Ubuntu
- `macOS`

Your system must have the following software packages (you likely have at least some of these already):

- `git` — [a distributed version control system](https://git-scm.com/)
- `java` — [Java 17](https://openjdk.org/projects/jdk/17)

Additionally, you might have to additionally install gcc-multilib on Linux to be able to build for 32-bit targets:

```bash
sudo apt-get install gcc-multilib
```

### 1.2. Micro C Target for Lingua Franca

This repository uses [a customization of reactor-uc](https://github.com/lassezuengel/reactor-uc), the "micro C" target for Lingua Franca. Clone this repo with one of the following commands:

#### Clone via HTTPS

```bash
git clone https://github.com/lassezuengel/reactor-uc --recurse-submodules
```

#### Or Clone via SSH

```bash
git clone git@github.com:lassezuengel/reactor-uc.git --recurse-submodules
```

And make sure that the `REACTOR_UC_PATH` environment variable is pointing to it.

### 1.3. Building an Application

To build an LF application directly with `lfc-dev`, pass the LF source file to the compiler from this repository:

```sh
$REACTOR_UC_PATH/lfc/bin/lfc-dev src/PingPong.lf
```

If `REACTOR_UC_PATH` is not set, use the absolute path to your reactor-uc checkout instead:

```sh
<path/to/reactor-uc>/lfc/bin/lfc-dev src/PingPong.lf
```

For faster rebuilds of federated applications, use the local mono-build helper:

```sh
python3 tools/lfc-mono.py src/PingPong.lf
```

Using the `lfc-mono.py` script is also needed for building federated applications that use glossy clock sync, as this script automatically handles additional dependencies imposed by the glossy clock sync implementation.

### Getting started

Clone this new repository to your machine.
To start developing in this repo, you must first install the Zephyr dependencies, toolchains and SDK.
This requires you to follow selected parts of the Zephyr official Getting Started guide.

1. Install the dependencies used by Zephyr by following the steps in [Install Dependencies](https://docs.zephyrproject.org/4.1.0/develop/getting_started/index.html#install-dependencies)

2. Then install the Zephyr toolchains and SDK by following the steps in [Install the Zephyr SDK](https://docs.zephyrproject.org/4.1.0/develop/getting_started/index.html#install-the-zephyr-sdk)

Within your newly cloned project, create and activate a virtual environment for this project.

```sh
python3 -m venv ./venv
source ./venv/bin/activate
```

**IMPORTANT**: Remember to always activate the virtual environment before using the template.

Install the west, the Zephyr build tool

```sh
pip install west
```

Pull down the Zephyr RTOS sources using west (this can take a while)

```sh
west update
```

Install Python dependencies

```sh
pip install -r deps/zephyr/scripts/requirements.txt
```

Export a CMake package

```sh
west zephyr-export
```

## Troubleshooting

```sh
Command 'west' not found, did you mean:
```

or

```sh
Traceback (most recent call last):
  File "/home/erling/dev/lf-west-template/deps/zephyr/scripts/build/gen_kobject_list.py", line 62, in <module>
    import elftools
ModuleNotFoundError: No module named 'elftools'
```

Activate the virtual environment where the Zephyr dependencies are installed.
