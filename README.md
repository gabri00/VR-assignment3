# Assignment 3 - Virtual Reality

Final assignment of the Virtual Reality course at [UNIGE](https://unige.it/).

## GNAM group

| Name Surname | ID |
| ------------ | -- |
| [Gabriele Nicchiarelli](https://github.com/gabri00) | S4822677 |
| [Alessio Mura](https://github.com/alemuraa)      | S4861320 |

## Requirements

- Unreal Engine 5.2
- Visual Studio 2022 (with Windows 10 SDK 10.0.19041 and latest .NET Framework SDK)
- Cesium plugin for Unreal
- Python 3.8
- WSL (with Ubuntu 20.04)
- ROS Noetic

## Before running the simulation

#### Python set up

Open a terminal and run:

```bash
pip install msgpack-rpc-python
pip install airsim
```

#### Install Colosseum (on Windows)

To install and build Colosseum on Windows open the `Developer Command Prompt for VS 2022`:

```bash
git clone https://github.com/CodexLabsLLC/Colosseum.git
cd Colosseum
build.cmd
```

For more details follow the steps at this [link](https://microsoft.github.io/AirSim/build_windows/).

#### Set up the Unreal environment

Form the Unreal editor create a `new C++ class`, leave everything as default, then close Unreal and Visual Studio.
Copy `Unreal\Plugins` into your project's folder and add the following lines in your `<project_name>.uproject`:

```json
"Modules": [
    {
        "AdditionalDependencies": [
            "AirSim"
        ]
    }
]

"Plugins": [
    {
        "Name": "AirSim",
        "Enabled": true
    }
]
```

Also add this line to the `Config\DefaultGame.ini` file in your project's directory:

```bash
+MapsToCook=(FilePath="/AirSim/AirSimAssets")
```

Finally, right click the `.uproject` and select `Generate Visual Studio Project Files`.

Full tutorial at this [link](https://microsoft.github.io/AirSim/unreal_custenv/).

#### Launch the project

Open the `<prject_name>.sln`. In Visual Studio select "DebugGame editor" and "Win64" as build configuration.

Now you can run the project by pressing `F5`.

#### Set up ROS in WSL

To install and build Colosseum, in your ROS workspace:

```bash
git clone https://github.com/CodexLabsLLC/Colosseum.git src/
cd src/Colosseum
./setup.sh
./build.sh
```

Afterwards, build your workspace using `catkin build`.

Full guide at this [link](https://microsoft.github.io/AirSim/airsim_ros_pkgs/).

## Problems and solution adopted

Everything is a problem :,(.

## Troubleshooting
Since AirSim is no more updated from 2022, all its dependencies should be based on a compatible version of Python. That is why you can NOT use `Python > 3.8`.
Eventually, consider to create a virtual environment with the correct Python version.

## References

- [Cesium for UE](https://cesium.com/learn/unreal/)
- [Colosseum](https://github.com/CodexLabsLLC/Colosseum)
- [AirSim documentation](https://microsoft.github.io/AirSim/)
- [D-Flight](https://www.d-flight.it/new_portal/)
- [SVEA platform](https://github.com/Fedezac/svea)
- [ROS Noetic](https://wiki.ros.org/noetic)

#### Old projects

- [Group AmaRa](https://github.com/AuroraD-Hub/VR_Assignment/tree/main) (Airsim)
- [Group I Dronati](https://github.com/mmatteo-hub/VR4R_Assignment/tree/main) (Airsim)
- [Group SPIDRONE](https://github.com/Nirmalkumar-007/VIRTUAL-REALITY-FOR-ROBOTICS-SPIDRONE) (Airsim)
- [Group Prusa](https://github.com/MatteoCarlone/Virtual_Reality) (Colosseum)

<!-- - [UE & VS setup](https://docs.unrealengine.com/4.27/en-US/ProductionPipelines/DevelopmentSetup/VisualStudioSetup/)
- [Full UE guide](https://github.com/mikeroyal/Unreal-Engine-Guide) -->

#### Supervisors

- [Gianni Vercelli](gianni.vercelli@unige.it)
- [Saverio Iacono](saverio.iacono@unige.it)
