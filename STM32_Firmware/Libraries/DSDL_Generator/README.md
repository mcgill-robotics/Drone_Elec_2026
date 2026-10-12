Necessary files for writing code as dependancies for libcanard library can be found in dsdl_generated for px4/ardupilot dronecan nodes.  


The purpose of dsdl is to create the files required for the different uavcan V0 functions. There is a python script which does this, but is slightly complicated to create. These scripts should be able to be reused between all projects done with ardupilot/px4. The reason dronecan works like this is to ensure it is language agnostic.

The packages in this script need a virtual environment (or at least in the linux WSL in which I ran this - works on my machine).

--------------------------------------------------

Start virtual environment with:
source venv/bin/activate

End venv with:
deactivate

Create virtual environment with:
python3 -m venv venv

To install package dependancies for dronecan_dsdlc use pip install -r requirements.txt

Files were built using:
python3 dronecan_dsdlc/dronecan_dsdlc.py -O dsdl_generated DSDL/dronecan DSDL/uavcan DSDL/com DSDL/ardupilot

## Folders

- `DSDL/` — the DSDL message/service definitions (text files) for each vendor namespace, cloned from upstream DroneCAN/ArduPilot/vendor repos.
- `dronecan_dsdlc/`, `libcanard/`, `pydronecan/` — the upstream generator tooling and reference C implementation (each has its own README from upstream).
- `dsdl_generated/` — the output of the command above; the `.h`/`.c` files actually used by firmware are copied from here into [`../Drone_Can/UAVCAN_DSDL`](../Drone_Can/README.md).

## See also

- [Drone_Can library README](../Drone_Can/README.md) — where the generated output ends up and how firmware consumes it
- [STM32_Firmware README](../../README.md) — the projects that ultimately depend on this
