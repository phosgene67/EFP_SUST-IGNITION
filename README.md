# EFP_SUST-IGNITION

Escape from Phobos controller repository for **Team SUST IGNITION**, representing
**Shahjalal University of Science and Technology, Bangladesh** in Space Teams University 2026.

The main controller file is **`Controller.py`**. It runs within the official
Space Teams simulation environment and controls the crew vehicle using radar,
RGB cameras, and depth cameras.

## Team

**Team members**

- Md. Abdullah Al Sami Chowdhury
- Zarif Hasan Sadiq
- Arjun Shill

**Team Advisor**

Irfan Nafiz Shahan

Lecturer, Department of Electrical & Electronics Engineering

Shahjalal University of Science and Technology (SUST)

## Setup Guide

1. Set up Space Teams PRO using the [official competition wiki](https://github.com/SimDynamX/STU4-Solution/wiki).
2. Configure the development and simulator checkouts as described in **Local Repository Setup** below.
3. Keep the repository's supporting files alongside `Controller.py`, including
   `UserAPI.py` and `camera_viewer.py`.
4. After pushing changes from the development checkout, use **Pull** in Space Teams
   to update the simulator checkout, then run `FINAL_SUBMISSION.simconf`.

The camera window opens during the simulation. See the
[camera interface setup guide](CAMERA_GUI.md) for details and troubleshooting.

[Competition rules and information](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/218)
| [Competition forum](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/categories/2026-escape-from-phobos)

### Install Python Packages

`requirements.txt` includes NumPy, SciPy, OpenCV, and Pillow, pinned to the
versions tested with Space Teams PRO v0.56.0 and Python 3.12.

Open PowerShell in the development repository and install the packages into
the simulator's bundled Python environment:

```powershell
Set-Location "E:\EFP_SUST-IGNITION-main"
& "$env:LOCALAPPDATA\SimDynamX\STPro\Versions\v0.56.0\py\Scripts\python.exe" -m pip install -r requirements.txt
```

If you use a different Space Teams version, replace `v0.56.0` with its installed
version folder.

For a separate local Python 3.12 environment, run this from the repository folder
using that environment's Python:

```powershell
python -m pip install -r requirements.txt
```

Packages are installed into the Python environment used by the command. Installing
them into your local development environment does not update the simulator's
bundled environment.

Space Teams PRO supplies the `spaceteams` SDK and simulation engine separately.
Continue to launch `Controller.py` through the simulator.

### Local Repository Setup

The GitHub repository is `EFP_SUST-IGNITION`. Use these two separate checkouts:

- **Development:** `E:\EFP_SUST-IGNITION-main` — edit code here, then commit and push to GitHub.
- **Simulator:** `C:\Users\Asus\AppData\Local\SimDynamX\STPro\Assets\Repos\STU4-Solution` — update this checkout using **Pull** in Space Teams.

The simulator checkout must be named `STU4-Solution` because the official
competition configuration references `Local/Repos/STU4-Solution/Controller.py`.
The development folder can keep its current name. Keep the GitHub repository
name (`EFP_SUST-IGNITION`) unchanged.

After editing: **commit and push from the development repository, then Pull in
the simulator**. Avoid editing files or using Commit & Push in the simulator
checkout; separate local commits there can cause divergence.
