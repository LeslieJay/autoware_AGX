# Where is the source code?

Autoware uses **multiple Git repositories** managed through a meta-repository approach.
The `autoware` repository itself **does not contain the full source code**. Instead, it references many separately maintained repositories.

To manage these repositories, Autoware uses **[vcs2l](https://github.com/ros-infrastructure/vcs2l)** (`vcs`), which allows you to clone and keep multiple repositories in sync using manifest files under [repositories](../repositories) folder.

## Repository structure

- [`repositories/autoware.repos`](../repositories/autoware.repos) ➡️ The manifest file that lists essential Autoware repositories and their versions
- `src/` (here) ➡️ The directory where all Autoware source repositories will be cloned

⭐ Check the [**🔗 Repository structure and versioning documentation**](https://autowarefoundation.github.io/autoware-documentation/main/design/repository-structure/) for more details.

## Cloning the essential Autoware source code

⭐ Check the [**🔗‍ Installation guide**](https://autowarefoundation.github.io/autoware-documentation/main/installation/) for detailed setup instructions.

Here is a brief summary of the steps:

```bash
# Navigate to the repository root
cd autoware

# Clone the Autoware source code into src/
vcs import src < repositories/autoware.repos
```

This command will clone and check out the correct versions as defined in [`autoware.repos`](../repositories/autoware.repos)

> **Note**
> Make sure `vcs2l` is installed before running this command:
>
> ```bash
> sudo apt install python3-vcs2l
> ```

In the future, to update all source repositories to their latest versions as defined in the manifest file, run:

```bash
vcs pull src
```
ros2 bag play "$BAG_DIR" \
  -s sqlite3 \
  -r 0.2 \
  --clock 100 \
  --topics \
  /tf \
  /localization/kinematic_state \
  /localization/acceleration \
  /localization/initialization_state \
  /perception/object_recognition/objects \
  /perception/occupancy_grid_map/map \
  /vehicle/status/velocity_status \
  /vehicle/status/steering_status \
  /vehicle/status/gear_status \
  /vehicle/status/control_mode \
  /planning/mission_planning/route \
  /planning/mission_planning/state \
  /planning/route \
  /planning/route_state
