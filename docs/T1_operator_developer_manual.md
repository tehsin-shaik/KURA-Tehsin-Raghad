# Booster T1 Operator&Developer README

------------------------------------------------------------------------

# 1. System Architecture

## Hardware Versions

### Basic Version

-   Single board (Jetson AGX Orin)
-   Motion control + perception run on same board

### Standard Version

-   Motion Control Board (x86)
-   Perception Board (AGX Orin)
-   Motion and perception run independently

Always confirm your version before development: - SSH target differs -
Upgrade process differs - Logs retrieved separately (Standard)

------------------------------------------------------------------------

# 2. Network Configuration

## Wired Connection (Recommended)

Default robot static IP:

IP: 192.168.10.101
Netmask: 255.255.255.0
Gateway: 192.168.10.1

Example (Linux):

``` bash
sudo ip addr add 192.168.10.10/24 dev eth0
```

------------------------------------------------------------------------

# 3. SSH Access

## Basic Version

``` bash
ssh booster@192.168.10.101
```

Default password:

123456

## Standard Version

### Motion Control Board

``` bash
ssh master@192.168.10.101
```

### Perception Board

``` bash
ssh booster@<perception_ip>
```

If perception IP unknown:

``` bash
ssh master@192.168.10.101
ssh booster@192.168.10.102
```

------------------------------------------------------------------------

# 4. Robot Control Service Management

``` bash
# Start service
booster-cli launch -c start

# Stop service
booster-cli launch -c stop

# Restart service
booster-cli launch -c restart
```

Safety: - Robot must be supported before stopping service. - Never stop
service while robot is balancing upright.

------------------------------------------------------------------------

# 5. Robot Modes (Developer-Relevant)

  Mode      Description
  --------- -----------------------
  DAMP      Passive damping
  PREP      Rigid standing
  WALK      Predefined locomotion
  CUSTOM    Full SDK control
  PROTECT   Auto safety state

CUSTOM mode: - Full joint control - No automatic balance - Use
hoist/support - Switch from PREP or DAMP only

------------------------------------------------------------------------

# 6. Zero Calibration

Stop control service first.

``` bash
sudo /opt/booster/Tools/encos_reset_zero_pos_v2 all
```

After calibration: 1. Remove all zero parts 2. Restart service 3. Enter
PREP mode 4. Verify posture

------------------------------------------------------------------------

# 7. Firmware Version Check

``` bash
cat /opt/booster/version.txt
```

------------------------------------------------------------------------

# 8. Firmware Upgrade

## Method 1 --- CLI Upgrade (Recommended)

``` bash
booster-cli upgrade
```

(Standard version: run separately on motion and perception boards.)

## Method 2 --- Package-Based Upgrade (.run Installer)

Grant execution permission:

``` bash
sudo chmod +x <release-file>.run
```

Execute installer:

``` bash
sudo ./<release-file>.run
```

Standard version: - Upgrade motion board and perception board
separately.

------------------------------------------------------------------------

# 9. Factory Reset

Basic:

``` bash
cd /home/booster/Booster/recovery
sudo ./<recovery_file>.run
```

Standard: - Restore motion and perception boards separately.

------------------------------------------------------------------------

# 10. Log Retrieval

``` bash
booster-cli log -st YYYYMMDD-HHMMSS -et YYYYMMDD-HHMMSS -o OUTPUT_PATH
```

Example:

``` bash
booster-cli log -st 20200808-120800 -et 20200808-120820 -o /home/master/Documents
```

Copy log:

``` bash
scp master@192.168.10.101:/home/master/Documents/20200808-120800.zip ~/Downloads
```

Default timezone: UTC+8

------------------------------------------------------------------------

# Quick Command Summary

``` bash
ssh booster@192.168.10.101

booster-cli launch -c start
booster-cli launch -c stop
booster-cli launch -c restart

cat /opt/booster/version.txt

booster-cli upgrade

sudo chmod +x file.run
sudo ./file.run

sudo /opt/booster/Tools/encos_reset_zero_pos_v2 all

booster-cli log -st <time> -et <time> -o <path>
```
