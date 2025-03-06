# FRC Robot Control Button Assignments

## Driver Controller (Port 0)

| Button | Function |
|--------|----------|
| A Button | Put drive into brake mode (setX) |
| Left Stick Y-Axis | Drive forward/backward |
| Left Stick X-Axis | Drive left/right |
| Right Stick X-Axis | Rotate robot |

## Operator Controller (Port 1)

| Button | Function |
|--------|----------|
| A Button | Move coral arm forward |
| B Button | Move coral arm in reverse |
| X Button | Position algae arm for algae pickup |
| Y Button | Position algae arm to hold algae |
| Right Bumper | Run end effector wheel in reverse |
| Left Bumper | Run end effector wheel forward |
| Left Trigger | Run algae intake (proportional to trigger pressure) |
| Right Trigger | Run algae intake in reverse (proportional to trigger pressure) |
| POV Left | Incrementally adjust arm position (positive) |
| POV Right | Incrementally adjust arm position (negative) |
| POV Up | Run algae arm forward |
| POV Down | Run algae arm in reverse |

## Available Buttons (Not Currently Assigned)

### Driver Controller
- B Button - *Available* (commented code for field-oriented mode)
- X Button - *Available* (commented code for robot-oriented mode)
- Y Button - *Available* (commented code for moving to specific pose)
- Right Bumper - *Available*
- Left Bumper - *Available*
- Start Button - *Available* (commented code for reset function)
- Back Button - *Available* (commented code for reset function)
- POV Up - *Available* (commented code for speed control)
- POV Down - *Available* (commented code for speed control)
- POV Right - *Available* (commented code for speed control)
- POV Left - *Available* (commented code for speed control)
- Left Trigger - *Available*
- Right Trigger - *Available*

### Operator Controller
- Start Button - *Available*
- Back Button - *Available*

## Subsystems

### Drive Subsystem
- Controls robot movement with swerve drive
- Default command: Drive using joysticks with deadband

### Arm Subsystem
- Controls the coral arm
- Default command: Incremental control using POV Left/Right buttons

### End Effector Subsystem
- Controls the wheel mechanism on the end effector
- No default command, activated by bumper buttons

### Algae Subsystem
- Controls algae intake and arm position
- No default command, controlled by various buttons

## Notes
- The code is organized into subsystems with their corresponding commands
- Button bindings are grouped by subsystem for better organization
- Algae intake uses proportional control with triggers
- Autonomous mode includes a simple forward trajectory
- Many driver buttons have commented code suggesting future implementation