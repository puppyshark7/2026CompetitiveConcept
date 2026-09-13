<!--- View using Ctrl+k, v (press ctrl+k first, look at the bottom for (Ctrl+k) was pressed, waiting for second chord, and press v)--->

# Swerve Generator Instructions
When running the CTRE Swerve generator, the following block of code MUST be removed; otherwise the code will not function.

### Java
```java
    /**
     * Creates a Swerve instance.
     * This should only be called once in your robot program.
     */
    public static Swerve createDrivetrain() {
        return new Swerve(
            DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
        );
    }
```
The following import should also be removed as well.
### Java
```java
import frc.robot.subsystems.CommandSwerveDrivetrain;
```