package commands;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import subsystems.MecanumDriveSubsystem;

public class DriverJoystickCommand extends CommandBase {

    private final MecanumDriveSubsystem m_drive;
    
    // Speed multipliers to maximize robot performance
    private static final double ROTATION_SPEED_MULTIPLIER = 2.5;  // Increased from 1.5 for faster rotation
    private static final double LINEAR_SPEED_MULTIPLIER = 2.5;  // Multiplier for x/y movement speed

    DoubleSupplier xSpdSupplier;
    DoubleSupplier ySpdSupplier;
    DoubleSupplier rotationSpdSupplier;
    BooleanSupplier fieldOrientedSupplier;
    BooleanSupplier robotOrientedSupplier;
    BooleanSupplier resetCurrentHeading2Zero;
    BooleanSupplier towLeftSupplier;
    BooleanSupplier towRightSupplier;
    DoubleSupplier precisionLeftSupplier;
    DoubleSupplier precisionRightSupplier;
    BooleanSupplier turnToForwardSupplier;
    BooleanSupplier turnToBackwardSupplier;
    BooleanSupplier turnToLeftSupplier;
    BooleanSupplier turnToRightSupplier;

    DoubleSupplier currentHeadingSupplier;

    public DriverJoystickCommand(
            DoubleSupplier xSpdFunction,
            DoubleSupplier ySpdFunction,
            DoubleSupplier rotationSpdFunction,
            BooleanSupplier fieldOrientedFunction,
            BooleanSupplier robotOrientedFunction,
            BooleanSupplier resetCurrentHeading2Zero,
            BooleanSupplier towLeftSupplier,
            BooleanSupplier towRightSupplier,
            DoubleSupplier precisionLeftSupplier,
            DoubleSupplier precisionRightSupplier,
            BooleanSupplier turnToForwardSupplier,
            BooleanSupplier turnToBackwardSupplier,
            BooleanSupplier turnToLeftSupplier,
            BooleanSupplier turnToRightSupplier,
            DoubleSupplier currentHeadingPI2NPI,
            MecanumDriveSubsystem drive)
    {
        this.xSpdSupplier = xSpdFunction;
        this.ySpdSupplier = ySpdFunction;
        this.rotationSpdSupplier = rotationSpdFunction;
        this.fieldOrientedSupplier = fieldOrientedFunction;
        this.robotOrientedSupplier = robotOrientedFunction;
        this.resetCurrentHeading2Zero = resetCurrentHeading2Zero;
        this.towLeftSupplier = towLeftSupplier;
        this.towRightSupplier = towRightSupplier;
        this.precisionLeftSupplier = precisionLeftSupplier;
        this.precisionRightSupplier = precisionRightSupplier;
        this.turnToForwardSupplier = turnToForwardSupplier;
        this.turnToBackwardSupplier = turnToBackwardSupplier;
        this.turnToLeftSupplier = turnToLeftSupplier;
        this.turnToRightSupplier = turnToRightSupplier;
        this.currentHeadingSupplier = currentHeadingPI2NPI;
        m_drive = drive;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        //m_drive.resetEncoders();
    }

    boolean doAutoHeading = false;
    double targetAutoHeading = 0;
    @Override
    public void execute() {
        if(resetCurrentHeading2Zero.getAsBoolean())
        {
            m_drive.resetHeading2Zero();
        }
        // Retrieve real-time inputs from joystick and button states
        double xSpeed = xSpdSupplier.getAsDouble();
        double ySpeed = ySpdSupplier.getAsDouble();
        double rotationSpeed = rotationSpdSupplier.getAsDouble();

        // Apply simple deadband filtering (filters were causing issues - removed)
        // Deadband for x/y movement
        if (Math.abs(xSpeed) < 0.05) xSpeed = 0.0;
        if (Math.abs(ySpeed) < 0.05) ySpeed = 0.0;
        // Deadband for rotation
        if (Math.abs(rotationSpeed) < 0.1) rotationSpeed = 0.0;
        
        double filteredXSpeed = xSpeed;
        double filteredYSpeed = ySpeed;
        double filteredTurningSpeed = rotationSpeed;

        // Retrieve current heading and control mode states
        double currentHeading = currentHeadingSupplier.getAsDouble();

        if( fieldOrientedSupplier.getAsBoolean()) {
            m_drive.setFieledRelative(true);
        }
        if( robotOrientedSupplier.getAsBoolean()) {
            m_drive.setFieledRelative(false);
        }

        // Tow mode and precision mode disabled - always use maximum performance
        boolean towMode = false;  // Disabled - never turns on
        double precisionMode = 0.0;  // Disabled - never turns on

        // Increase max output to allow higher speeds (this multiplies the final motor output)
        // Using 2.5x to match the speed multiplier
        m_drive.setMaxOutput(2.5);
        
        // Apply rotation speed multiplier directly to rotation input
        filteredTurningSpeed *= ROTATION_SPEED_MULTIPLIER;
        // Clamp rotation speed to valid range [-1.0, 1.0] for drive method
        filteredTurningSpeed = Math.max(-1.0, Math.min(1.0, filteredTurningSpeed));

        // Handling combinations of directional inputs
        boolean forward = turnToForwardSupplier.getAsBoolean();
        boolean backward = turnToBackwardSupplier.getAsBoolean();
        boolean left = turnToLeftSupplier.getAsBoolean();
        boolean right = turnToRightSupplier.getAsBoolean();

        // Initialize variables to track state and compute the target heading
        double targetHeading = 0;
        int directionCount = 0;

        // Calculate target heading based on button presses
        if (forward) {
            directionCount++;
            targetHeading += 0; // Forward is 0 degrees
        }
        if (backward) {
            directionCount++;
            targetHeading += 180; // Backward is 180 degrees
        }
        if (left) {
            directionCount++;
            targetHeading += 90; // Left is 90 degrees
        }
        if (right) {
            directionCount++;
            targetHeading -= 90; // Right is -90 degrees
        }

        if (directionCount > 0) {
            targetAutoHeading = targetHeading / directionCount; // Average if multiple buttons are pressed
        }

        // Normalize the target heading to be within -180 to 180 degrees
        targetAutoHeading = ((targetAutoHeading + 180) % 360) - 180;

        // Update auto-heading flag
        doAutoHeading = (directionCount > 0);

        // Apply automatic heading adjustments if required
        if (doAutoHeading) {
            m_drive.adjustToHeading(targetAutoHeading, currentHeadingSupplier.getAsDouble());
            doAutoHeading = false; // Reset the flag after adjustment begins
        } else {
            // Continue with the regular driving command - use filtered speeds for maximum performance
            m_drive.drive(filteredXSpeed, filteredYSpeed, filteredTurningSpeed, false, currentHeadingSupplier.getAsDouble());
        }
    }
}
