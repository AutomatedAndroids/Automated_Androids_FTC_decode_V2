package subsystems;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    private Motor intakeMotor;
    private Servo sortArm;
    private Telemetry telemetry;

    private final double middle = 0.47;
    private final double swing = 0.19;

    public IntakeSubsystem(Motor intakeMotor, Servo sortArm, Telemetry telemetry) {
        this.intakeMotor = intakeMotor;
        this.sortArm = sortArm;
        this.telemetry = telemetry;
    }

    // --------------------------------------------------------
    // ROADRUNNER ACTIONS
    // --------------------------------------------------------

    /**
     * Turns the intake motor on at full power.
     */
    public Action turnOnIntake() {
        return packet -> {
            intakeMotor.set(1);
            return false;
        };
    }

    /**
     * Turns the intake motor off.
     */
    public Action turnOffIntake() {
        return packet -> {
            intakeMotor.set(0);
            return false;
        };
    }

    /**
     * Moves the sort arm to the specific side.
     * @param side false for left (middle + swing), true for right (middle - swing)
     */
    public Action sort(boolean side) {
        return packet -> {
            if (side) {
                sortArm.setPosition(middle - swing);
            } else {
                sortArm.setPosition(middle + swing);
            }
            return false;
        };
    }

    /**
     * Moves the sort arm to the default sort position (0.75).
     */
    public Action sortDefault() {
        return packet -> {
            sortArm.setPosition(0.75);
            return false;
        };
    }

    /**
     * Reverses the intake motor (useful for clearing jams in auto).
     */
    public Action reverseIntake() {
        return packet -> {
            intakeMotor.set(-1);
            return false;
        };
    }

    // --------------------------------------------------------
    // TELEOP / MANUAL METHODS
    // --------------------------------------------------------

    public void setIntakePower(double power) {
        intakeMotor.set(power);
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}