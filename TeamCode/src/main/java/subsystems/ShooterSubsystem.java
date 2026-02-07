package subsystems;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubsystem extends SubsystemBase {

    private CRServo leftFeeder, rightFeeder;
    private Servo leftSafety, rightSafety;
    private MotorEx shooterMotor;
    private Telemetry telemetry;

    // --- CONFIGURATION ---
    private static final double TICKS_PER_REV = 28.0;
    private static double TARGET_RPM_CLOSE = 2750;
    private static double TARGET_RPM_FAR = 3000;
    private static final double SERVO_BOTTOM = 0.25;
    private static final double SERVO_TOP = 0.5;

    public ShooterSubsystem(CRServo leftFeeder, CRServo rightFeeder, MotorEx shooterMotor, Servo leftSafety, Servo rightSafety, Telemetry telemetry) {
        this.leftFeeder = leftFeeder;
        this.rightFeeder = rightFeeder;
        this.shooterMotor = shooterMotor;
        this.leftSafety = leftSafety;
        this.rightSafety = rightSafety;
        this.telemetry = telemetry;

        leftFeeder.setInverted(true);
        rightFeeder.setInverted(true);

        // Configure Motor for Velocity Control
        shooterMotor.setRunMode(Motor.RunMode.VelocityControl);
        this.shooterMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        this.shooterMotor.setVeloCoefficients(0.05, 0, 0);
    }

    // --------------------------------------------------------
    // ROADRUNNER ACTIONS
    // Use these in your Actions.runBlocking(new SequentialAction(...))
    // --------------------------------------------------------

    /**
     * Sets the flywheel to the 'Close' RPM target.
     */
    public Action shoot_close() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                shooterMotor.setVelocity((TARGET_RPM_CLOSE / 60.0) * TICKS_PER_REV);
                return false; // Returns false because the command is sent instantly
            }
        };
    }

    /**
     * Sets the flywheel to the 'Far' RPM target.
     */
    public Action shoot_far() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                shooterMotor.setVelocity((TARGET_RPM_FAR / 60.0) * TICKS_PER_REV);
                return false;
            }
        };
    }

    /**
     * Activates both feeders and moves safety servos to the feeding position.
     */
    public Action feed() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftFeeder.set(1);
                leftSafety.setPosition(SERVO_BOTTOM);
                rightFeeder.set(-1);
                rightSafety.setPosition(SERVO_TOP);
                return false;
            }
        };
    }

    /**
     * Stops the feeders and returns safety servos to the idle position.
     */
    public Action stopFeeding() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftFeeder.set(-0.001);
                leftSafety.setPosition(SERVO_TOP);
                rightFeeder.set(-0.001);
                rightSafety.setPosition(SERVO_BOTTOM);
                return false;
            }
        };
    }

    /**
     * Stops the shooter flywheels.
     */
    public Action stopFlywheels() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                shooterMotor.setVelocity(0);
                return false;
            }
        };
    }

    // --------------------------------------------------------
    // TELEOP / MANUAL METHODS
    // --------------------------------------------------------

    public void feedLeft() {

        leftFeeder.set(1);

        leftSafety.setPosition(SERVO_BOTTOM);



    }



    public void feedRight() {

        rightFeeder.set(-1);

        rightSafety.setPosition(SERVO_TOP);

    }

    public void stopLeft() {

        leftFeeder.set(-0.001);

// Ensures velocity is cleared

        leftSafety.setPosition(SERVO_TOP);

    }



    public void stopRight() {

        rightFeeder.set(-0.001);

// Ensures velocity is cleared

        rightSafety.setPosition(SERVO_BOTTOM);

    }

    public void increaseShootClose() { TARGET_RPM_CLOSE += 50; }
    public void decreaseShootClose() { TARGET_RPM_CLOSE -= 50; }
    public void increaseShootFar() { TARGET_RPM_FAR += 50; }
    public void decreaseShootFar() { TARGET_RPM_FAR -= 50; }

    public double getShooterRPM() {
        if (shooterMotor == null || shooterMotor.motorEx == null) {
            return 0.0;
        }
        DcMotorEx motor = shooterMotor.motorEx;
        double velocityTicksPerSecond = motor.getVelocity();
        return (velocityTicksPerSecond / TICKS_PER_REV) * 60.0;
    }

    @Override
    public void periodic() {
        super.periodic();
        if (telemetry != null) {
            telemetry.addData("Shooter Far Target", TARGET_RPM_FAR);
            telemetry.addData("Shooter Close Target", TARGET_RPM_CLOSE);
            telemetry.addData("Shooter Actual RPM", String.format("%.2f", getShooterRPM()));
        }
    }
}