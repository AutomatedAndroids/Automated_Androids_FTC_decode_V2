package subsystems;

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

    public void sort(boolean side) { //false is left, true is right
        if (side) {
            sortArm.setPosition(middle - swing);
        } else {
            sortArm.setPosition(middle + swing);
        }

    }

    public void sort(){
        sortArm.setPosition(0.75);
    }

    public void turnOnIntake() {
        intakeMotor.set(1);
    }

    public void turnOffIntake() {
        intakeMotor.set(0);
    }

    @Override
    public void periodic() {
        super.periodic();
        //telemetry.update();
    }
}
