package opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import Config.DriveConstants;
import opmodes.DriveCoords;
import subsystems.IntakeSubsystem;
import subsystems.MecanumDriveSubsystem;
import subsystems.ShooterSubsystem;


import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * Sample autonomous opmode using RoadRunner actions.
 *
 * This opmode demonstrates how to use RoadRunner's action builder to:
 * - Move forward (lineToX)
 * - Turn (turn)
 * - Move sideways (lineToY)
 * - Use splines for smooth curved paths
 *
 * Modify the actions in runOpMode() to create your autonomous routine.
 */


@Autonomous(name = "Red Back Score", group = "Auto")
public class RedBackScore extends LinearOpMode {

    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;

    private Motor frontLeft, frontRight, backLeft, backRight, intakeMotor;
    private MotorEx shooterMotor;
    private Servo sortArm, leftSafety, rightSafety;
    private CRServo leftFeeder, rightFeeder;
    private GyroEx gyro;
    private Limelight3A limelightApriltag;
    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private MecanumDrive mecanumDrive;

    private void initDriveWheels() {
        frontLeft = new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        frontRight = new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        backLeft = new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        backRight = new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_312);

        frontLeft.setInverted(true);
        backLeft.setInverted(true);

        frontLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.stopAndResetEncoder();
        frontRight.stopAndResetEncoder();
        backLeft.stopAndResetEncoder();
        backRight.stopAndResetEncoder();

        frontLeft.setRunMode(Motor.RunMode.VelocityControl);
        frontRight.setRunMode(Motor.RunMode.VelocityControl);
        backLeft.setRunMode(Motor.RunMode.VelocityControl);
        backRight.setRunMode(Motor.RunMode.VelocityControl);

        frontLeft.setVeloCoefficients(1.2, 0, 0.01);
        frontRight.setVeloCoefficients(1.2, 0, 0.01);
        backLeft.setVeloCoefficients(1.2, 0, 0.01);
        backRight.setVeloCoefficients(1.2, 0, 0.01);

        frontLeft.setFeedforwardCoefficients(0.4, 0.6, 0.5);
        frontRight.setFeedforwardCoefficients(0.4, 0.6, 0.5);
        backLeft.setFeedforwardCoefficients(0.2, 0.6, 0.5);
        backRight.setFeedforwardCoefficients(0.2, 0.6, 0.5);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        frontLeft.setInverted(true);
        backLeft.setInverted(true);

        frontLeft.encoder.reset();
        frontRight.encoder.reset();
        backLeft.encoder.reset();
        backRight.encoder.reset();
        frontLeft.encoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
        frontRight.encoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
        backLeft.encoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
        backRight.encoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);

        backRight.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initGyro() {
        gyro = new GyroEx() {
            IMU imu = hardwareMap.get(IMU.class, "imu");

            @Override
            public void init() {
                RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                        RevHubOrientationOnRobot.LogoFacingDirection.UP;
                RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
                RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                        logoDirection, usbDirection);

                imu.initialize(new IMU.Parameters(orientationOnRobot));
                imu.resetYaw();
            }

            @Override
            public double getHeading() {
                return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }

            @Override
            public double getAbsoluteHeading() {
                return 0;
            }

            @Override
            public double[] getAngles() {
                return new double[]{
                        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),
                        imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES),
                        imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES)
                };
            }

            @Override
            public com.arcrobotics.ftclib.geometry.Rotation2d getRotation2d() {
                return new com.arcrobotics.ftclib.geometry.Rotation2d(getHeading());
            }

            @Override
            public void reset() {
                imu.resetYaw();
            }

            @Override
            public void disable() {
            }

            @Override
            public String getDeviceType() {
                return "Internal IMU";
            }
        };

        gyro.init();
    }

    private void initLimelight() {
        try {
            limelightApriltag = hardwareMap.get(Limelight3A.class, "limelight");
        } catch (Exception e) {
            limelightApriltag = null;
            telemetry.addLine(" Lost Limelight /n");
            telemetry.update();
            return;
        }

        limelightApriltag.pipelineSwitch(0);
        limelightApriltag.start();
    }

    private void initIntake()
    {
        try {
            intakeMotor = new Motor(hardwareMap, "intake");
            sortArm = hardwareMap.get(Servo.class, "sortArm");

        } catch (Exception e) {
            telemetry.addData("Warning", "Intake failed to init");
            telemetry.addData("Warning", e);
            telemetry.update();
            intakeMotor = null;
            sortArm = null;
        }
    }

    private void initShooter()
    {
        try {
            shooterMotor = new MotorEx(hardwareMap, "shooter");
            leftFeeder  = new CRServo(hardwareMap, "leftFeeder");
            rightFeeder = new CRServo(hardwareMap, "rightFeeder");
            leftSafety = hardwareMap.get(Servo.class, "leftSafety");
            rightSafety = hardwareMap.get(Servo.class, "rightSafety");
        } catch (Exception e) {
            telemetry.addData("Warning", "Shooter failed to init");
            telemetry.addData("Warning", e);
            telemetry.update();
            shooterMotor = null;
            leftFeeder = null;
            rightFeeder = null;
            leftSafety = null;
            rightSafety = null;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        initDriveWheels();
        initGyro();
        initLimelight();
        initIntake();
        initShooter();

        // Initialize the drive subsystem (pass null for webcam since we're using Limelight)
        Pose2d beginPose = new Pose2d(62, 15, Math.PI);
        edu.wpi.first.math.geometry.Pose2d bpWpi = new edu.wpi.first.math.geometry.Pose2d( 62, 15, new edu.wpi.first.math.geometry.Rotation2d(Math.PI));
        mecanumDriveSubsystem = new MecanumDriveSubsystem(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                gyro,
                null,  // No webcam - using Limelight instead
                limelightApriltag,
                bpWpi,
                telemetry,
                hardwareMap
        );

        if (intakeMotor != null && sortArm != null) {
            intakeSubsystem = new IntakeSubsystem(
                    intakeMotor,
                    sortArm,
                    telemetry
            );
        } else {
            intakeSubsystem = null;
        }

        if (shooterMotor != null && leftFeeder != null && rightFeeder != null &&
                leftSafety != null && rightSafety != null) {
            shooterSubsystem = new ShooterSubsystem(
                    leftFeeder,
                    rightFeeder,
                    shooterMotor,
                    leftSafety,
                    rightSafety,
                    telemetry
            );
        } else {
            shooterSubsystem = null;
        }

        mecanumDriveSubsystem.enableDrive();

        // Get the RoadRunner MecanumDrive instance
        mecanumDrive = mecanumDriveSubsystem.getMecanumDrive();

        // Starting pose (in inches for RoadRunner)


        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Ready to start. Press PLAY to begin autonomous.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addData("Status", "Running RoadRunner actions...");
        telemetry.update();

        // ===== ROADRUNNER ACTIONS =====
        // Modify these actions to create your autonomous routine

        // APPROACH 1: Chain all actions together (recommended)
        // RoadRunner automatically uses the end pose of one action as the start of the next

        VelConstraint slowVel = new TranslationalVelConstraint(10);


        Actions.runBlocking(
                new SequentialAction(
                        shooterSubsystem.shoot_far(),

                        mecanumDrive.actionBuilder(beginPose)
                                .strafeToSplineHeading(DriveCoords.RedShootFar.position, DriveCoords.RedShootFar.heading)
                                .build(),

                        new SleepAction(8),

                        shooterSubsystem.feed(),

                        intakeSubsystem.turnOnIntake(),

                        new SleepAction(4),

                        shooterSubsystem.stopFeeding(),

                        mecanumDrive.actionBuilder(DriveCoords.RedShootFar)
                                        .strafeToSplineHeading(DriveCoords.RedPickup3.position, DriveCoords.RedPickup3.heading)
                                        .build(),

                        new ParallelAction(
                                intakeSubsystem.turnOnIntake(),

                                mecanumDrive.actionBuilder(DriveCoords.RedPickup3)
                                        .strafeTo(DriveCoords.RedPickup3End.position)
                                        .build()
                        ),

                        mecanumDrive.actionBuilder(DriveCoords.RedPickup3End)
                                .strafeToSplineHeading(DriveCoords.RedShootFar.position, DriveCoords.RedShootFar.heading)
                                .build(),

                        shooterSubsystem.feed(),

                        intakeSubsystem.turnOnIntake(),

                        new SleepAction(3.5),

                        shooterSubsystem.stopFeeding(),

                        mecanumDrive.actionBuilder(DriveCoords.RedShootFar)
                                .strafeToSplineHeading(DriveCoords.RedPickup2.position, DriveCoords.RedPickup2.heading)
                                .build(),

                        new ParallelAction(
                                intakeSubsystem.turnOnIntake(),

                                mecanumDrive.actionBuilder(DriveCoords.RedPickup2)
                                        .strafeTo(DriveCoords.RedPickup2End.position)
                                        .build()
                        ),

                        mecanumDrive.actionBuilder(DriveCoords.RedPickup2End)
                                .strafeToSplineHeading(DriveCoords.RedShootFar.position, DriveCoords.RedShootFar.heading)
                                .build(),


                        shooterSubsystem.feed(),

                        intakeSubsystem.turnOnIntake(),

                        new SleepAction(3.5)
                )

        );


        /*Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(startPose)
                                .lineToYSplineHeading(40, Math.toRadians(90))
                                .build(),

                        lift.dropCubeAction(), // Your subsystem code here

                        drive.actionBuilder(new Pose2d(0, 40, Math.toRadians(90)))
                                .lineToY(0)
                                .build()
                )
        );*/

        // APPROACH 2: Separate actions (get current pose between actions)
        // Uncomment this section and comment out APPROACH 1 to use separate actions
        /*
        Actions.runBlocking(
                mecanumDrive.actionBuilder(beginPose)
                        .lineToX(24)  // Move forward 24 inches
                        .build()
        );

        // Get current pose from localizer for next action
        Pose2d currentPose = mecanumDrive.localizer.getPose();
        Actions.runBlocking(
                mecanumDrive.actionBuilder(currentPose)
                        .turn(Math.toRadians(90))  // Turn 90 degrees
                        .build()
        );
        */

        // APPROACH 3: Use splines for smooth curved movement
        // Uncomment this section and comment out APPROACH 1 to use splines
        /*
        Actions.runBlocking(
                mecanumDrive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(30, 30), Math.PI / 2)  // Smooth curve to (30, 30) facing 90 degrees
                        .splineTo(new Vector2d(0, 60), Math.PI)      // Continue curve to (0, 60) facing 180 degrees
                        .build()
        );
        */

        telemetry.addData("Status", "Autonomous complete!");
        telemetry.update();

        // Cleanup
        if (limelightApriltag != null) {
            limelightApriltag.stop();
        }
    }
}
