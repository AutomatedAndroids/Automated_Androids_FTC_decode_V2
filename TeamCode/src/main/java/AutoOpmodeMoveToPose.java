import android.util.Size;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import Config.DriveConstants;
import commands.MecanumDynamicControllerCommand;
import subsystems.MecanumDriveSubsystem;
import util.DashServer;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Autonomous opmode that moves the robot to a specific Pose2D.
 * 
 * Modify the TARGET_POSE constant to set the desired position and rotation.
 */
@Autonomous(name = "Auto: Move To Pose", group = "Auto")
public class AutoOpmodeMoveToPose extends CommandOpMode {

    private Motor frontLeft, frontRight, backLeft, backRight;
    private GyroEx gyro;
    private AprilTagProcessor webcamAprilTag;
    private Limelight3A limelightApriltag;
    private MecanumDriveSubsystem mecanumDriveSubsystem;

    double ACHIEVABLE_MAX_DISTANCE_PER_SECOND;

    // ===== CONFIGURATION: Set your target pose here =====
    // Pose2d(x, y, rotation)
    // x and y are in meters
    // rotation is a Rotation2d (can use Rotation2d.fromDegrees(angle) for degrees)
    private static final Pose2d TARGET_POSE = new Pose2d(
            1.0,  // x position in meters (forward/backward)
            0.5,  // y position in meters (left/right)
            Rotation2d.fromDegrees(90)  // rotation in degrees (0 = forward, 90 = left, -90 = right, 180 = backward)
    );
    // ====================================================

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

        double ACHIEVABLE_MAX_TICKS_PER_SECOND = frontLeft.ACHIEVABLE_MAX_TICKS_PER_SECOND;
        ACHIEVABLE_MAX_DISTANCE_PER_SECOND = ACHIEVABLE_MAX_TICKS_PER_SECOND * DriveConstants.DISTANCE_PER_PULSE;

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

    private void initWebCamAprilTag() {
        AprilTagLibrary apriltagLib;
        VisionPortal visionPortal;
        WebcamName apriltagCam;

        try {
            apriltagCam = hardwareMap.get(WebcamName.class, "Webcam 1");
        } catch (Exception e) {
            webcamAprilTag = null;
            telemetry.addLine(" Lost Webcam 1 /n");
            telemetry.update();
            return;
        }

        AprilTagProcessor.Builder myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myAprilTagProcessorBuilder.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary());

        myAprilTagProcessorBuilder.setDrawTagID(true);
        myAprilTagProcessorBuilder.setDrawTagOutline(true);
        myAprilTagProcessorBuilder.setDrawAxes(true);
        myAprilTagProcessorBuilder.setDrawCubeProjection(true);
        myAprilTagProcessorBuilder.setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES);
        webcamAprilTag = myAprilTagProcessorBuilder.build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(apriltagCam);
        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.addProcessor(webcamAprilTag);
        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(webcamAprilTag, true);
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

    @Override
    public void initialize() {
        initDriveWheels();
        initGyro();
        initLimelight();
        initWebCamAprilTag();

        // Initialize the drive subsystem with starting pose at origin
        mecanumDriveSubsystem = new MecanumDriveSubsystem(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                gyro,
                webcamAprilTag,
                limelightApriltag,
                new Pose2d(),  // Start at origin (0, 0, 0)
                telemetry,
                hardwareMap
        );

        mecanumDriveSubsystem.enableDrive();

        // Configure trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                DriveConstants.TRAJECTORY_MAX_VELOCITY,
                DriveConstants.MAX_ACCELERATION,
                false)  // Set to true if you want to allow reverse
                .setKinematics(DriveConstants.kinematicsWPI);

        // Create PID controllers for x, y, and theta (rotation)
        // These values may need tuning for your robot
        PIDController xController = new PIDController(0.3, 0, 0.001);
        PIDController yController = new PIDController(0.2, 0, 0.001);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                0.1, 0, 0.005,
                new TrapezoidProfile.Constraints(0.5, 0.5)
        );

        // Schedule the command to move to target pose
        schedule(new MecanumDynamicControllerCommand(
                mecanumDriveSubsystem::getCurrentEstimatedPose,  // Current pose supplier
                TARGET_POSE,  // Target pose
                trajectoryConfig,  // Trajectory configuration
                mecanumDriveSubsystem::getCurrentEstimatedPose,  // Pose supplier for controller
                mecanumDriveSubsystem.getKinematics(),  // Kinematics
                xController,  // X PID controller
                yController,  // Y PID controller
                thetaController,  // Theta (rotation) PID controller
                ACHIEVABLE_MAX_DISTANCE_PER_SECOND,  // Max wheel velocity
                mecanumDriveSubsystem::driveBySpeedEvent,  // Output wheel speeds
                mecanumDriveSubsystem  // Required subsystem
        ).whenFinished(mecanumDriveSubsystem::stop));  // Stop when finished
    }

    ////// DO NOT MODIFY THIS FUNCTION UNLESS YOU GET CONFIRMED!!!
    private int FrameCounter = 0;
    static final double MIN_TASK_RUN_PERIOD = 100;

    @Override
    public void runOpMode() {
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        DashServer.Init();
        boolean connected = false;
        do {
            connected = DashServer.Connect();
            connected |= DashServer.AddData("time", FrameCounter);
            sleep(1);
        } while (!connected);

        initialize();
        waitForStart();
        double taskRunTime = 0;

        while (!isStopRequested() && opModeIsActive()) {
            DashServer.AddData("tskTime", taskRunTime);
            double currentTime = (double) System.nanoTime() / 1E9;
            DashServer.AddData("OSTime", currentTime);
            run();
            DashServer.AddData("time", FrameCounter++);
            DashServer.AddData("busVoltage",
                    controlHub.getInputVoltage(VoltageUnit.VOLTS));
            DashServer.DashData();

            taskRunTime = (double) System.nanoTime() / 1E9 - currentTime;
            long sleepTime = (long) (MIN_TASK_RUN_PERIOD - taskRunTime * 1000);
            if (sleepTime > 0)
                sleep(sleepTime);
        }
        reset();
        if (limelightApriltag != null) limelightApriltag.stop();
        DashServer.AddData("time", FrameCounter++);
        DashServer.AddData("OSTime", (double) System.nanoTime() / 1E9);
        DashServer.DashData();
        DashServer.Close();
    }
}
