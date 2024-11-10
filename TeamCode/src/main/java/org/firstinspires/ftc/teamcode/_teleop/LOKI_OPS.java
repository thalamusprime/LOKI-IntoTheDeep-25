package org.firstinspires.ftc.teamcode._teleop;

import static org.firstinspires.ftc.teamcode.ftc6205.constants.AUTOConstants.touch_duration;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.ftc6205.constants.AUTOConstants;
import org.firstinspires.ftc.teamcode.ftc6205.metrics.DSTelemetry;
import org.firstinspires.ftc.teamcode.ftc6205.motors.ShortArm;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Drivetrain;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Claw;
import org.firstinspires.ftc.teamcode.ftc6205.motors.LongArm;
import org.firstinspires.ftc.teamcode.ftc6205.motors.ForeArm;
import org.firstinspires.ftc.teamcode.ftc6205.pidcontrol.TrueNorth;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.DriveEncoders;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.FieldSense;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp(name = "*: LOKI", group = "6205")
public class LOKI_OPS extends LinearOpMode {
    // Subsystems
    FtcDashboard dashboard;
    DSTelemetry dsTelemetry;
    DriveEncoders driveEncoders;
    Drivetrain drivetrain;
    Claw claw;
    FieldSense fieldSense;
    ShortArm shortArm;
    LongArm longArm;
    ForeArm foreArm;

    ///////////////////////////////////////////////////////////
    DistanceSensor distFront, distBack;
    Servo pixelThumb;
    IMU imu;
    OpenCvCamera controlHubCam;
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;

    // Global variables
    double refHeading, botHeading, pidOutput;
    double y, x, rz, rotX, rotY;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems
        this.initMonitoring();
        this.initActuators();
        this.initSensors();

        // Pause until "PLAY".  Close program if "Stop".
        waitForStart();
        if (isStopRequested()) return;

        /////////////////////////////////////////////////////////////// TELEOP LOOP
        while (opModeIsActive()) {
            // OPEN/CLOSE claw
            if (gamepad1.right_bumper) {
                claw.setPosition(AUTOConstants.claw_release);
            } else {
                claw.setPosition(AUTOConstants.claw_pinch);
            }

            // Rumble gamepad1 when touchArm on floor
            if (fieldSense.isPressed()) {
                gamepad1.rumble(touch_duration);
            }

            // RUN SUBSYSTEMS
            shortArm.rotate(gamepad1); // ROTATE SHORTARM
            longArm.raise(gamepad1);        // RAISE LONGARM
            foreArm.reach(gamepad1);        // REACH FOREARM reach
            fieldSense.isPressed();         // arm touching floor?
            driveEncoders.runEncoders();    // drive encoders
            this.runMain();         // todo imu.runTrueNorth | this.runResetEncoders | imu.runIMU | drivetrain.runBot
            dsTelemetry.sendTelemetry(telemetry, driveEncoders, fieldSense);    //telemetry
        }
    }

    //TODO /////////////////////////////////////////////////////////////// CUSTOM PRIVATE FUNCTIONS

    private void initMonitoring() {
        // FtcDashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
        dsTelemetry = new DSTelemetry();
    }

    private void initActuators() {
        // Deadwheel encoders, declare BEFORE drive motor declaration or drive motors won't reg.
        driveEncoders = new DriveEncoders();
        driveEncoders.init(hardwareMap);
        // Drivetrain motors
        drivetrain = new Drivetrain();
        drivetrain.init(hardwareMap);
        // Claw
        claw = new Claw();
        claw.init(hardwareMap);
        // ShortArm
        shortArm = new ShortArm();
        shortArm.initArm(hardwareMap);
        // LongArm
        longArm = new LongArm();
        longArm.initLongArm(hardwareMap);
        // ForeArm
        foreArm = new ForeArm();
        foreArm.initForeArm(hardwareMap);

        // Touch sensor on wrist
        fieldSense = new FieldSense();
        fieldSense.init(hardwareMap);
    }

    private void initSensors() throws InterruptedException {
        // SENSORS
        initIMU();
        initDistSensors();
        initAprilTag();
        initVision();
    }

    private void runMain() {
        // todo reset TrueNorth | motor encoders
        // Get yaw, reset in match optional
        if (gamepad1.start) {
            refHeading = 0;
            botHeading = 0;
            imu.resetYaw();
        }
        if (gamepad1.options) {
            shortArm.initArm(hardwareMap);
            driveEncoders.init(hardwareMap);
            drivetrain.init(hardwareMap);
        }

        // todo: split IMU field-centric/robot-centric
        // Get XY: gamepad1
        y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        x = -gamepad1.left_stick_x; //-

        // Get Z: gamepad1
        if (Math.abs(gamepad1.right_stick_x) > 0.03) { // Yaw threshold
            rz = -gamepad1.right_stick_x;
            refHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); // ref
        } else {
            rz = 0;
            // PID Controller
            TrueNorth pidControl = new TrueNorth();
            pidOutput = pidControl.PIDControl(refHeading, botHeading);
            if (Math.abs(pidOutput) > 0.03) {
                rz = pidOutput;
            }
        }
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); // bot

        if (gamepad1.left_bumper) {
            rotX = x;
            rotY = y;
        } else {
            // Rotate the movement direction counter to the bot's rotation
            rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        }
        rotX = rotX * 1.1;  // Counteract imperfect strafing

        drivetrain.runBot(gamepad1, rotY, rotX, rz);

    }

    private void initDistSensors() {
        distFront = hardwareMap.get(DistanceSensor.class, "distFront");
        distBack = hardwareMap.get(DistanceSensor.class, "distBack");
    }

    private void initIMU() {
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT)); //FORWARD
        imu.initialize(parameters);
    }

//    private void initServos() throws InterruptedException {
//        //pixelThumb = hardwareMap.servo.get("pixelThumb");
//        //pixelThumb.setDirection(Servo.Direction.REVERSE);
//        //pixelThumb.setPosition(0.5);
//    }

    private void initAprilTag() throws InterruptedException {
        // Tag Processing
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
    }

    private void initVision() throws InterruptedException {
        // VisionPortal
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1920, 1080))
                .build();
    }

}
