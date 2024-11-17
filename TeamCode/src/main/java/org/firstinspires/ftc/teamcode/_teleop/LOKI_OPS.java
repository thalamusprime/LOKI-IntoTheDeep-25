package org.firstinspires.ftc.teamcode._teleop;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.ftc6205.logging.DSTelemetry;
import org.firstinspires.ftc.teamcode.ftc6205.motors.ShortArm;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Drivetrain;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Claw;
import org.firstinspires.ftc.teamcode.ftc6205.motors.LongArm;
import org.firstinspires.ftc.teamcode.ftc6205.motors.ForeArm;
import org.firstinspires.ftc.teamcode.ftc6205.controllers.TrueNorth;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Wrist;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.DriveEncoders;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.DistSensors;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.FieldSense;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.MicroNavX;
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
    MicroNavX navx;
    DistSensors distSensors;

    /////////////////////////////////////////////////////////// to be classed
    Wrist wrist;
    OpenCvCamera controlHubCam;
    AprilTagProcessor aprilTagProcessor;
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

        waitForStart();
        // Pause until "PLAY".  Close program if "Stop".
        if (isStopRequested()) return;

        // TELEOP LOOP
        while (opModeIsActive()) {
            // RUN SUBSYSTEMS
            fieldSense.check(gamepad1);     // CHECK FIELD SENSOR
            claw.grab(gamepad1);            // GRAB CLAW
            shortArm.rotate(gamepad1);      // ROTATE SHORTARM
            longArm.raise(gamepad1);        // RAISE LONGARM
            foreArm.reach(gamepad1);        // REACH FOREARM reach
            driveEncoders.runEncoders();    // READ DRIVEENCODERS
            this.resetCheck();              // RESET TrueNorth | Encoders
            this.runTrueNorth();            // PID Straight
            this.runFieldCentric();         //
            drivetrain.runBot(gamepad1, rotY, rotX, rz);

            dsTelemetry.sendTelemetry(  telemetry,
                                        driveEncoders,
                                        fieldSense
            );
        }
    }

    // TODO create Monitor
    private void initMonitoring() {
        // FtcDashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
        dsTelemetry = new DSTelemetry();
    }

    private void initSensors() throws InterruptedException {
        navx = new MicroNavX(hardwareMap);
        fieldSense = new FieldSense(hardwareMap);
        distSensors = new DistSensors(hardwareMap);

        initAprilTag();
        initVision();
    }

    private void initActuators() {
        driveEncoders = new DriveEncoders(hardwareMap); // Deadwheel encoders, declare b4 motors.
        drivetrain = new Drivetrain(hardwareMap);       // Drivetrain motors
        claw = new Claw(hardwareMap);
        shortArm = new ShortArm(hardwareMap);
        longArm = new LongArm(hardwareMap);
        foreArm = new ForeArm(hardwareMap);
        //wrist = new Wrist(hardwareMap);
    }

    private void resetCheck() {
        // Get yaw, reset in match optional
        if (gamepad1.start) {
            refHeading = 0;
            botHeading = 0;
            navx.resetYaw();
        }
        if (gamepad1.options) {
            shortArm.initArm(hardwareMap);
            driveEncoders.initEncoders(hardwareMap);
            drivetrain.initDriveMotors(hardwareMap);
        }
    }

    private void runTrueNorth() {
        // Get XY: gamepad1
        y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        x = -gamepad1.left_stick_x; //-

        // Get  Z: gamepad1
        if (Math.abs(gamepad1.right_stick_x) > 0.03) { // Yaw threshold
            rz = -gamepad1.right_stick_x;
            refHeading = navx.getYawInDegrees();//.getYaw(AngleUnit.RADIANS); // ref
        } else {
            rz = 0;
            // PID Controller
            TrueNorth pidControl = new TrueNorth();
            pidOutput = pidControl.TwistControl(refHeading, botHeading);
            if (Math.abs(pidOutput) > 0.02) {
                rz = pidOutput;
            }
        }
    }

    private void runFieldCentric() {
        botHeading = navx.getYawInDegrees();

        // Field-centric drive, Robot-centric default
        if (gamepad1.left_bumper) {
            rotX = x;
            rotY = y;
        } else {
            // Rotate the movement direction counter to the bot's rotation
            rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        }
        rotX = rotX * 1.1;  // Counteract imperfect strafing
    }


    //todo create WRIST


    // todo create AprilTag
    private void initAprilTag() {
        // Tag Processing
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
    }

    // todo create VISION PORTAL
    private void initVision() {
        // VisionPortal
        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1920, 1080))
                .build();
    }

}
