package org.firstinspires.ftc.teamcode._auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ftc6205.controllers.TrueNorth;
import org.firstinspires.ftc.teamcode.ftc6205.logging.DSTelemetry;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Drivetrain;
import org.firstinspires.ftc.teamcode.ftc6205.motors.LongArm;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.DriveEncoders;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.FieldSense;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.IMU;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

@Config
@Autonomous(name = "AUTO-LOKI", group = "6205")
public class LOKI_AUTO extends LinearOpMode {
    // Subsystems
    FtcDashboard dashboard;
    DSTelemetry dsTelemetry;
    DriveEncoders driveEncoders;
    Drivetrain drivetrain;
    PIDController pidController;
    FieldSense fieldSense;
    IMU navx;
    public static double p = 0.9;
    public static double i = 0;
    public static double d = 0.05;
    public static double f = 0.1;
//    public static double p = DrivePIDConstants.Kp;
//    public static double i = DrivePIDConstants.Ki;
//    public static double d = DrivePIDConstants.Kd;
//    public static double f = DrivePIDConstants.Kf;
    public static int targetInches = 24;
    //DistSensors distSensors;

    /////////////////////////////////////////////////////////// to be classed
    Servo wrist;
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

        waitForStart();                     // Pause until "PLAY".  Close program if "Stop".
        if (isStopRequested()) return;

        /////////////////////////////////////////////////////////////// TELEOP LOOP
        while (opModeIsActive()) {
            // RUN SUBSYSTEMS
            fieldSense.check(gamepad1);     // CHECK FIELD SENSOR
            driveEncoders.runEncoders();    // READ DRIVEENCODERS
            this.resetCheck();              // RESET TrueNorth | Encoders
            this.runTrueNorth();            // PID Straight
            this.runFieldCentric();         //
            drivetrain.runBot(gamepad1, rotY, rotX, rz);

            if (gamepad1.square) {
                //this.runForwardUntil(targetInches);
                this.runStrafeUntil(targetInches);
            }
//            dsTelemetry.sendTelemetry(  telemetry,
//                                        driveEncoders,
//                                        fieldSense
//            );
            telemetry.update();
        }
    }

    // TODO create Monitor
    private void initMonitoring() {
        // FtcDashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
        //dsTelemetry = new DSTelemetry();
    }

    private void initActuators() {
        driveEncoders = new DriveEncoders(hardwareMap); // Encoders before Drivetrain
        drivetrain = new Drivetrain(hardwareMap);       // Drivetrain
    }

    private void initSensors() throws InterruptedException {
        pidController = new PIDController(p,i,d);
        navx = new IMU(hardwareMap);
        navx.initIMU(hardwareMap);
        fieldSense = new FieldSense(hardwareMap);
    }
    private void resetCheck() {
        // Get yaw, reset in match optional
        if (gamepad1.start) {
            refHeading = 0;
            botHeading = 0;
            navx.resetYaw();
        }
        if (gamepad1.options) {
            driveEncoders.initEncoders(hardwareMap);
            drivetrain.initDriveMotors(hardwareMap);
        }
    }
    private void runTrueNorth() {
        // Get XY: gamepad1
        y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        x = -gamepad1.left_stick_x; //-

        // Get Z: gamepad1
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
        botHeading = navx.getYawInDegrees();//.getYaw(AngleUnit.RADIANS); // bot

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

    public void runForwardUntil(double forwardTarget) {
        double power = 0;
        double forwardValue = (driveEncoders.encLeftValue +
                driveEncoders.encRightValue) / 2;
        if (Math.abs(forwardTarget - forwardValue) >= 1) {
            //TrueNorth pidControl = new TrueNorth();
            double current_position = forwardValue;
            double pid = pidController.calculate(current_position, forwardTarget);
            double ff = Math.cos(Math.toRadians(forwardTarget / LongArm.ticks_in_degree)) *
                    f;
            power = pid * ff;
            drivetrain.autoForward(power);//,
//                    driveEncoders,
//                    driveEncoders.encLeftValue,
//                    driveEncoders.encRightValue);

        } else {
            drivetrain.autoForward(0);
            ;
        }
        telemetry.addLine(String.format(
                "Target: %5.2f",
                forwardTarget
        ));
        telemetry.addLine(String.format(
                "Value %5.2f",
                forwardValue // 0.0075
        ));
        telemetry.addLine(String.format(
                "Power %5.2f",
                power
        ));
        telemetry.addLine(String.format(
                "ENCODER L|B|R %5.2f %5.2f %5.2f",
                driveEncoders.encLeftValue, // 0.0075
                driveEncoders.encBackValue,
                driveEncoders.encRightValue
        ));
    }
    public void runStrafeUntil(double sideTarget) {
        double power = 0;
        double forwardTicks = 0;
        double sideValue = (driveEncoders.encBackValue) / 2;
        if (Math.abs(sideTarget - sideValue) >= 1) {
            //TrueNorth pidControl = new TrueNorth();
            double current_position = sideValue;
            double pid = pidController.calculate(current_position, sideTarget);
            double ff = Math.cos(Math.toRadians(sideTarget / LongArm.ticks_in_degree)) *
                    f;
            power = pid * ff;
            drivetrain.autoStrafe(power);

        } else {
            drivetrain.autoStrafe(0);
            ;
        }
        telemetry.addLine(String.format(
                "Target: %5.2f",
                sideTarget
        ));
        telemetry.addLine(String.format(
                "Value %5.2f",
                sideValue // 0.0075
        ));
        telemetry.addLine(String.format(
                "Power %5.2f",
                power
        ));
        telemetry.addLine(String.format(
                "ENCODER L|B|R %5.2f %5.2f %5.2f",
                driveEncoders.encLeftValue, // 0.0075
                driveEncoders.encBackValue,
                driveEncoders.encRightValue
        ));
        //telemetry.update();
    }

}
