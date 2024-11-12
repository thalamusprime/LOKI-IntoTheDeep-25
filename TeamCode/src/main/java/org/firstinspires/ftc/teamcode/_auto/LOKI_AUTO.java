package org.firstinspires.ftc.teamcode._auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ftc6205.constants.DrivePIDConstants;
import org.firstinspires.ftc.teamcode.ftc6205.controllers.TrueNorth;
import org.firstinspires.ftc.teamcode.ftc6205.logging.DSTelemetry;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Claw;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Drivetrain;
import org.firstinspires.ftc.teamcode.ftc6205.motors.ForeArm;
import org.firstinspires.ftc.teamcode.ftc6205.motors.LongArm;
import org.firstinspires.ftc.teamcode.ftc6205.motors.ShortArm;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.DistSensors;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.DriveEncoders;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.FieldSense;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.MicroNavX;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "AUTO-LOKI", group = "6205")
public class LOKI_AUTO extends LinearOpMode {
    // Subsystems
    //FtcDashboard dashboard;
    //DSTelemetry dsTelemetry;
    DriveEncoders driveEncoders;
    Drivetrain drivetrain;
    PIDController pidController;
    MicroNavX navx;
    public static double p = 1;
    public static double i = 0;
    public static double d = 0.01;
    public static double f = 0.005;
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
//        this.initMonitoring();
        this.initActuators();
        this.initSensors();

        waitForStart();                     // Pause until "PLAY".  Close program if "Stop".
        if (isStopRequested()) return;

        /////////////////////////////////////////////////////////////// TELEOP LOOP
        while (opModeIsActive()) {
            // RUN SUBSYSTEMS
//            fieldSense.check(gamepad1);     // CHECK FIELD SENSOR
//            claw.grab(gamepad1);            // GRAB CLAW
//            shortArm.rotate(gamepad1);      // ROTATE SHORTARM
//            longArm.raise(gamepad1);        // RAISE LONGARM
//            foreArm.reach(gamepad1);        // REACH FOREARM reach
            driveEncoders.runEncoders();    // READ DRIVEENCODERS
//            this.resetCheck();              // RESET TrueNorth | Encoders
            this.runTrueNorth();            // PID Straight
            this.runFieldCentric();         //
            drivetrain.runBot(gamepad1, rotY, rotX, rz);
//            dsTelemetry.sendTelemetry(  telemetry,
//                                        driveEncoders,
//                                        fieldSense
//            );
        }
    }

    // TODO create Monitor
//    private void initMonitoring() {
//        // FtcDashboard
//        dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
//        dsTelemetry = new DSTelemetry();
//    }

    private void initActuators() {
        // Deadwheel encoders, declare BEFORE drive motor declaration or drive motors won't reg.
        driveEncoders = new DriveEncoders();
        driveEncoders.init(hardwareMap);
        // Drivetrain motors
        drivetrain = new Drivetrain();
        drivetrain.initDriveMotors(hardwareMap);
    }
    private void initSensors() throws InterruptedException {
        // microNavX
        navx = new MicroNavX();
        navx.initIMU(hardwareMap);
        pidController = new PIDController(p,i,d);
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
            pidOutput = pidControl.PIDControl(refHeading, botHeading);
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

    public void runForwardUntil(int forwardTarget) {
        int forwardValue = (driveEncoders.encoderLeft.getCurrentPosition() +
                driveEncoders.encoderLeft.getCurrentPosition() ) / 2;
        if ( Math.abs(forwardTarget - forwardValue) >= 2 ) {
            TrueNorth pidControl = new TrueNorth();
            int current_position = forwardValue;
            double pid = pidController.calculate(current_position, forwardTarget);
            double ff = Math.cos(Math.toRadians(forwardTarget / LongArm.ticks_in_degree)) *
                    f;
            double power = pid * ff;
            drivetrain.autoForward(power);
        } else {
            drivetrain.autoForward(0);;
        }
    }

}
