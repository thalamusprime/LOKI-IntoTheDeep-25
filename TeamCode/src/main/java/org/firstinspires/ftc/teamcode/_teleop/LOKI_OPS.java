package org.firstinspires.ftc.teamcode._teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ftc6205.globals.RobotConfiguration;
import org.firstinspires.ftc.teamcode.ftc6205.globals.TwistPIDConstants;
import org.firstinspires.ftc.teamcode.ftc6205.logging.DSTelemetry;
import org.firstinspires.ftc.teamcode.ftc6205.motors.ShortArm;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Drivetrain;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Claw;
import org.firstinspires.ftc.teamcode.ftc6205.motors.LongArm;
import org.firstinspires.ftc.teamcode.ftc6205.motors.ForeArm;
import org.firstinspires.ftc.teamcode.ftc6205.controllers.TrueNorth;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Shoulder;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Wrist;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.ScarViz;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.DeadWheels;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.DistSensors;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.FieldSense;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.IMU;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

import kotlin.jvm.internal.Ref;

@TeleOp(name = "*: LOKI", group = "6205")
public class LOKI_OPS extends LinearOpMode {
    // Subsystems
    FtcDashboard dashboard;
    DSTelemetry dsTelemetry;
    DeadWheels deadWheels;
    Drivetrain drivetrain;
    Claw claw;
    Wrist wrist;
    Shoulder shoulder;
    FieldSense fieldSense;
    //ShortArm shortArm;
    LongArm longArm;
    ForeArm foreArm;
    IMU navx;
    DistSensors distSensors;
    TrueNorth trueNorth;
    PIDController pidController;

    // to be classed
    ScarViz scarViz;
    OpenCvCamera controlHubCam;
    AprilTagProcessor aprilTagProcessor;
    VisionPortal visionPortal;

    // Global variables
    double refHeading, botHeading, pidOutput;
    double y, x, rz, rotX, rotY;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems
        this.initSensors();
        this.initActuators();
        this.initControllers();
        this.initMonitoring();

        waitForStart();  // Pause until "PLAY"
        if (isStopRequested()) return; // Close program if "STOP"

        // TELEOP LOOP
        while (opModeIsActive()) {
            // RUN SUBSYSTEMS
            //fieldSense.check(gamepad1);   // CHECK FIELD SENSOR
            claw.grab(gamepad1);            // GRAB CLAW
            shoulder.shrug(gamepad1);       // SHRUG SHOULDER
            wrist.revolve(gamepad1);        // ROTATE SHORT-ARM
            foreArm.reach(gamepad1);        // REACH FORE-ARM
            longArm.raise(gamepad1);        // RAISE LONG-ARM

            deadWheels.runEncoders();       // READ DRIVE-ENCODERS
            this.resetCheck();              // RESET TrueNorth | Encoders
            this.runTrueNorth();            // PID Straight
            this.runFieldCentric();         //
            drivetrain.runBot(gamepad1, rotY, rotX, rz);
            dsTelemetry.sendTelemetry(
                    telemetry,
                    deadWheels,
                    wrist,
                    shoulder);
        }
    }

    private void initMonitoring() {
        // FtcDashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(scarViz.controlHubCam, 30);
        dsTelemetry = new DSTelemetry();
    }

    private void initSensors() {
        navx = new IMU(hardwareMap);
        fieldSense = new FieldSense(hardwareMap);
        distSensors = new DistSensors(hardwareMap);
        scarViz = new ScarViz(hardwareMap);
    }

    private void initActuators() {
        deadWheels = new DeadWheels(hardwareMap); // Deadwheel encoders, declare b4 motors.
        drivetrain = new Drivetrain(hardwareMap);       // Drivetrain motors
        claw = new Claw(RobotConfiguration.claw_name, hardwareMap, gamepad1, telemetry);
        wrist = new Wrist(RobotConfiguration.wrist_name, hardwareMap, gamepad1, telemetry);
        shoulder = new Shoulder(hardwareMap);
        //shortArm = new ShortArm(hardwareMap);
        //longArm = new LongArm(hardwareMap);
        //foreArm = new ForeArm(hardwareMap);
    }

    private void initControllers() {
        pidController = new PIDController(  TwistPIDConstants.Kp,
                                            TwistPIDConstants.Ki,
                                            TwistPIDConstants.Kd );
        trueNorth = new TrueNorth();
    }

    private void resetCheck() {
        if (gamepad1.share) {
            //claw.initClaw();
            //wrist.initWrist();
            //foreArm.initForeArm();
            //arm.initArm();
            //lift.initLift();
            deadWheels.initEncoders(hardwareMap);
            drivetrain.initDriveMotors(hardwareMap);
        }
        if (gamepad1.options) {
            refHeading = 0;
            botHeading = 0;
            navx.resetYaw();
            //otos.resetTracking();
        }
        if (gamepad1.ps) {
            wrist.initWrist();
            shoulder.initShoulder(hardwareMap);
        }
    }

    private void runTrueNorth(){
        y = gamepad1.left_stick_y;  //
        x = -gamepad1.left_stick_x; //

        if (Math.abs(gamepad1.right_stick_x) > 0.03) { // Yaw threshold
            rz = -gamepad1.right_stick_x;
            refHeading = navx.getYawInRadians();
        } else {
            rz = 0;
            // PID Controller
            TrueNorth trueNorth = new TrueNorth();
            pidOutput = trueNorth.twistControl(refHeading, botHeading);

            if (Math.abs(pidOutput) > 0.03) {
                rz = pidOutput;
            }
        }
    }

    private void runFieldCentric() {
        // Get heading
        botHeading = navx.getYawInRadians(); // bot

        // Robot-centric, Field-centric
        if (gamepad1.left_bumper) {
            rotX = x;
            rotY = y;
        } else {
            rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        }
        rotX = rotX * 1.1;  // Counteract imperfect strafing
    }

} // end LOKI_OPS class
