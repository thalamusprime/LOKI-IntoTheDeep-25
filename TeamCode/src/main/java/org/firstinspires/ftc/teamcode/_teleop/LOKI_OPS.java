package org.firstinspires.ftc.teamcode._teleop;

import static org.firstinspires.ftc.teamcode.ftc6205.constants.AUTOConstants.touch_duration;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.ftc6205.constants.AUTOConstants;
import org.firstinspires.ftc.teamcode.ftc6205.metrics.DSTelemetry;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Arm;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Drivetrain;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Claw;
import org.firstinspires.ftc.teamcode.ftc6205.motors.LiftArm;
import org.firstinspires.ftc.teamcode.ftc6205.motors.LiftWrist;
import org.firstinspires.ftc.teamcode.ftc6205.pidcontrol.TrueNorth;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.Encoders;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.Touch;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

import com.arcrobotics.ftclib.controller.PIDController;

@TeleOp(name = "*: LOKI", group = "6205")
public class LOKI_OPS extends LinearOpMode {
    //////////////////////////////////////////////////////////// DRIVETRAIN
    DSTelemetry dsTelemetry;
    Encoders encoders;
    Drivetrain drivetrain;
    Claw claw; //claw
    Arm arm;
    Touch touchArm;

    ///////////////////////////////////////////////////////////
    PIDController armController;
    LiftArm liftArm;
    LiftWrist liftWrist;

    // SERVOS
    Servo pixelThumb; //purp-drop

    // SENSORS
    DcMotor encoderLeft, encoderBack, encoderRight;
    double encLeftValue, encBackValue, encRightValue;
    // DISTANCE
    DistanceSensor distFront, distBack;

    // IMU
    IMU imu;
    double refHeading, botHeading, pidOutput;
    double y, x, rz, rotX, rotY;

    // VisionPortal
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    private OpenCvCamera controlHubCam;

    // CONTROLLERS
    //private PIDController armController;

    @Override
    public void runOpMode() throws InterruptedException {
        // FtcDashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
        // DSTelemetry
        dsTelemetry = new DSTelemetry();

        // Deadwheel encoders, declare BEFORE drive motor declaration
        encoders = new Encoders();
        encoders.init(hardwareMap);

        // Drivetrain motors
        drivetrain = new Drivetrain();
        drivetrain.init(hardwareMap);

        // Claw
        claw = new Claw();
        claw.init(hardwareMap);

        // Arm
        arm = new Arm();
        arm.initArm(hardwareMap);

        // Touch sensor on wrist
        touchArm = new Touch();
        touchArm.init(hardwareMap);

        // Other sensors/motors
//        initLiftArm();
//        initLiftWrist();

        initDevices();

        // Pause until "y".  Close program if "Stop".
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

            // ROTATE ARM
            if (!gamepad1.left_bumper && !gamepad1.right_bumper && gamepad1.dpad_down) {
                arm.rot(0.5);
            }  else if (!gamepad1.left_bumper && !gamepad1.right_bumper && gamepad1.dpad_up) {
                arm.rot(-0.5);
            }  else if (gamepad1.a) {
                arm.rotArmUntil(arm.armFloor);
            }  else if (gamepad1.b) {
                arm.rotArmUntil(arm.armLowGoal);
            }  else {
                arm.rot(0);
            }
            // Rumble gamepad1 when touchArm on floor
            if (touchArm.isPressed()) {
                gamepad1.rumble(touch_duration);
            }

            // TODO: liftArm - Manual
            if (gamepad1.right_bumper && gamepad1.dpad_left) {
                liftArm.drive(0.2);
            }  else if (gamepad1.right_bumper && gamepad1.dpad_right) {
                liftArm.drive(-0.2);
            }  else if (gamepad1.left_bumper && gamepad1.dpad_left) {
                liftArm.runArmUntil(liftArm.liftArmHigh);
            }  else if (gamepad1.left_bumper && gamepad1.dpad_right) {
                liftArm.runArmUntil(liftArm.liftArmLow);
            }  else {
                liftArm.drive(0);
            }

            // TODO: liftWrist - Manual
            if (gamepad1.right_bumper && gamepad1.dpad_down) {
                liftWrist.drive(-0.75);
            }  else if (gamepad1.right_bumper && gamepad1.dpad_up) {
                liftWrist.drive(0.75);
            }  else if (gamepad1.left_bumper && gamepad1.dpad_up) {
                liftWrist.runArmUntil(liftWrist.liftWristHigh);
            }  else if (gamepad1.left_bumper && gamepad1.dpad_down) {
                liftWrist.runArmUntil(liftWrist.liftWristLow);
            } else {
                liftWrist.drive(0);
            }

            // RUN SUBSYSTEMS
            touchArm.isPressed();   // arm touching floor?
            encoders.runEncoders(); // encoders
            this.runMain();         // drivetrain
            dsTelemetry.sendTelemetry(telemetry, encoders, touchArm);    //telemetry
        }
    }

    //TODO /////////////////////////////////////////////////////////////// CUSTOM PRIVATE FUNCTIONS
    private void initDevices() throws InterruptedException {
        // SENSORS
        initIMU();
        initDistSensors();
        //initTouchSensors();

        initLiftArm();
        initLiftWrist();

        initAprilTag();
        initVision();
    }

    private void runMain() throws InterruptedException {
        // DRIVETRAIN
        // Get yaw, reset in match optional
        if (gamepad1.start) {
            refHeading = 0;
            botHeading = 0;
            imu.resetYaw();
        }
        if (gamepad1.options) {
            arm.initArm(hardwareMap);
            encoders.init(hardwareMap);
            drivetrain.init(hardwareMap);
        }

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

        // Denominator (absolute value) or 1, at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rz), 1);
        double frontLeftPower = (rotY + rotX + rz) / denominator;
        double backLeftPower = (rotY - rotX + rz) / denominator;
        double frontRightPower = (rotY - rotX - rz) / denominator;
        double backRightPower = (rotY + rotX - rz) / denominator;

        // Trigger gain
        frontLeftPower = frontLeftPower * (0.3 + 0.7 * gamepad1.right_trigger);
        backLeftPower = backLeftPower * (0.3 + 0.7 * gamepad1.right_trigger);
        frontRightPower = frontRightPower * (0.3 + 0.7 * gamepad1.right_trigger);
        backRightPower = backRightPower * (0.3 + 0.7 * gamepad1.right_trigger);

        // Set motor power
        drivetrain.frontLeftDriveMotor.setPower(frontLeftPower);
        drivetrain.backLeftDriveMotor.setPower(backLeftPower);
        drivetrain.frontRightDriveMotor.setPower(frontRightPower);
        drivetrain.backRightDriveMotor.setPower(backRightPower);
    }

    private void initLiftArm() throws InterruptedException {
        liftArm = new LiftArm();
        liftArm.initLiftArm(hardwareMap);
        armController = new PIDController(liftArm.p,liftArm.i,liftArm.d);
    }

    private void initLiftWrist() throws InterruptedException {
        liftWrist = new LiftWrist();
        liftWrist.initLiftWrist(hardwareMap);
        armController = new PIDController(liftArm.p,liftArm.i,liftArm.d);
    }

    private void initDistSensors() throws InterruptedException {
        distFront = hardwareMap.get(DistanceSensor.class, "distFront");
        distBack = hardwareMap.get(DistanceSensor.class, "distBack");
    }

    private void initIMU() throws InterruptedException {
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
