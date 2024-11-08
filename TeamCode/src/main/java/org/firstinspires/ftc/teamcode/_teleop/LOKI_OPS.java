package org.firstinspires.ftc.teamcode._teleop;

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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ftc6205.constants.AUTOConstants;
import org.firstinspires.ftc.teamcode.ftc6205.metrics.DSTelemetry;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Arm;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Drivetrain;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Claw;
import org.firstinspires.ftc.teamcode.ftc6205.pidcontrol.TrueNorth;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.Encoders;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;

@TeleOp(name = "*: LOKI", group = "6205")
public class LOKI_OPS extends LinearOpMode {
    //////////////////////////////////////////////////////////// DRIVETRAIN
    DSTelemetry dsTelemetry;
    Encoders encoders;
    Drivetrain drivetrain;
    Claw claw; //claw
    Arm arm;

    ///////////////////////////////////////////////////////////
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
    VisionPortal tfPortal;
    //TfodProcessor tfod;
    private OpenCvCamera controlHubCam;

    // CONTROLLERS
    //private PIDController armController;

    @Override
    public void runOpMode() throws InterruptedException {
        // FtcDashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        // Deadwheel encoders, declare BEFORE drive motor declaration
        encoders = new Encoders();
        encoders.init(hardwareMap);

        // Drivetrain motors
        drivetrain = new Drivetrain();
        drivetrain.init(hardwareMap);

        // Claw
        claw = new Claw();
        claw.init(hardwareMap);

        // Claw
        arm = new Arm();
        arm.initServo(hardwareMap);

        // Other sensors/motors
        initDevices();

        // DSTelemetry
        dsTelemetry = new DSTelemetry();

        // Pause until "Play".  Close program if "Stop".
        waitForStart();
        if (isStopRequested()) return;

        /////////////////////////////////////////////////////////////// TELEOP LOOP
        while (opModeIsActive()) {
            // OPEN/CLOSE claw
            if (gamepad1.x) {
                claw.setPosition(AUTOConstants.claw_pinch);
            } else if (gamepad1.y) {
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
                arm.rotArmUntil(arm.pixelArmLow);
            }  else if (gamepad1.b) {
                arm.rotArmUntil(arm.pixelArmHigh);
            }  else if (gamepad1.right_bumper) {
                arm.rotArmUntil(0);
            }  else {
                arm.rot(0);
            }

            // TODO: liftArm - Manual
//            if (gamepad1.right_bumper && gamepad1.dpad_left) {
//                liftArm.drive(0.2);
//            }  else if (gamepad1.right_bumper && gamepad1.dpad_right) {
//                liftArm.drive(-0.2);
//            }  else if (gamepad1.left_bumper && gamepad1.dpad_left) {
//                liftArm.runArmUntil(liftArm.liftArmHigh);
//            }  else if (gamepad1.left_bumper && gamepad1.dpad_right) {
//                liftArm.runArmUntil(liftArm.liftArmLow);
//            }  else {
//                liftArm.drive(0);
//            }

            // TODO: liftWrist - Manual
//            if (gamepad1.right_bumper && gamepad1.dpad_down) {
//                liftWrist.drive(-0.75);
//            }  else if (gamepad1.right_bumper && gamepad1.dpad_up) {
//                liftWrist.drive(0.75);
//            }  else if (gamepad1.left_bumper && gamepad1.dpad_up) {
//                liftWrist.runArmUntil(liftWrist.liftWristHigh);
//            }  else if (gamepad1.left_bumper && gamepad1.dpad_down) {
//                liftWrist.runArmUntil(liftWrist.liftWristLow);
//            } else {
//                liftWrist.drive(0);
//            }

            // RUN
            runEncoders();      //encoders
            runDrive();         //drivetrain
            dsTelemetry.sendTelemetry(telemetry, encLeftValue, encBackValue, encRightValue);    //telemetry
        }
    }

    //TODO /////////////////////////////////////////////////////////////// CUSTOM PRIVATE FUNCTIONS
    private void initDevices() throws InterruptedException {
        // SENSORS
        initIMU();
        initDistSensors();
        //initTouchSensors();
        //initEncoders();

        // MOTORS
        //initMotors();
        //initPixelArm();
        initLiftArm();
        initLiftWrist();
        //initServos();

        initAprilTag();
        //initTfod();
        initVision();
        //initCV();
    }
    private void runEncoders() throws InterruptedException {
        // Get current encoder position
        encLeftValue = encoders.encoderLeft.getCurrentPosition();
        encBackValue = encoders.encoderBack.getCurrentPosition();
        encRightValue = encoders.encoderRight.getCurrentPosition();
    }
    private void runDrive() throws InterruptedException {
        // DRIVETRAIN
        // Get yaw, reset in match optional
        if (gamepad1.start) {
            refHeading = 0;
            botHeading = 0;
            imu.resetYaw();
        }
        if (gamepad1.back) {
            encoders.encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoders.encoderBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoders.encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //pixelArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //pixelArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

//    private void initPixelArm() throws InterruptedException {
//        pixelArm = new PixelArm();
//        pixelArm.initPixelArm(hardwareMap);
//        //armController = new PIDController(pixelArm.p,pixelArm.i,pixelArm.d);
//    }

    private void initLiftArm() throws InterruptedException {
//        liftArm = new LiftArm();
//        liftArm.initLiftArm(hardwareMap);
        //armController = new PIDController(liftArm.p,liftArm.i,liftArm.d);
    }

    private void initLiftWrist() throws InterruptedException {
//        liftWrist = new LiftWrist();
//        liftWrist.initLiftWrist(hardwareMap);
        //armController = new PIDController(liftArm.p,liftArm.i,liftArm.d);
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
//
//        claw = hardwareMap.servo.get("pixelClaw");
//        claw.setDirection(Servo.Direction.FORWARD);
//        claw.setPosition(0.0); // pinch
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
                //.addProcessor(tfod)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1920, 1080))
                .build();
    }

    private void sendTelemetry() throws InterruptedException {
        String tag1_x = "";
        String tag1_y = "";

        String tag2_x = "";
        String tag2_y = "";

        String tag3_x = "";
        String tag3_y = "";

        String tag4_x = "";
        String tag4_y = "";

        String tag5_x = "";
        String tag5_y = "";

        String tag6_x = "";
        String tag6_y = "";

        String tag7_x = "";
        String tag7_y = "";

        String tag8_x = "";
        String tag8_y = "";

        String tag9_x = "";
        String tag9_y = "";

        String tag10_x = "";
        String tag10_y = "";

        telemetry.addLine(String.format(
                "y-x-rz | %5.2f : %5.2f : %5.2f",
                y,
                x,
                rz));

        telemetry.addLine(String.format(
                "REF,BOT,PID | %5.2f : %5.2f : %5.2f",
                refHeading,
                botHeading,
                pidOutput));

        telemetry.addLine(String.format(
                "THUMB |  %5.2f",
                pixelThumb.getPosition()
        ));

//        telemetry.addLine(String.format(
//                "PIXEL ARM |  %5d : %5d"
//                pixelArm.getTargetPosition(),
//                pixelArm.getCurrentPosition()
//        ));

        telemetry.addLine(String.format(
                "LIFT ARM |  %5d : %5d"
//                liftArm.getTargetPosition(),
//                liftArm.getCurrentPosition()
        ));

        telemetry.addLine(String.format(
                "LIFT WRIST |  %5d : %5d"
//                liftWrist.getTargetPosition(),
//                liftWrist.getCurrentPosition()
        ));

        telemetry.addLine("---");

        telemetry.addLine(String.format(
                "ENCODER LBR | %5.2f : %5.2f : %5.2f",
                encLeftValue * 0.003, // 0.0075
                encBackValue * 0.003,
                encRightValue * 0.003
        ));
        telemetry.addLine(String.format(
                "IMU HEADING | %5.2f",
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
        ));

        telemetry.addLine("---");

        telemetry.addLine(String.format(
                "DIST FB | %7.2f : %7.2f",
                distFront.getDistance(DistanceUnit.INCH), // 0.0075
                distBack.getDistance(DistanceUnit.INCH)
        ));

        //Vision Processing
        int tags = tagProcessor.getDetections().size();
        if (tagProcessor.getDetections().size() > 0) {
            ArrayList tagList = tagProcessor.getDetections();
            if (tagList != null) {
                for (Object tagItem: tagList) {
                    AprilTagDetection tag = (AprilTagDetection) tagItem;
                    if (tag.id == 1) {
                        tag1_x = String.format("%7.2f", tag.ftcPose.x);
                        tag1_y = String.format("%7.2f", tag.ftcPose.y);
                    }
                    if (tag.id == 2) {
                        tag2_x = String.format("%7.2f", tag.ftcPose.x);
                        tag2_y = String.format("%7.2f", tag.ftcPose.y);
                    }
                    if (tag.id == 3) {
                        tag3_x = String.format("%7.2f", tag.ftcPose.x);
                        tag3_y = String.format("%7.2f", tag.ftcPose.y);
                    }
                    if (tag.id == 4) {
                        tag4_x = String.format("%7.2f", tag.ftcPose.x);
                        tag4_y = String.format("%7.2f", tag.ftcPose.y);
                    }
                    if (tag.id == 5) {
                        tag5_x = String.format("%7.2f", tag.ftcPose.x);
                        tag5_y = String.format("%7.2f", tag.ftcPose.y);
                    }
                    if (tag.id == 6) {
                        tag6_x = String.format("%7.2f", tag.ftcPose.x);
                        tag6_y = String.format("%7.2f", tag.ftcPose.y);
                    }
                    if (tag.id == 7) {
                        tag7_x = String.format("%7.2f", tag.ftcPose.x);
                        tag7_y = String.format("%7.2f", tag.ftcPose.y);
                    }
                    if (tag.id == 8) {
                        tag8_x = String.format("%7.2f", tag.ftcPose.x);
                        tag8_y = String.format("%7.2f", tag.ftcPose.y);
                    }
                    if (tag.id == 9) {
                        tag9_x = String.format("%7.2f", tag.ftcPose.x);
                        tag9_y = String.format("%7.2f", tag.ftcPose.y);
                    }
                    if (tag.id == 10) {
                        tag10_x = String.format("%7.2f", tag.ftcPose.x);
                        tag10_y = String.format("%7.2f", tag.ftcPose.y);
                    }
                }
            }
            telemetry.addLine("---");
            telemetry.addLine("=== BLUE");
            telemetry.addLine("ID (1)" + tag1_x +  tag1_y);
            telemetry.addLine("ID (2)" + tag2_x +  tag2_y);
            telemetry.addLine("ID (3)" + tag3_x +  tag3_y);
            telemetry.addLine("---");
            telemetry.addLine("ID (9)" + tag9_x +  tag9_y);
            telemetry.addLine("ID (10)" + tag10_x +  tag10_y);

            telemetry.addLine("---");
            telemetry.addLine("=== RED");
            telemetry.addLine("ID (4)" + tag4_x +  tag4_y);
            telemetry.addLine("ID (5)" + tag5_x +  tag5_y);
            telemetry.addLine("ID (6)" + tag6_x +  tag6_y);
            telemetry.addLine("---");
            telemetry.addLine("ID (7)" + tag7_x +  tag7_y);
            telemetry.addLine("ID (8)" + tag8_x +  tag8_y);
        }
        telemetry.update();
    }
}
