package org.firstinspires.ftc.teamcode._auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
// RR-specific imports

import org.firstinspires.ftc.teamcode.ftc6205.actions.DriveAction;
import org.firstinspires.ftc.teamcode.ftc6205.actions.LiftAction;
import org.firstinspires.ftc.teamcode.ftc6205.controllers.TrueNorth;
import org.firstinspires.ftc.teamcode.ftc6205.globals.GridCoordinates;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Claw;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Drivetrain;
import org.firstinspires.ftc.teamcode.ftc6205.motors.LongArm;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.DeadWheels;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.IMU;

@Config
@Autonomous(name = "*A: BLUE_BASE", group = "TEST")
public class BlueBase extends LinearOpMode {
    LongArm longArm;
    Claw claw;
    //DeadWheels deadWheels;
    Drivetrain drivetrain;
    TrueNorth trueNorth;
    IMU imu;

    LiftAction liftAction;
    DriveAction driveAction;

    public static double START_ANGLE = Math.toRadians(-90);
    public static double START_X = GridCoordinates.III;
    public static double START_Y = GridCoordinates.A;

    Pose2d startPose = new Pose2d(START_X, START_Y, START_ANGLE);
    ElapsedTime timer = new ElapsedTime();

    double rotY, rotX, x, y, rz;
    double botHeading, refHeading;
    double pidOutput;

    @Override
    public void runOpMode() throws InterruptedException {

        this.initSensors();
        this.initActuators();
        this.initActions();
        this.initTrajectories();

        waitForStart();  // Pause until "PLAY"
        if (isStopRequested()) return; // Close program if "STOP"

        // TELEOP LOOP
        while (opModeIsActive()) {
            this.resetCheck();              // RESET TrueNorth | Encoders
            this.runTrueNorth();            // PID Straight
            this.runFieldCentric();         //
            drivetrain.runBot(gamepad1, rotY, rotX, rz);

            //telemetry.addLine(String.format("L|B|R: ",
                    //deadWheels.encLeftValue,
                    //deadWheels.encBackValue,
                    //deadWheels.encRightValue)
            //);
            //telemetry.addData("D: ", drivetrain.frontLeftDriveMotor.getPower());
            telemetry.update();
            Actions.runBlocking(
                    new SequentialAction(
                            liftAction.liftUp(),
                            driveAction.driveForward(),
                            liftAction.liftDown()
                    )
            );

        }
    }
    private void resetCheck() {
        if (gamepad1.share) {
            //claw.initClaw();
            //wrist.initWrist();
            //foreArm.initForeArm();
            //arm.initArm();
            //lift.initLift();
            //deadWheels.initEncoders(hardwareMap);
            drivetrain.initDriveMotors(hardwareMap);
        }
        if (gamepad1.options) {
            refHeading = 0;
            botHeading = 0;
            imu.resetYaw();
            //otos.resetTracking();
        }
    }

    private void runTrueNorth(){
        y = gamepad1.left_stick_y;  //
        x = -gamepad1.left_stick_x; //

        if (Math.abs(gamepad1.right_stick_x) > 0.03) { // Yaw threshold
            rz = -gamepad1.right_stick_x;
            refHeading = imu.getYawInRadians();  // todo: otos?
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
        botHeading = imu.getYawInRadians(); // bot

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

    private void initSensors(){
        imu = new IMU(hardwareMap);
        //deadWheels = new DeadWheels(hardwareMap);
    }
    private void initActuators(){
        drivetrain = new Drivetrain(hardwareMap);
    }

    private void initActions(){
        liftAction = new LiftAction(hardwareMap);
        driveAction = new DriveAction(hardwareMap);
    }
    private void initTrajectories(){

    }
}