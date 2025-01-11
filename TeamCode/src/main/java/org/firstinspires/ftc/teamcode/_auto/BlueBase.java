package org.firstinspires.ftc.teamcode._auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
// RR-specific imports

import org.firstinspires.ftc.teamcode.ftc6205.actions.ClawAction;
import org.firstinspires.ftc.teamcode.ftc6205.actions.DriveAction;
import org.firstinspires.ftc.teamcode.ftc6205.actions.LongArmAction;
import org.firstinspires.ftc.teamcode.ftc6205.controllers.TrueNorth;
import org.firstinspires.ftc.teamcode.ftc6205.globals.GridCoordinates;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Claw;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Drivetrain;
import org.firstinspires.ftc.teamcode.ftc6205.motors.LongArm;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.IMU;

@Config
@Autonomous(name = "*A: BLUE_BASE", group = "TEST")
public class BlueBase extends LinearOpMode {
    LongArm longArm;
    Claw claw;
    Drivetrain drivetrain;
    TrueNorth trueNorth;
    IMU imu;

    ClawAction clawAction;
    LongArmAction longArmAction;
    DriveAction driveAction;

    public static double START_ANGLE = Math.toRadians(-90);
    public static double START_X = GridCoordinates.III;
    public static double START_Y = GridCoordinates.A;

    Pose2d startPose = new Pose2d(START_X, START_Y, START_ANGLE);
    ElapsedTime timer = new ElapsedTime();

    double rotY, rotX, x, y, rz;
    double botHeading, refHeading;
    double pidOutput;

    boolean single_run = false;
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
            //todo: conditional for drivetrain.  may currently interfere with DriveAction
            drivetrain.runBot(gamepad1, rotY, rotX, rz);

            telemetry.update();

            // todo: abstract distance
            double distance = 24;

            // todo: conditional Actions
            if (true){
                //if (single_run == false){
                single_run = true;
                Actions.runBlocking(
                        new ParallelAction(
                                new SequentialAction(
                                        //longArmAction.liftDown(),
                                        clawAction.openClaw(),
                                        new SleepAction(1.0),
                                        //longArmAction.liftUp(),
                                        clawAction.closeClaw(),
                                        new SleepAction(1.0)
                                ),
                                new SequentialAction(
                                        driveAction.straight(distance),
                                        new SleepAction(1.0),
                                        //driveAction.turn(GridCoordinates.WEST),
                                        driveAction.strafe(distance),
                                        //new SleepAction(1.0),
                                        driveAction.turn(GridCoordinates.EAST),
                                        new SleepAction(1.0),
                                        driveAction.turn(GridCoordinates.NORTH)
                                )
                        )
                ); //end runBlocking
            } //end: Action Pipeline
        } //end: opModeisActive
    }
    private void resetCheck() {
        if (gamepad1.share) {
            //claw.initClaw();
            //wrist.initWrist();
            //foreArm.initForeArm();
            //arm.initArm();
            //lift.initLift();
            //deadWheels.initEncoders(hardwareMap);
            driveAction.deadWheels.initEncoders(hardwareMap);
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
    }
    private void initActuators(){
        drivetrain = new Drivetrain(hardwareMap);
    }

    private void initActions(){
        clawAction = new ClawAction(hardwareMap);
        longArmAction = new LongArmAction(hardwareMap);
        driveAction = new DriveAction(hardwareMap);
    }
    private void initTrajectories(){

    }
}