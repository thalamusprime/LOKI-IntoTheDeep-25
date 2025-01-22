package org.firstinspires.ftc.teamcode._auto;

import static org.firstinspires.ftc.teamcode.ftc6205.globals.FieldCoordinates.*;
//import static org.firstinspires.ftc.teamcode.ftc6205.globals.FieldCoordinates.E;
//import static org.firstinspires.ftc.teamcode.ftc6205.globals.FieldCoordinates.F;
//import static org.firstinspires.ftc.teamcode.ftc6205.globals.FieldCoordinates.III;
//import static org.firstinspires.ftc.teamcode.ftc6205.globals.FieldCoordinates.IV;
//import static org.firstinspires.ftc.teamcode.ftc6205.globals.FieldCoordinates.V;
//import static org.firstinspires.ftc.teamcode.ftc6205.globals.FieldCoordinates.VI;
//import static org.firstinspires.ftc.teamcode.ftc6205.globals.FieldCoordinates.WEST;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ftc6205.actions.ClawAction;
import org.firstinspires.ftc.teamcode.ftc6205.actions.ShoulderAction;
import org.firstinspires.ftc.teamcode.ftc6205.actions.WristAction;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
@Autonomous(name = "*F: RED_RIGHT_HIGH_SPECIMEN2", group = "AUTO")
//@Disabled
public final class RedRight_HighSpecimen2 extends LinearOpMode {
    public static double START_ANGLE = Math.toRadians(90);
    public static double START_X = IV;
    public static double START_Y_OFFSET = 0;
    public static double START_Y = F-START_Y_OFFSET;
    public static double X_OFFSET = 8;
    public static double Y_OFFSET = 8;

    MecanumDrive drive;
    ClawAction clawAction;
    WristAction wristAction;
    ShoulderAction shoulderAction;

    TrajectoryActionBuilder trajSubmersible; // Action
    TrajectoryActionBuilder trajToRedSample; // Action

    Pose2d startPose = new Pose2d(START_X, START_Y, START_ANGLE);

    @Override
    public void runOpMode() throws InterruptedException {
        //INIT
        this.initSubsystems();
        this.buildTrajectories();

        waitForStart();
        if (isStopRequested()) return;

        //PLAY
        Actions.runBlocking(
                new ParallelAction(
                        new SleepAction(4.0),
                        trajSubmersible.build(),
                        clawAction.openClaw(),
                        shoulderAction.stageShoulder()
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                    new SleepAction(4.0),
                        trajToRedSample.build(),
                        clawAction.closeClaw(),
                        shoulderAction.hookShoulder()
                )
        );
//                new SequentialAction(
                    // Load stored specimen
//                    new ParallelAction(
//                            new SequentialAction(
//                                    trajSubmersible
//                            ),
//                            new SequentialAction(
//                                    shoulderAction.stageShoulder()
                                    //wristAction.floorWrist()
//                            )
//                    )//,
//                    new SequentialAction(
//                            shoulderAction.hookShoulder(),
//                            wristAction.initWrist(),
//                            shoulderAction.initShoulder()
//                    ),
//                    new ParallelAction(
//                            new SequentialAction(
//                                    trajToRedSample
//                            ),
//                            new SequentialAction(
//                                    clawAction.openClaw(),
//                                    new SleepAction(1.0),
//                                    clawAction.closeClaw()
//                            )
//                    )
//                )
//        );
//        Actions.runBlocking(
//                drive.actionBuilder(startPose)
//                        //todo: 1 | to submersible
//                        .strafeToLinearHeading(new Vector2d(III+X_OFFSET, E ), WEST)
//                        //todo: 2
//                        .strafeToLinearHeading(new Vector2d(V, E), WEST)
//                        //todo: 3
//                        .strafeToLinearHeading(new Vector2d(V, D), WEST)
//                        .build());
//        Actions.runBlocking(
//                drive.actionBuilder(startPose)
//                        //todo: 4
//                        .strafeToLinearHeading(new Vector2d(V+X_OFFSET, D), WEST)
//                        //todo: 5
//                        .strafeToLinearHeading(new Vector2d(V+X_OFFSET, F+Y_OFFSET), WEST)
//                        //todo: 6
//                        .strafeToLinearHeading(new Vector2d(V+X_OFFSET, D), WEST)
//                        //todo: 7
//                        .strafeToLinearHeading(new Vector2d(VI, D), WEST)
//                        //todo: 8
//                        .strafeToLinearHeading(new Vector2d(VI, F+Y_OFFSET), WEST)
//                        // completed at 15 sec
//                        // .strafeToLinearHeading(new Vector2d(VI, D), WEST) //9
//                        // .strafeToLinearHeading(new Vector2d(VI+X_OFFSET/2, D), WEST) //10
//                        // .strafeToLinearHeading(new Vector2d(VI+X_OFFSET/2, F+Y_OFFSET), WEST) //11
//                        .build());
    }

    private void initSubsystems() {
        //motors
        drive = new MecanumDrive(hardwareMap, startPose);
        //actions
        clawAction = new ClawAction(hardwareMap);
        clawAction.closeClaw();
        wristAction = new WristAction(hardwareMap);
        wristAction.initWrist();
        shoulderAction = new ShoulderAction(hardwareMap);
        shoulderAction.initShoulder();
    }

    private void buildTrajectories(){
        trajSubmersible = drive.actionBuilder(startPose)
                //todo: 1 | to submersible
                .strafeToLinearHeading(new Vector2d(III+X_OFFSET, E ), WEST);
                //.build();
        trajToRedSample = drive.actionBuilder(startPose)
                //todo: 2
                .strafeToLinearHeading(new Vector2d(V, E), WEST)
                //todo: 3
                .strafeToLinearHeading(new Vector2d(V, D), WEST);
                //.build();

    }

}
