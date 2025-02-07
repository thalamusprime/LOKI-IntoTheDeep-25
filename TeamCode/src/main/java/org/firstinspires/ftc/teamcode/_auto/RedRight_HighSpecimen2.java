package org.firstinspires.ftc.teamcode._auto;

import static org.firstinspires.ftc.teamcode.ftc6205.globals.FieldCoordinates.*;

import com.acmerobotics.dashboard.config.Config;
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

    public static double WALL_X = V;
    public static double WALL_X_OFFSET = PanelWidth/2;
    public static double WALL_Y = E;
    public static double WALL_Y_OFFSET = PanelWidth/2;

    public static double X_OFFSET = 8;
    public static double Y_OFFSET = 8;
    public static double H_OFFSET = Math.toRadians(-2);
    MecanumDrive drive;
    ClawAction clawAction;
    WristAction wristAction;
    ShoulderAction shoulderAction;

    TrajectoryActionBuilder
            initTraj,
            trajSubmersible,
            trajStageRedSamples,
            trajPush1stRedSample,
            trajPush2ndRedSample,
            trajStageRedWall,
            trajGrabHookRedSample;

    Pose2d startPose = new Pose2d(START_X, START_Y, START_ANGLE);
    Vector2d stageWallPose = new Vector2d(WALL_X+WALL_X_OFFSET, WALL_Y+WALL_Y_OFFSET);
    Vector2d hookWallPose = new Vector2d(WALL_X+WALL_X_OFFSET, WALL_Y-WALL_Y_OFFSET);

    @Override
    public void runOpMode() throws InterruptedException {
        //INIT
        this.initSubsystems();
        this.buildTrajectories();

        waitForStart();
        if (isStopRequested()) return;

        //PLAY
        //todo 1: Submersible ------------------------------------------

        // Travel to submersbile
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                new SleepAction(2.0),
                                trajSubmersible.build(),
                                shoulderAction.stageShoulder(),
                                wristAction.hookWrist()
                        )
                )
        );

        // Hook
        Actions.runBlocking(
                new ParallelAction(
                    new SleepAction(3.0),
                        new SequentialAction(
                                new ParallelAction(
                                        new SleepAction(1.0),
                                        shoulderAction.hookShoulder()
                                ),
                                clawAction.openClaw(),
                                wristAction.restWrist(),
                                shoulderAction.restShoulder()
                    )
                )
        );

        //todo 2/3: Staging Red Samples ------------------------------------------
        Actions.runBlocking(
                new ParallelAction(
                        new SleepAction(3.0),
                        trajStageRedSamples.build(),
                        //clawAction.closeClaw(),
                        shoulderAction.restShoulder()
                )
        );

        //todo 4/5/6: Push Sample One ------------------------------------------
        Actions.runBlocking(
                new ParallelAction(
                        new SleepAction(3.0),
                        trajPush1stRedSample.build()
                        //clawAction.closeClaw(),
                        //shoulderAction.initShoulder()
                )
        );

        //todo 7/8: Push Sample Two ------------------------------------------
        Actions.runBlocking(
                new ParallelAction(
                        new SleepAction(3.0),
                        trajPush2ndRedSample.build()
                )
        );

        //todo 9: Stage Wall
        Actions.runBlocking(
                new SequentialAction(
                    new ParallelAction(
                        wristAction.wallWrist(),
                        shoulderAction.wallShoulder(),
                        trajStageRedWall.build()
                    ),
                        new ParallelAction(
                        clawAction.openClaw()
                    )
                )
        );

        Actions.runBlocking(
                new SleepAction(4.0)
        );
        // completed at 15 sec
        // .strafeToLinearHeading(new Vector2d(VI, D), WEST) //9
        // .strafeToLinearHeading(new Vector2d(VI+X_OFFSET/2, D), WEST) //10
        // .strafeToLinearHeading(new Vector2d(VI+X_OFFSET/2, F+Y_OFFSET), WEST) //11
    }

    private void initSubsystems() {
        //motors
        drive = new MecanumDrive(hardwareMap, startPose);
        //actions
        clawAction = new ClawAction(hardwareMap);
        clawAction.closeClaw();
        wristAction = new WristAction(hardwareMap);
        wristAction.restWrist();
        shoulderAction = new ShoulderAction(hardwareMap);
        shoulderAction.restShoulder();
    }

    private void buildTrajectories(){
        initTraj = drive.actionBuilder(startPose);
        trajSubmersible = initTraj.endTrajectory().fresh()
                //todo: 1
                .strafeToLinearHeading(new Vector2d(III+X_OFFSET, E ), WEST);
        trajStageRedSamples = trajSubmersible.endTrajectory().fresh()
                //todo: 2
                .strafeToLinearHeading(new Vector2d(V, E), WEST)
                //todo: 3
                .strafeToLinearHeading(new Vector2d(V, D), WEST);
        trajPush1stRedSample = trajStageRedSamples.endTrajectory().fresh()
                //todo: 4
                .strafeToLinearHeading(new Vector2d(V+X_OFFSET, D), WEST)
                //todo: 5
                .strafeToLinearHeading(new Vector2d(V+X_OFFSET, F+Y_OFFSET), WEST)
                //todo: 6
                .strafeToLinearHeading(new Vector2d(V+X_OFFSET, D), WEST);
        trajPush2ndRedSample = trajPush1stRedSample.endTrajectory().fresh()
                //todo: 7
                .strafeToLinearHeading(new Vector2d(VI, D), WEST)
                //todo: 8
                .strafeToLinearHeading(new Vector2d(VI, F+Y_OFFSET), WEST);
        trajStageRedWall = trajPush2ndRedSample.endTrajectory().fresh()
                //todo: 9
                .strafeToLinearHeading(stageWallPose, EAST+H_OFFSET);
        trajGrabHookRedSample = trajStageRedWall.endTrajectory().fresh()
                //todo: 10
                .strafeToLinearHeading(hookWallPose, EAST+H_OFFSET);

    }

}
