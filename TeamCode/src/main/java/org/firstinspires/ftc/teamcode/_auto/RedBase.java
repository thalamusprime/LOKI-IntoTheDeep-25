package org.firstinspires.ftc.teamcode._auto;

import static org.firstinspires.ftc.teamcode.ftc6205.globals.FieldCoordinates.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.TuningOpModes;
@Config
@Autonomous(name = "*F: RED_BASE", group = "AUTO")
//@Disabled
public final class RedBase extends LinearOpMode {
    public static double START_ANGLE = Math.toRadians(90);
    public static double START_X = IV;
    public static double START_Y_OFFSET = 0;
    public static double START_Y = F-START_Y_OFFSET;
    public static double X_OFFSET = 8;
    public static double Y_OFFSET = 8;

    Pose2d startPose = new Pose2d(START_X, START_Y, START_ANGLE);

    @Override
    public void runOpMode() throws InterruptedException {
            MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .strafeToLinearHeading(new Vector2d(12,0 ), 0)
                    //todo: 1
                    .strafeToLinearHeading(new Vector2d(III+X_OFFSET, E ), WEST)
                    //todo: 2
                    .strafeToLinearHeading(new Vector2d(V, E), WEST)
                    //todo: 3
                    .strafeToLinearHeading(new Vector2d(V, D), WEST)
                    //todo: 4
                    .strafeToLinearHeading(new Vector2d(V+X_OFFSET, D), WEST)
                    //todo: 5
                    .strafeToLinearHeading(new Vector2d(V+X_OFFSET, F+Y_OFFSET), WEST)
                    //todo: 6
                    .strafeToLinearHeading(new Vector2d(V+X_OFFSET, D), WEST)
                    //todo: 7
                    .strafeToLinearHeading(new Vector2d(VI, D), WEST)
                    //todo: 8
                    .strafeToLinearHeading(new Vector2d(VI, F+Y_OFFSET), WEST)
                    // completed at 15 sec
                    // .strafeToLinearHeading(new Vector2d(VI, D), WEST) //9
                    // .strafeToLinearHeading(new Vector2d(VI+X_OFFSET/2, D), WEST) //10
                    // .strafeToLinearHeading(new Vector2d(VI+X_OFFSET/2, F+Y_OFFSET), WEST) //11
                    .build());
    }
}
