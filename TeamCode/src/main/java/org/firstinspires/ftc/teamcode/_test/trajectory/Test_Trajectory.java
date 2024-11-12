package org.firstinspires.ftc.teamcode._test.trajectory;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "TEST - Trajectory", group = "TEST")
//@Disabled
public class Test_Trajectory extends LinearOpMode {
    Trajectory trajectory;
    List<Trajectory.State> states;
    Trajectory.State point;
    @Override
    public void runOpMode() throws InterruptedException {
        generateTrajectory();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            double duration = trajectory.getTotalTimeSeconds();
            point = trajectory.sample(5);
            telemetry.addLine(String.format(
                    "duration: %5.2f",
                    duration
            ));
            telemetry.addLine(String.format(
                    "Initial pose: ",
                    String.valueOf(trajectory.getInitialPose())
            ));
            telemetry.addLine(String.format(
                    "Heading after 5 seconds: ",
                    String.valueOf(point.poseMeters)
            ));
            telemetry.addLine(String.format(
                    "TIME:  %3f",
                    point.timeSeconds
            ));
            telemetry.update();
        }
    }
    public void generateTrajectory() {
        Pose2d sideStart = new Pose2d(0, 0, Rotation2d.fromDegrees(-180));
        Pose2d crossScale = new Pose2d(24, 24, Rotation2d.fromDegrees(-90));

        ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(14, 0));
        interiorWaypoints.add(new Translation2d(10, 18));

        TrajectoryConfig config = new TrajectoryConfig(12, 12);
        config.setReversed(true);

        trajectory = TrajectoryGenerator.generateTrajectory(
                sideStart,
                interiorWaypoints,
                crossScale,
                config);
    }
}
