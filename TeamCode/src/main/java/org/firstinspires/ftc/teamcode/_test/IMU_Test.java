package org.firstinspires.ftc.teamcode._test;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "TEST - IMU Orthogonal", group = "TEST")
@Disabled   // Comment this out to add to the OpMode list
public class IMU_Test extends LinearOpMode {
    // The IMU sensor object
    IMU imu;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() throws InterruptedException {

        // Retrieve and initialize the IMU.
        imu = hardwareMap.get(IMU.class, "imu");

        // Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Initialize the IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Loop and update the dashboard
        while (!isStopRequested()) {

            telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);

            // Check to see if heading reset is requested
            if (gamepad1.y) {
                telemetry.addData("Yaw", "Resetting\n");
                imu.resetYaw();
            } else {
                telemetry.addData("Yaw", "Press Y (triangle) on Gamepad to reset\n");
            }

            // Retrieve Rotational Angles and Velocities
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.update();
        }
    }
}
