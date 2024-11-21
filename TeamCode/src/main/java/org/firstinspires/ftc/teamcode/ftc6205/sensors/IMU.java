package org.firstinspires.ftc.teamcode.ftc6205.sensors;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IMU {

    com.qualcomm.robotcore.hardware.IMU navx;
    HardwareMap hwMap;
    Gamepad gpad1;
    public double refHeading, botHeading;
    public double x, y, rotX, rotY;

    public IMU(HardwareMap ahwMap){
        initIMU(ahwMap);
    }

    public void initIMU(HardwareMap ahwMap) {
        hwMap = ahwMap;
        // Retrieve the IMU from the hardware map
        navx = hwMap.get(com.qualcomm.robotcore.hardware.IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        com.qualcomm.robotcore.hardware.IMU.Parameters parameters = new com.qualcomm.robotcore.hardware.IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT)); //FORWARD
        navx.initialize(parameters);
    }

//    private void runFieldCentric(Gamepad gpad) {
//        gpad1 = gpad;
//        botHeading = this.getYawInDegrees();
//        x = gpad1.left_stick_x;
//        y = -gpad1.left_stick_y;
//
//        // Field-centric drive, Robot-centric default
//        if (gpad1.left_bumper) {
//            rotX = x;
//            rotY = y;
//        } else {
//            // Rotate the movement direction counter to the bot's rotation
//            rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//            rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//        }
//        rotX = rotX * 1.1;  // Counteract imperfect strafing
//    }

    public void resetYaw() {
        navx.resetYaw();
    }

    public double getYawInDegrees() {
        return navx.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}
