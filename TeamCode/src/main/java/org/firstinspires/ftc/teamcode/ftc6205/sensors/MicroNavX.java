package org.firstinspires.ftc.teamcode.ftc6205.sensors;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MicroNavX {

    IMU navx;
    HardwareMap hwMap;
    public void initIMU(HardwareMap ahwMap) {
        hwMap = ahwMap;
        // Retrieve the IMU from the hardware map
        navx = hwMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT)); //FORWARD
        navx.initialize(parameters);
    }

    public void resetYaw() {
        navx.resetYaw();
    }

    public double getYawInDegrees() {
        return navx.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}
