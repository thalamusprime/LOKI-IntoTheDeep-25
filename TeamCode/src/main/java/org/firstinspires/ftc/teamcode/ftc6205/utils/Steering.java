package org.firstinspires.ftc.teamcode.ftc6205.utils;

public class Steering {
    private void resetCheck() {
        //PS: share
        if (gamepad1.back) {
            shortArm.initArm(hardwareMap);
            longArm.initLongArm(hardwareMap);
            driveEncoders.initEncoders(hardwareMap);
            drivetrain.initDriveMotors(hardwareMap);
        }
        //PS: options
        if (gamepad1.start) {
            refHeading = 0;
            botHeading = 0;
            navx.resetYaw();
        }
    }

    private void runTrueNorth() {
        // Get XY: gamepad1
        y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        x = -gamepad1.left_stick_x; //-

        // Get  Z: gamepad1
        if (Math.abs(gamepad1.right_stick_x) > 0.03) { // Yaw threshold
            rz = -gamepad1.right_stick_x;
            refHeading = navx.getYawInDegrees();//.getYaw(AngleUnit.RADIANS); // ref
        } else {
            rz = 0;
            //pidOutput = trueNorth.TwistControl(refHeading, botHeading);
            pidOutput = pidController.calculate(botHeading, refHeading);
            if (Math.abs(pidOutput) > 0.03) {
                rz = pidOutput;
            }
        }
    }

    private void runFieldCentric() {
        botHeading = navx.getYawInDegrees();

        // Field-centric drive, Robot-centric default
        if (gamepad1.left_bumper) {
            rotX = x;
            rotY = y;
        } else {
            // Rotate the movement direction counter to the bot's rotation
            rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        }
        rotX = rotX * 1.1;  // Counteract imperfect strafing
    }
}
