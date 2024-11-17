package org.firstinspires.ftc.teamcode.ftc6205.motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ftc6205.controllers.TrueNorth;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.DriveEncoders;

public class Drivetrain {
    public DcMotor frontLeftDriveMotor;
    public DcMotor backLeftDriveMotor;
    public DcMotor frontRightDriveMotor;
    public DcMotor backRightDriveMotor;

    HardwareMap hwMap;
    Gamepad gpad1;

    public Drivetrain(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Control Hub
        frontLeftDriveMotor = hwMap.dcMotor.get("frontleft");
        backLeftDriveMotor = hwMap.dcMotor.get("backleft");
        frontRightDriveMotor = hwMap.dcMotor.get("frontright");
        backRightDriveMotor = hwMap.dcMotor.get("backright");

        //THIS IS THE CORRECT ORIENTATION
        frontLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        //
        frontLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //
        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void initDriveMotors(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Control HUb
        frontLeftDriveMotor = hwMap.dcMotor.get("frontleft");
        backLeftDriveMotor = hwMap.dcMotor.get("backleft");
        frontRightDriveMotor = hwMap.dcMotor.get("frontright");
        backRightDriveMotor = hwMap.dcMotor.get("backright");

        //THIS IS THE CORRECT ORIENTATION
        frontLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        //
        frontLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //
        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runBot(Gamepad gpad, double rotY, double rotX, double rz) {
        gpad1 = gpad;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rz), 1);
        double frontLeftPower = (rotY + rotX + rz) / denominator;
        double backLeftPower = (rotY - rotX + rz) / denominator;
        double frontRightPower = (rotY - rotX - rz) / denominator;
        double backRightPower = (rotY + rotX - rz) / denominator;

        // Trigger gain
        frontLeftPower = frontLeftPower * (0.3 + 0.7 * gpad1.right_trigger);
        backLeftPower = backLeftPower * (0.3 + 0.7 * gpad1.right_trigger);
        frontRightPower = frontRightPower * (0.3 + 0.7 * gpad1.right_trigger);
        backRightPower = backRightPower * (0.3 + 0.7 * gpad1.right_trigger);

        // Set motor power
        frontLeftDriveMotor.setPower(frontLeftPower);
        backLeftDriveMotor.setPower(backLeftPower);
        frontRightDriveMotor.setPower(frontRightPower);
        backRightDriveMotor.setPower(backRightPower);
    }

    public void autoForward(double power){//;//, DriveEncoders driveEncoders, int leftTarget, int rightTarget){
//        double forwardTicks = 0;
//        double leftValue = driveEncoders.encLeftValue;
//        double rightValue = driveEncoders.encRightValue;
//        double pid = pidController.calculate(current_position, leftValue);

        double frontLeftPower = -power;
        double backLeftPower = -power;
        double frontRightPower = -power;
        double backRightPower = -power;
        frontLeftDriveMotor.setPower(frontLeftPower);
        backLeftDriveMotor.setPower(backLeftPower);
        frontRightDriveMotor.setPower(frontRightPower);
        backRightDriveMotor.setPower(backRightPower);
    }
    public void autoStrafe(double power){
        double frontLeftPower = power;
        double backLeftPower = -power;
        double frontRightPower = -power;
        double backRightPower = power;
        frontLeftDriveMotor.setPower(frontLeftPower);
        backLeftDriveMotor.setPower(backLeftPower);
        frontRightDriveMotor.setPower(frontRightPower);
        backRightDriveMotor.setPower(backRightPower);
    }

}