package org.firstinspires.ftc.teamcode.ftc6205.motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    public DcMotor frontLeftDriveMotor;
    public DcMotor backLeftDriveMotor;
    public DcMotor frontRightDriveMotor;
    public DcMotor backRightDriveMotor;

    HardwareMap hwMap;
    Gamepad gpad1;

    public void init(HardwareMap ahwMap) {
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
}