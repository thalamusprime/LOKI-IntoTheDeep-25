package org.firstinspires.ftc.teamcode.ftc6205.motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    public DcMotor topLeftDriveMotor;
    public DcMotor bottomLeftDriveMotor;
    public DcMotor topRightDriveMotor;
    public DcMotor bottomRightDriveMotor;

    HardwareMap hwMap;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Control HUb
        topLeftDriveMotor = hwMap.dcMotor.get("frontleft");
        bottomLeftDriveMotor = hwMap.dcMotor.get("backleft");
        topRightDriveMotor = hwMap.dcMotor.get("frontright");
        bottomRightDriveMotor = hwMap.dcMotor.get("backright");

        //todo: research why front two motors do not work with this method call
        //topLeftDriveMotor = hwMap.get(DcMotor.class, "frontleft");
        //bottomLeftDriveMotor = hwMap.get(DcMotor.class, "backleft");
        //topRightDriveMotor = hwMap.get(DcMotor.class, "frontright");
        //bottomRightDriveMotor = hwMap.get(DcMotor.class, "backright");

        //THIS IS THE CORRECT ORIENTATION
        //topLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        //bottomLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        topRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        //
        topLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //
        topLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}