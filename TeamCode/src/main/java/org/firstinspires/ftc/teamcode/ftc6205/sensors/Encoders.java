package org.firstinspires.ftc.teamcode.ftc6205.sensors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Encoders {
    public DcMotor encoderLeft, encoderBack, encoderRight;
    public double encLeftValue, encBackValue, encRightValue;

    HardwareMap hwMap;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        // Encoders
        encoderLeft = hwMap.dcMotor.get("frontleft");
        encoderBack = hwMap.dcMotor.get("backleft");
        encoderRight = hwMap.dcMotor.get("frontright");
        encoderLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        encoderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderBack.setDirection(DcMotorSimple.Direction.FORWARD);
        encoderBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderRight.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
