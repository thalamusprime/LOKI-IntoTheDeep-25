package org.firstinspires.ftc.teamcode.ftc6205.sensors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class DriveEncoders {
    public DcMotor encoderLeft, encoderBack, encoderRight;
    public double encLeftValue, encBackValue, encRightValue;
    public static double tickToInches = 0.00285;
    HardwareMap hwMap;

    public DriveEncoders(HardwareMap ahwMap) {
        initEncoders(ahwMap);
    }

    public void initEncoders(HardwareMap ahwMap) {
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

    public void runEncoders() throws InterruptedException {
        // Get current encoder position
        encLeftValue = encoderLeft.getCurrentPosition() * tickToInches;
        encBackValue = encoderBack.getCurrentPosition() * tickToInches;
        encRightValue = encoderRight.getCurrentPosition() * tickToInches;
    }

}
