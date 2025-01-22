package org.firstinspires.ftc.teamcode.ftc6205.motors;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Timer;

@Config
@TeleOp(name = "CONFIG - Short Arm", group = "CONFIG")
@Disabled
public class ShortArm extends OpMode {
    public PIDController armController;
    public static double p = 1;
    public static double i = 0;
    public static double d = 0.01;
    public static double f = 0.005;
    public static int armFloor = 3400;
    public static int armLowGoal = 2350;
    public static int armInit = 0;
    public static int target = 0;
    public static double short_arm_speed = 0.75;

    //private final double ticks_in_degree = 5281.1/360; //537.7;
    public static double ticks_in_degree = 1680; //5281.1/360; //537.7;

    private DcMotorEx arm;
    private Timer timer;

    private HardwareMap hwMap;
    private Gamepad gpad1;

    public ShortArm(HardwareMap ahwMap) {
        initArm(ahwMap);
    }
    public void initArm(HardwareMap ahwMap) {
        hwMap = ahwMap;

        armController = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = hwMap.get(DcMotorEx.class, "shortArm");
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //timer = new Timing.Timer();
        timer = new Timer();
    }

    @Override
    public void init() {

        armController = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = hardwareMap.get(DcMotorEx.class, "shortArm");
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //timer = new Timing.Timer();
        timer = new Timer();
    }


    @Override
    public void loop(){
        armController.setPID(p,i,d);
        int arm_position = arm.getCurrentPosition();
        double pid = armController.calculate(arm_position, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid * ff;

        arm.setPower(power);

        telemetry.addData("position ", arm_position);
        telemetry.addData("target ", target);
        telemetry.update();
    }

    public void runArm(int armTarget) {
        armController.setPID(p,i,d);
        int arm_position = arm.getCurrentPosition();
        double pid = armController.calculate(arm_position, armTarget);
        double ff = Math.cos(Math.toRadians(armTarget / ShortArm.ticks_in_degree)) * f;
        double power = pid * ff;
        arm.setPower(power);
    }
    public void initPixelArm(HardwareMap hMap) {
        armController = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = hMap.get(DcMotorEx.class, "pixelArm");
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        timer = new Timer();
    }

    public void autoArmUntil(int armTarget) {
        while ( Math.abs(armTarget - arm.getCurrentPosition()) > 1 ) {
            int arm_position = arm.getCurrentPosition();
            double pid = armController.calculate(arm_position, armTarget);
            double ff = Math.cos(Math.toRadians(armTarget / ShortArm.ticks_in_degree)) * f;
            double power = pid * ff;
            arm.setPower(power);
        }
        arm.setPower(0);
    }

        public void rotArmUntil(int armTarget) {
        if ( Math.abs(armTarget - arm.getCurrentPosition()) >= 2 ) {
            int arm_position = arm.getCurrentPosition();
            double pid = armController.calculate(arm_position, armTarget);
            double ff = Math.cos(Math.toRadians(armTarget / ShortArm.ticks_in_degree)) * f;
            double power = pid * ff;
            arm.setPower(power);
        } else {
            arm.setPower(0);
        }
    }

    public void rot(double speed){
        arm.setPower(speed);
    }

    public int getTargetPosition() {
        return arm.getTargetPosition();
    }

    public void rotate(Gamepad gpad) {
        gpad1 = gpad;

//        if (!gpad1.left_bumper && gpad1.dpad_down) {
//            this.rot(this.short_arm_speed);
//        }  else if (!gpad1.left_bumper && gpad1.dpad_up) {
//            this.rot(-this.short_arm_speed);
//        }  else if (gpad1.a) {
//            this.rotArmUntil(this.armFloor);
//        }  else if (gpad1.b) {
//            this.rotArmUntil(this.armLowGoal);
//        }  else {
//            this.rot(0);
//        }

    }

    public int getCurrentPosition() {
        return arm.getCurrentPosition();
    }

    public void setMode(DcMotor.RunMode mode) {
        arm.setMode(mode);
    }
}
