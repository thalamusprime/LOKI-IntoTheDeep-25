package org.firstinspires.ftc.teamcode.ftc6205.motors;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.controller.PIDController;

import java.util.Timer;

@Config
@TeleOp(name = "CONFIG - Lift Arm", group = "CONFIG")
public class LiftArm extends OpMode {
    public PIDController armController;
    public static double p = 1;
    public static double i = 0;
    public static double d = 0.01;
    public static double f = 0.005;
    public static int liftArmLow = 50;
    public static int liftArmHigh = 1200;
    public static int target = 0;

    //private final double ticks_in_degree = 5281.1/360; //537.7;
    public static double ticks_in_degree = 1680; //5281.1/360; //537.7;

    private DcMotorEx liftArm;
    private Timer timer;


    @Override
    public void init() {
        armController = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        liftArm = hardwareMap.get(DcMotorEx.class, "liftArm");
        liftArm.setDirection(DcMotorSimple.Direction.FORWARD);
        liftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //timer = new Timing.Timer();
        timer = new Timer();
    }
    @Override
    public void loop(){
        armController.setPID(p,i,d);
        int arm_position = liftArm.getCurrentPosition();
        double pid = armController.calculate(arm_position, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid * ff;

        liftArm.setPower(power);

        telemetry.addData("position ", arm_position);
        telemetry.addData("target ", target);
        telemetry.update();
    }

    public void runArm(int armTarget) {
        armController.setPID(p,i,d);
        int arm_position = liftArm.getCurrentPosition();
        double pid = armController.calculate(arm_position, armTarget);
        double ff = Math.cos(Math.toRadians(armTarget / LiftArm.ticks_in_degree)) * f;
        double power = pid * ff;
        liftArm.setPower(power);
    }

    public void initLiftArm(HardwareMap hMap) {
        armController = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        liftArm = hMap.get(DcMotorEx.class, "liftArm");
        liftArm.setDirection(DcMotorSimple.Direction.FORWARD);
        liftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        timer = new Timer();
    }

    public void runArmUntil(int armTarget) {
        if ( Math.abs(armTarget - liftArm.getCurrentPosition()) >= 2 ) {
            int arm_position = liftArm.getCurrentPosition();
            double pid = armController.calculate(arm_position, armTarget);
            double ff = Math.cos(Math.toRadians(armTarget / LiftArm.ticks_in_degree)) * f;
            double power = pid * ff;
            liftArm.setPower(power);
        } else {
            liftArm.setPower(0);
        }
    }

    public void drive(double speed){
        liftArm.setPower(speed);
    }

    public int getTargetPosition() {
        return liftArm.getTargetPosition();
    }

    public int getCurrentPosition() {
        return liftArm.getCurrentPosition();
    }
}
