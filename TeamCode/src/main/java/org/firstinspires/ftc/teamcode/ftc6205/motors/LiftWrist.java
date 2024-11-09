package org.firstinspires.ftc.teamcode.ftc6205.motors;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Timer;

@Config
@TeleOp(name = "CONFIG - Lift Wrist", group = "CONFIG")
public class LiftWrist extends OpMode {
    public PIDController armController;
    public static double p = 1;
    public static double i = 0;
    public static double d = 0.01;
    public static double f = 0.005;
    public static int liftWristLow = 50;
    public static int liftWristHigh = 3500;
    public static int target = 0;

    //private final double ticks_in_degree = 5281.1/360; //537.7;
    public static double ticks_in_degree = 1680; //5281.1/360; //537.7;

    private DcMotorEx liftWrist;
    private Timer timer;


    @Override
    public void init() {
        armController = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        liftWrist = hardwareMap.get(DcMotorEx.class, "liftWrist");
        liftWrist.setDirection(DcMotorSimple.Direction.FORWARD);
        liftWrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //timer = new Timing.Timer();
        timer = new Timer();
    }
    @Override
    public void loop(){
        armController.setPID(p,i,d);
        int arm_position = liftWrist.getCurrentPosition();
        double pid = armController.calculate(arm_position, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid * ff;

        liftWrist.setPower(power);

        telemetry.addData("position ", arm_position);
        telemetry.addData("target ", target);
        telemetry.update();
    }

    public void initArm(HardwareMap hMap) {
        armController = new PIDController(p,i,d);

        liftWrist = hardwareMap.get(DcMotorEx.class, "liftWrist");
        liftWrist.setDirection(DcMotorSimple.Direction.FORWARD);
        liftWrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runArm(int armTarget) {
        armController.setPID(p,i,d);
        int arm_position = liftWrist.getCurrentPosition();
        double pid = armController.calculate(arm_position, armTarget);
        double ff = Math.cos(Math.toRadians(armTarget / LiftWrist.ticks_in_degree)) * f;
        double power = pid * ff;
        liftWrist.setPower(power);
    }

    public void initLiftWrist(HardwareMap hMap) {
        armController = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        liftWrist = hMap.get(DcMotorEx.class, "liftWrist");
        liftWrist.setDirection(DcMotorSimple.Direction.FORWARD);
        liftWrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        timer = new Timer();
    }

    public void runArmUntil(int armTarget) {
        if ( Math.abs(armTarget - liftWrist.getCurrentPosition()) >= 2 ) {
            int arm_position = liftWrist.getCurrentPosition();
            double pid = armController.calculate(arm_position, armTarget);
            double ff = Math.cos(Math.toRadians(armTarget / LiftWrist.ticks_in_degree)) * f;
            double power = pid * ff;
            liftWrist.setPower(power);
        } else {
            liftWrist.setPower(0);
        }
    }

    public void drive(double speed){
        liftWrist.setPower(speed);
    }

    public int getTargetPosition() {
        return liftWrist.getTargetPosition();
    }

    public int getCurrentPosition() {
        return liftWrist.getCurrentPosition();
    }
}
