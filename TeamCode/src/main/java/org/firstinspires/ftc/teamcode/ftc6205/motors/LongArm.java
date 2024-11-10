package org.firstinspires.ftc.teamcode.ftc6205.motors;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.controller.PIDController;

import java.util.Timer;

@Config
@TeleOp(name = "CONFIG - ForeArm", group = "CONFIG")
public class LongArm extends OpMode {
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

    private DcMotorEx longArm;
    private Timer timer;

    private Gamepad gpad1;

    @Override
    public void init() {
        armController = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        longArm = hardwareMap.get(DcMotorEx.class, "liftArm");
        longArm.setDirection(DcMotorSimple.Direction.FORWARD);
        longArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //timer = new Timing.Timer();
        timer = new Timer();
    }
    @Override
    public void loop(){
        armController.setPID(p,i,d);
        int arm_position = longArm.getCurrentPosition();
        double pid = armController.calculate(arm_position, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid * ff;

        longArm.setPower(power);

        telemetry.addData("position ", arm_position);
        telemetry.addData("target ", target);
        telemetry.update();
    }

    public void runArm(int armTarget) {
        armController.setPID(p,i,d);
        int arm_position = longArm.getCurrentPosition();
        double pid = armController.calculate(arm_position, armTarget);
        double ff = Math.cos(Math.toRadians(armTarget / LongArm.ticks_in_degree)) * f;
        double power = pid * ff;
        longArm.setPower(power);
    }

    public void raise(Gamepad gpad) {
        gpad1 = gpad;
        if (gpad1.right_bumper && gpad1.dpad_left) {
            this.drive(0.2);
        }  else if (gpad1.right_bumper && gpad1.dpad_right) {
            this.drive(-0.2);
        }  else if (gpad1.left_bumper && gpad1.dpad_left) {
            this.runArmUntil(this.liftArmHigh);
        }  else if (gpad1.left_bumper && gpad1.dpad_right) {
            this.runArmUntil(this.liftArmLow);
        }  else {
            this.drive(0);
        }
    }
    public void initLongArm(HardwareMap hMap) {
        armController = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        longArm = hMap.get(DcMotorEx.class, "longArm");
        longArm.setDirection(DcMotorSimple.Direction.FORWARD);
        longArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        timer = new Timer();
    }
//    private void initLiftArm() throws InterruptedException {
//        longArm = new LongArm();
//        longArm.initLiftArm(hardwareMap);
//        armController = new PIDController(longArm.p, longArm.i, longArm.d);
//    }

    public void runArmUntil(int armTarget) {
        if ( Math.abs(armTarget - longArm.getCurrentPosition()) >= 2 ) {
            int arm_position = longArm.getCurrentPosition();
            double pid = armController.calculate(arm_position, armTarget);
            double ff = Math.cos(Math.toRadians(armTarget / LongArm.ticks_in_degree)) * f;
            double power = pid * ff;
            longArm.setPower(power);
        } else {
            longArm.setPower(0);
        }
    }

    public void drive(double speed){
        longArm.setPower(speed);
    }

    public int getTargetPosition() {
        return longArm.getTargetPosition();
    }

    public int getCurrentPosition() {
        return longArm.getCurrentPosition();
    }
}
