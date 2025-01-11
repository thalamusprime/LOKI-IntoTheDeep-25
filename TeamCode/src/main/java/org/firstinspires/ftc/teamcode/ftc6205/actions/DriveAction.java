package org.firstinspires.ftc.teamcode.ftc6205.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ftc6205.controllers.TrueNorth;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Drivetrain;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.DeadWheels;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.IMU;

@Config
public class DriveAction {
    public Drivetrain drivetrain;
    public DeadWheels deadWheels;
    public IMU imu;

    public static double DISTANCE = 24;
    public double forwardDist = 0;
    public double sideDist = 0;
    public double angleDist = 0;
    public double target_x, target_y, target_h;
    public double refHeading, botHeading;


    public DriveAction(HardwareMap hardwareMap) {
        deadWheels = new DeadWheels(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        imu = new IMU(hardwareMap);

    }
    public class Straight implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            deadWheels.runEncoders();

            if (!initialized) {
                drivetrain.autoSpeedForward(0.3);
                initialized = true;
            }

            double encLeftPos = deadWheels.encLeftValue;
            double encBackPos = deadWheels.encBackValue;
            double encRightPos = deadWheels.encRightValue;
            packet.put("encLeftValue", encLeftPos);
            packet.put("encBackValue", encBackPos);
            packet.put("encRightValue", encRightPos);
            if (encRightPos < forwardDist) {
                return true;
            } else {
                drivetrain.autoSpeedForward(0);
                forwardDist = 0;
                return false;
            }
        }
    }

    public Action straight(double aDist) {
        forwardDist = aDist;
        return new Straight();
    }
    public class Strafe implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            deadWheels.runEncoders();

            if (!initialized) {
                drivetrain.autoStrafe(0.5);
                refHeading = imu.getYawInRadians(); // get current heading
                initialized = true;
            }

            //todo: clean up TrueNorth
            //if (Math.abs(gamepad1.right_stick_x) > 0.03) { // Yaw threshold
            botHeading = imu.getYawInRadians();
            double strafePower = refHeading - botHeading;
            if (Math.abs(strafePower) > 0.03){
                //TrueNorth trueNorth = new TrueNorth();
                //double pidOutput = trueNorth.twistControl(refHeading, botHeading);
                drivetrain.autoTurn(refHeading, botHeading);
            }
            double encLeftPos = deadWheels.encLeftValue;
            double encBackPos = deadWheels.encBackValue;
            double encRightPos = deadWheels.encRightValue;
            packet.put("encLeftValue", encLeftPos);
            packet.put("encBackValue", encBackPos);
            packet.put("encRightValue", encRightPos);

            if (encBackPos < sideDist) {
                drivetrain.autoStrafe(strafePower);
                return true;
            } else {
                drivetrain.autoStrafe(0);
                sideDist = 0;
                return false;
            }
        }
    }

    public Action strafe(double aDist) {
        sideDist = aDist;
        return new Strafe();
    }
    public class Turn implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            deadWheels.runEncoders();

            if (!initialized) {
                //drivetrain.autoTurn(0.25);
                initialized = true;
            }
            refHeading = angleDist;
            botHeading = imu.getYawInRadians();
            double turnPower = refHeading-botHeading;

            double encLeftPos = deadWheels.encLeftValue;
            double encBackPos = deadWheels.encBackValue;
            double encRightPos = deadWheels.encRightValue;
            packet.put("botHeading", botHeading);
            packet.put("encLeftValue", encLeftPos);
            packet.put("encBackValue", encBackPos);
            packet.put("encRightValue", encRightPos);

            if (Math.abs(turnPower) > 0.03) {
                drivetrain.autoTurn(refHeading, botHeading);
                return true;
            } else {
                //drivetrain.autoTurn(0);
                drivetrain.autoStrafe(0);
                //angleDist = 0;
                return false;
            }
        }
    }

    public Action turn(double aDist) {
        angleDist = aDist;
        return new Turn();
    }

}


