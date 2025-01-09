package org.firstinspires.ftc.teamcode.ftc6205.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode._auto.BaseAuto;

public class LiftAction {
    private DcMotorEx lift;

    public LiftAction(HardwareMap hardwareMap) {
        lift = hardwareMap.get(DcMotorEx.class, "longArm");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public class LiftUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                lift.setPower(0.4);
                initialized = true;
            }

            double pos = lift.getCurrentPosition();
            packet.put("liftPos", pos);
            //telemetry.addData("LIFT: ", String.valueOf(pos));
            if (pos < 3000.0) {
                return true;
            } else {
                lift.setPower(0);
                return false;
            }

        }
    }

    public Action liftUp() {
        return new LiftUp();
    }
    public class LiftDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                lift.setPower(-0.8);
                initialized = true;
            }

            double pos = lift.getCurrentPosition();
            packet.put("liftPos", pos);
            if (pos > 100.0) {
                return true;
            } else {
                lift.setPower(0);
                return false;
            }
        }
    }
    public Action liftDown(){
        return new LiftDown();
    }
}


