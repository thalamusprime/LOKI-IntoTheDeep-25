package org.firstinspires.ftc.teamcode.ftc6205.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ftc6205.motors.Drivetrain;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.DeadWheels;

import javax.annotation.CheckForNull;

@Config
public class DriveAction {
    public Drivetrain drivetrain;
    public DeadWheels deadWheels;

    public static double DISTANCE = 24;

    public DriveAction(HardwareMap hardwareMap) {
        deadWheels = new DeadWheels(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);

    }
    public class DriveForward implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            deadWheels.runEncoders();

            if (!initialized) {
                drivetrain.autoForward(0.3);
                initialized = true;
            }

            double pos = deadWheels.encLeftValue;
            packet.put("liftPos", pos);
            if (pos < DriveAction.DISTANCE) {
                return true;
            } else {
                drivetrain.autoForward(0);
                return false;
            }

        }
    }

    public Action driveForward() {
        // calls drive forward
        return new DriveForward();
    }

}


