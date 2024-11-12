package org.firstinspires.ftc.teamcode.ftc6205.controllers;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ftc6205.constants.DrivePIDConstants;

//@TeleOp(name = "PID Control")
//@Disabled
public class TrueNorth {
    double integralSum = 0;
    double Kp = DrivePIDConstants.Kp;
    double Ki = DrivePIDConstants.Ki;
    double Kd = DrivePIDConstants.Kd;
    double Kp_d = DrivePIDConstants.Kp;
    double Ki_d = DrivePIDConstants.Ki;
    double Kd_d = DrivePIDConstants.Kd;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    //private BNO055IMU imu;

    public double PIDControl(double reference, double state) {
        double error = angleWrap(reference - state);
        //telemetry.addData("Error: ", error);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

    public double PIDDistance(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

    private double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }

}