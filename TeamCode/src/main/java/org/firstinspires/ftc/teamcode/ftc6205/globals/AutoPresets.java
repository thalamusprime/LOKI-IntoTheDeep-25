package org.firstinspires.ftc.teamcode.ftc6205.globals;
import com.acmerobotics.dashboard.config.Config;

@Config
public class AutoPresets {
    // Claw
    public static double claw_pinch = 0.0; // pinch
    public static double claw_release = 0.1; // drop
    public static int touch_duration = 20;

    // Wrist
    public static double wrist_inc = 0.005;
    public static double wrist_start = 0.0;  //0.85;
    public static double wrist_floor = 0.458;
    public static double wrist_wall = 0.751; //0.265;
    public static double wrist_stage = 0.680;
    public static double wrist_hook = 0.680;
    public static double wrist_pinch = 0.800;

    //Shoulder
    public static double shoulder_inc = 0.001;
    public static double shoulder_start = 0.0;
    public static double shoulder_floor = 0.238;
    public static double shoulder_wall = 0.219;
    public static double shoulder_specimen_stage = 0.137; //0.1 straight up
    public static double shoulder_specimen_hook = 0.177; //0.1 straight up
    public static double shoulder_pinch = 0.220;

}

