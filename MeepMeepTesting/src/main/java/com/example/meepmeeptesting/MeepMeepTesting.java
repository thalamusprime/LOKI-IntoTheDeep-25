package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static final double A = 60;
    public static final double B = 36;
    public static final double C = 12;
    public static final double D = -12;
    public static final double E = -36;
    public static final double F = -60;
    public static final double VI = 60;
    public static final double V = 36;
    public static final double IV = 12;
    public static final double III = -12;
    public static final double II = -36;
    public static final double I = -60;

    public static double DISTANCE = 36;
    public static double START_ANGLE = Math.toRadians(-90);
    public static double START_X = II;
    public static double START_Y = A;
    //public static String TOKEN_LOCATION = "LEFT";
    //SampleMecanumDrive drive;
    Pose2d startPose = new Pose2d(START_X, START_Y, START_ANGLE);
    //ElapsedTime timer = new ElapsedTime();
    public static String startPosition = "A3";
    //public static String startPosition = "A5";
    //public static String startPosition = "F2";
    //public static String startPosition = "F4";
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(640);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // A2
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(F, II, -90))
                .lineToX(23.5)
                .turn(Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
