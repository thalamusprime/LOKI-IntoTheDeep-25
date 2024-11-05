package org.firstinspires.ftc.teamcode.ftc6205.metrics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

//@TeleOp
@Disabled
public class DSTelemetry {
    Telemetry telemetry;
//    @Override
//    public void runOpMode() throws InterruptedException {

    //    DcMotor encoderLeft;
    //    DcMotor encoderBack;
    //    DcMotor encoderRight;
    //
    //    // TELEMETRY
    //
    //    // IMU
    //    //TrueNorth pidControl;
    //    double refHeading, botHeading, pidOutput;
    //    double y, x, rz;
    //    double rotX, rotY;
    //
    //    // SENSORS
    //
    //    // SERVOS
    //    Servo purpDrop; //purp-drop
    //    Servo purpLift; //purp-lift
    //
    //    // ENCODERS
    //    double encLeftValue;
    //    double encBackValue;
    //    double encRightValue;
    //
    //    // AprilTag
    //    AprilTagProcessor tagProcessor;
    //    // VisionPortal
    //    VisionPortal visionPortal;
    // Send telemetry data
    public void sendTelemetry(
            Telemetry telemetry,
            //AprilTagProcessor tagProcessor,
            //VisionPortal visionPortal,
//            double y,
//            double x,
//            double rz,
//            double refHeading,
//            double botHeading,
//            double pidOutput,
//            Servo purpDrop,
            double encLeftValue,
            double encBackValue,
            double encRightValue
            ) throws InterruptedException {
        String servo0 = "";
        String servo1 = "";
        String tag4_x = "";
        String tag4_y = "";
        String tag4_z = "";
        String tag5_x = "";
        String tag5_y = "";
        String tag5_z = "";
        String tag6_x = "";
        String tag6_y = "";
        String tag6_z = "";
        String tag7_x = "";
        String tag7_y = "";
        String tag7_z = "";
        String tag8_x = "";
        String tag8_y = "";
        String tag8_z = "";

//        telemetry.addLine(String.format(
//                "Throttle y:x:rz | %5.2f : %5.2f : %5.2f",
//                y,
//                x,
//                rz));
//        telemetry.addLine(String.format(
//                "REF|BOT|PID %5.2f %5.2f %5.2f",
//                refHeading,
//                botHeading,
//                pidOutput));

//        telemetry.addLine(String.format(
//                "SERVO (0|1) %5.2f,
//                purpDrop.getPosition()
//        ));

//        telemetry.addLine(String.format(
//                "COLOR rgb|a %5.2f %5.2f %5.2f %5.2f",
//                redValue,
//                greenValue,
//                blueValue,
//        ));
//                alphaValue     // light on

        telemetry.addLine(String.format(
                "ENCODER L|B|R %5.2f %5.2f %5.2f",
                encLeftValue * 0.003, // 0.0075
                encBackValue * 0.003,
                encRightValue * 0.003
        ));

        //Vision Processing
//        int tags = tagProcessor.getDetections().size();
//        if (tagProcessor.getDetections().size() > 0) {
//            ArrayList tagList = tagProcessor.getDetections();
//            if (tagList != null) {
//                for (Object tagItem: tagList) {
//                    AprilTagDetection tag = (AprilTagDetection) tagItem;
//                    if (tag.id == 4) {
//                        tag4_x = String.format("%7.2f", tag.ftcPose.x);
//                        tag4_y = String.format("%7.2f", tag.ftcPose.y);
//                        tag4_z = String.format("%7.2f", tag.ftcPose.z);
//                    }
//                    if (tag.id == 5) {
//                        tag5_x = String.format("%7.2f", tag.ftcPose.x);
//                        tag5_y = String.format("%7.2f", tag.ftcPose.y);
//                        tag5_z = String.format("%7.2f", tag.ftcPose.z);
//                    }
//                    if (tag.id == 6) {
//                        tag6_x = String.format("%7.2f", tag.ftcPose.x);
//                        tag6_y = String.format("%7.2f", tag.ftcPose.y);
//                        tag6_z = String.format("%7.2f", tag.ftcPose.z);
//                    }
//                    if (tag.id == 7) {
//                        tag7_x = String.format("%7.2f", tag.ftcPose.x);
//                        tag7_y = String.format("%7.2f", tag.ftcPose.y);
//                        tag7_z = String.format("%7.2f", tag.ftcPose.z);
//                    }
//                    if (tag.id == 8) {
//                        tag8_x = String.format("%7.2f", tag.ftcPose.x);
//                        tag8_y = String.format("%7.2f", tag.ftcPose.y);
//                        tag8_z = String.format("%7.2f", tag.ftcPose.z);
//                    }
//                }
//            }
////            telemetry.addLine("---");
////            telemetry.addLine("=== BLUE");
////            telemetry.addLine("ID (1)" + tag1_x +  tag1_y + tag1_z);
////            telemetry.addLine("ID (2)" + tag2_x +  tag2_y + tag2_y);
////            telemetry.addLine("ID (3)" + tag3_x +  tag3_y + tag3_z);
////            telemetry.addLine("---");
////            telemetry.addLine("ID (9)" + tag9_x +  tag9_y + tag9_z);
////            telemetry.addLine("ID (10)" + tag10_x +  tag10_y + tag10_z);
////            telemetry.addLine("---");
//
//            telemetry.addLine("---");
//            telemetry.addLine("=== RED");
//            telemetry.addLine("ID (4)" + tag4_x +  tag4_y + tag4_z);
//            telemetry.addLine("ID (5)" + tag5_x +  tag5_y + tag5_z);
//            telemetry.addLine("ID (6)" + tag6_x +  tag6_y + tag6_z);
//            telemetry.addLine("---");
//            telemetry.addLine("ID (7)" + tag7_x +  tag7_y + tag7_z);
//            telemetry.addLine("ID (8)" + tag8_x +  tag8_y + tag8_z);
//        }
        telemetry.update();
    }}
