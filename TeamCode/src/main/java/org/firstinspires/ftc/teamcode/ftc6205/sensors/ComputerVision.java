package org.firstinspires.ftc.teamcode.ftc6205.sensors;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

public class ComputerVision {
    public OpenCvCamera controlHubCam;
    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal;

    HardwareMap hwMap;

    public ComputerVision(HardwareMap ahwMap){
        initAprilTag();
        initVision(ahwMap);
    }
    private void initAprilTag() {
        // Tag Processing
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
    }

    private void initVision(HardwareMap ahwMap) {
        hwMap = ahwMap;
        // VisionPortal
        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1920, 1080))
                .build();
    }

}
