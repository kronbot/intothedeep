package org.firstinspires.ftc.teamcode.kronbot.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.kronbot.utils.detection.RedPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.opencv.core.Point;

@Autonomous(name = "R+Y Detection Test", group = "Test")
public class PipelineTest2 extends LinearOpMode {

    private OpenCvCamera camera;
    private RedPipeline pipeline;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.startCameraStream(camera, 30);


        pipeline = new RedPipeline();
        camera.setPipeline(pipeline);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", "Error code: " + errorCode);
                telemetry.update();
            }
        });

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Place red and yellow samples in front of camera");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            int redCount = pipeline.getRedCount();
            int yellowCount = pipeline.getYellowCount();
            int totalCount = redCount + yellowCount;

            Point largestRedCenter = pipeline.getLargestRedCenter();
            Point largestYellowCenter = pipeline.getLargestYellowCenter();


            telemetry.addData("=== SAMPLE DETECTION ===", "");
            telemetry.addData("Red Samples", redCount);
            telemetry.addData("Yellow Samples", yellowCount);
            telemetry.addData("Total Samples", totalCount);


            if (redCount > 0) {
                telemetry.addData("Red Status", "✓ RED FOUND");
                if (largestRedCenter != null) {
                    telemetry.addData("Red X", "%.1f", largestRedCenter.x);
                    telemetry.addData("Red Y", "%.1f", largestRedCenter.y);
                    telemetry.addData("Red Position", getPositionDescription(largestRedCenter.x, 320));
                }
            } else {
                telemetry.addData("Red Status", "✗ NO RED");
            }


            if (yellowCount > 0) {
                telemetry.addData("Yellow Status", "✓ YELLOW FOUND");
                if (largestYellowCenter != null) {
                    telemetry.addData("Yellow X", "%.1f", largestYellowCenter.x);
                    telemetry.addData("Yellow Y", "%.1f", largestYellowCenter.y);
                    telemetry.addData("Yellow Position", getPositionDescription(largestYellowCenter.x, 320));
                }
            } else {
                telemetry.addData("Yellow Status", "✗ NO YELLOW");
            }


            telemetry.addData("=== RED HSV SETTINGS ===", "");
            telemetry.addData("Red H Range 1", "%.1f - %.1f", RedPipeline.RED_H_MIN_1, RedPipeline.RED_H_MAX_1);
            telemetry.addData("Red H Range 2", "%.1f - %.1f", RedPipeline.RED_H_MIN_2, RedPipeline.RED_H_MAX_2);
            telemetry.addData("Red S Range", "%.1f - %.1f", RedPipeline.RED_S_MIN, RedPipeline.RED_S_MAX);
            telemetry.addData("Red V Range", "%.1f - %.1f", RedPipeline.RED_V_MIN, RedPipeline.RED_V_MAX);


            telemetry.addData("=== YELLOW HSV SETTINGS ===", "");
            telemetry.addData("Yellow H Range", "%.1f - %.1f", RedPipeline.YELLOW_H_MIN, RedPipeline.YELLOW_H_MAX);
            telemetry.addData("Yellow S Range", "%.1f - %.1f", RedPipeline.YELLOW_S_MIN, RedPipeline.YELLOW_S_MAX);
            telemetry.addData("Yellow V Range", "%.1f - %.1f", RedPipeline.YELLOW_V_MIN, RedPipeline.YELLOW_V_MAX);


            telemetry.addData("=== COMMON SETTINGS ===", "");
            telemetry.addData("Min Area", RedPipeline.MIN_AREA);
            telemetry.addData("Max Area", RedPipeline.MAX_AREA);
            telemetry.addData("Draw Centers", RedPipeline.DRAW_CENTER);

            telemetry.update();

            sleep(100);
        }


        camera.stopStreaming();
        camera.closeCameraDevice();
    }

    private String getPositionDescription(double x, int imageWidth) {
        double center = imageWidth / 2.0;
        double threshold = imageWidth * 0.2;

        if (x < center - threshold) {
            return "LEFT";
        } else if (x > center + threshold) {
            return "RIGHT";
        } else {
            return "CENTER";
        }
    }
}