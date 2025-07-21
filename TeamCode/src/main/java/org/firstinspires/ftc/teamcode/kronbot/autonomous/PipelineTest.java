package org.firstinspires.ftc.teamcode.kronbot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.kronbot.utils.detection.BluePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.opencv.core.Point;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;


@Autonomous(name = "B+Y Detection Test", group = "Test")
public class PipelineTest extends LinearOpMode {

    private OpenCvCamera camera;
    private BluePipeline pipeline;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);



        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.startCameraStream(camera, 30);



        pipeline = new BluePipeline();
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
        telemetry.addData("Instructions", "Place blue and yellow samples in front of camera");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            int blueCount = pipeline.getBlueCount();
            int yellowCount = pipeline.getYellowCount();
            int totalCount = blueCount + yellowCount;

            Point largestBlueCenter = pipeline.getLargestBlueCenter();
            Point largestYellowCenter = pipeline.getLargestYellowCenter();


            telemetry.addData("=== SAMPLE DETECTION ===", "");
            telemetry.addData("Blue Samples", blueCount);
            telemetry.addData("Yellow Samples", yellowCount);
            telemetry.addData("Total Samples", totalCount);


            if (blueCount > 0) {
                telemetry.addData("Blue Status", "✓ BLUE FOUND");
                if (largestBlueCenter != null) {
                    telemetry.addData("Blue X", "%.1f", largestBlueCenter.x);
                    telemetry.addData("Blue Y", "%.1f", largestBlueCenter.y);
                    telemetry.addData("Blue Position", getPositionDescription(largestBlueCenter.x, 320));
                }
            } else {
                telemetry.addData("Blue Status", "✗ NO BLUE");
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


            telemetry.addData("=== BLUE HSV SETTINGS ===", "");
            telemetry.addData("Blue H Range", "%.1f - %.1f", BluePipeline.BLUE_H_MIN, BluePipeline.BLUE_H_MAX);
            telemetry.addData("Blue S Range", "%.1f - %.1f", BluePipeline.BLUE_S_MIN, BluePipeline.BLUE_S_MAX);
            telemetry.addData("Blue V Range", "%.1f - %.1f", BluePipeline.BLUE_V_MIN, BluePipeline.BLUE_V_MAX);


            telemetry.addData("=== YELLOW HSV SETTINGS ===", "");
            telemetry.addData("Yellow H Range", "%.1f - %.1f", BluePipeline.YELLOW_H_MIN, BluePipeline.YELLOW_H_MAX);
            telemetry.addData("Yellow S Range", "%.1f - %.1f", BluePipeline.YELLOW_S_MIN, BluePipeline.YELLOW_S_MAX);
            telemetry.addData("Yellow V Range", "%.1f - %.1f", BluePipeline.YELLOW_V_MIN, BluePipeline.YELLOW_V_MAX);


            telemetry.addData("=== COMMON SETTINGS ===", "");
            telemetry.addData("Min Area", BluePipeline.MIN_AREA);
            telemetry.addData("Max Area", BluePipeline.MAX_AREA);
            telemetry.addData("Draw Centers", BluePipeline.DRAW_CENTER);

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