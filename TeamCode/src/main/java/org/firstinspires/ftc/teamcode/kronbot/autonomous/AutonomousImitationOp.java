package org.firstinspires.ftc.teamcode.kronbot.autonomous;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.MILLIAMPS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;

import ai.onnxruntime.*;

import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.ControlHubGyroscope;
import org.json.JSONObject;
import org.json.JSONException;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.Collections;


@Autonomous(name = "Autonomous with Imitation Learning", group = Constants.MAIN_GROUP)
public class AutonomousImitationOp extends LinearOpMode {
    private final KronBot robot = new KronBot();

    private static final String MODEL_PATH = "/sdcard/behavior_cloning_model_test.onnx";
    private static final String FEATURE_SCALER_PATH = "/sdcard/feature_scaler.json";
    private static final String OUTPUT_SCALER_PATH = "/sdcard/model_scaler_test.json";

    private OrtEnvironment env;
    private OrtSession session;

    private float[] featureMean;
    private float[] featureScale;
    private float[] outputMean;
    private float[] outputScale;


    private void loadModelAndScalers() throws IOException, OrtException, JSONException {
        // Load ONNX model
        env = OrtEnvironment.getEnvironment();
        session = env.createSession(MODEL_PATH, new OrtSession.SessionOptions());

        // Load scalers
        featureMean = loadScaler(FEATURE_SCALER_PATH, "mean");
        featureScale = loadScaler(FEATURE_SCALER_PATH, "scale");
        outputMean = loadScaler(OUTPUT_SCALER_PATH, "mean");
        outputScale = loadScaler(OUTPUT_SCALER_PATH, "scale");

        telemetry.addLine("Model and Scalers Loaded Successfully");
        telemetry.update();
    }

    private String readFile(String filePath) throws IOException {
        try (BufferedReader reader = new BufferedReader(
                new InputStreamReader(new FileInputStream(filePath), StandardCharsets.UTF_8))) {
            StringBuilder stringBuilder = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) {
                stringBuilder.append(line);
            }
            return stringBuilder.toString();
        }
    }

    private float[] loadScaler(String filePath, String key) throws IOException, JSONException {
        String jsonStr = readFile(filePath);
        JSONObject scalerData = new JSONObject(jsonStr);
        return parseJsonArray(scalerData.getJSONArray(key));
    }

    private float[] parseJsonArray(org.json.JSONArray jsonArray) throws JSONException {
        float[] result = new float[jsonArray.length()];
        for (int i = 0; i < jsonArray.length(); i++) {
            result[i] = (float) jsonArray.getDouble(i);
        }
        return result;
    }

    private float[] scaleFeatures(float[] inputFeatures) {
        float[] scaledFeatures = new float[inputFeatures.length];
        for (int i = 0; i < inputFeatures.length; i++) {
            scaledFeatures[i] = (inputFeatures[i] - featureMean[i]) / featureScale[i];
        }
        return scaledFeatures;
    }

    private float[] denormalizeOutputs(float[] normalizedOutputs) {
        float[] outputs = new float[normalizedOutputs.length];
        for (int i = 0; i < normalizedOutputs.length; i++) {
            outputs[i] = normalizedOutputs[i] * outputScale[i] + outputMean[i];
        }
        return outputs;
    }

    private void applyMotorPowers(float[] powers) {
        robot.motors.leftRear.setPower(powers[0]);
        robot.motors.rightRear.setPower(powers[1]);
        robot.motors.leftFront.setPower(powers[2]);
        robot.motors.rightFront.setPower(powers[3]);
    }

    @Override
    public void runOpMode() {
        try {
            telemetry.addLine("Before hardware initialization...");
            telemetry.update();

            robot.initSimpleDriving(hardwareMap);
            robot.gyroscope.updateOrientation();
            telemetry.addData("Gyro Heading", robot.gyroscope.getHeading());
            telemetry.update();
            /*robot.initSimpleDriving(hardwareMap);*/

            telemetry.addLine("Hardware initialized");
            telemetry.update();

            loadModelAndScalers();

            telemetry.addLine("Model and scalers loaded");
            telemetry.update();

            while (!opModeIsActive() && !isStopRequested()) {
                telemetry.update();
            }

            waitForStart();

            telemetry.addLine("OpMode started");
            telemetry.update();

            if (isStopRequested()) return;

            while (opModeIsActive() && !isStopRequested()) {
                telemetry.addLine("Collecting sensor data...");
                telemetry.update();


                // Collect sensor data
                float[] inputFeatures = new float[]{
                        (float) getRuntime(),
                        (float) robot.motors.leftRear.getCurrent(MILLIAMPS),
                        (float) robot.motors.rightRear.getCurrent(MILLIAMPS),
                        (float) robot.motors.leftFront.getCurrent(MILLIAMPS),
                        (float) robot.motors.rightFront.getCurrent(MILLIAMPS),
                        (float) -robot.motors.leftFront.getCurrentPosition(),
                        (float) robot.motors.rightFront.getCurrentPosition(),
                        (float) hardwareMap.voltageSensor.iterator().next().getVoltage(),
                        (float) robot.gyroscope.getHeading()
                };

                telemetry.addLine("Sensor data collected");
                telemetry.update();

                // Scale input features
                float[] scaledInput = scaleFeatures(inputFeatures);

                telemetry.addLine("Input features scaled");
                telemetry.update();

                // Run ONNX model
                try (OnnxTensor inputTensor = OnnxTensor.createTensor(env, new float[][]{scaledInput})) {
                    try (OrtSession.Result result = session.run(Collections.singletonMap("input", inputTensor))) {
                        float[][] outputArray = (float[][]) result.get(0).getValue();  // Fix: Extract first row properly
                        float[] normalizedOutputs = outputArray[0]; // Fix: Access first row

                        float[] predictedPowers = denormalizeOutputs(normalizedOutputs);

                        telemetry.addLine("Model prediction successful");
                        telemetry.update();

                        applyMotorPowers(predictedPowers);
                    }
                }


                // Update telemetry
                telemetry.addData("Runtime", getRuntime());
                telemetry.addData("Voltage", inputFeatures[7]);
                telemetry.addData("Heading", inputFeatures[8]);
                telemetry.update();
            }

        } catch (Exception e) {
            telemetry.addLine("Critical Error: " + e.getMessage());
            telemetry.update();
            e.printStackTrace();
        } finally {
            try {
                if (session != null) session.close();
                if (env != null) env.close();
            } catch (Exception e) {
                telemetry.addLine("Error closing ONNX resources: " + e.getMessage());
                telemetry.update();
            }
        }
    }
}
