package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaException;

import java.util.ArrayList;
import java.util.List;
import java.lang.Math;
import org.firstinspires.ftc.teamcode.Bootz;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Autonomous(name="Principal Auto x", group ="")
public class Auto extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "ATX1WMH/////AAABmZRCfAn9XENvtus7/8vEDzAjZUqBTtwyQRDZXSSxrKinG+QxvA51PBQf+MmhxVq5dnpIJ5pQcu8NIAo2ZJdWxJis8ws4Mx9efHxDgVdU4bwtKuOPmlUvdtwLHw8QZXpUA3nWj5G6HW2mX/7aE2hpj3sMsjjQKJMd7ify6hnYkVZwx3Ej1mODMH6FQ9UQTQuTJYN4vUGZQ3ggd/Yh+j1n+eldooxfN3G9SGh2eaa2vvfIsAsrZ7yrgHZUzw/fPDFa3MIk6lML0L02lKeksYGXqStHnzVJOiL/v2XNj+LmwpIvwWiijR4eTK3wl6oXjD0NP3i3bO37veiYrA0lTB+MZ18KpHQYiONQQ8GtTaP1eg3r";

    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    WebcamName webcamName = null;
    private TFObjectDetector tfod;

    VuforiaTrackables targetsUltimateGoal = null;
    List<VuforiaTrackable> allTrackables;

    private Bootz bootz;

    @Override
    public void runOpMode() {
        bootz = new Bootz();
        bootz.front_left = InitDcMotor("front_left", DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bootz.front_right = InitDcMotor("front_right", DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bootz.back_left = InitDcMotor("back_left", DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bootz.back_right = InitDcMotor("back_right", DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bootz.esteira = InitDcMotor("esteira", DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bootz.shooter_left = InitDcMotor("shooter_left", DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bootz.shooter_right = InitDcMotor("shooter_right", DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bootz.pendulo = InitDcMotor("pendulo", DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bootz.serv0 = hardwareMap.get(Servo.class, "serv0");
        bootz.serv1 = hardwareMap.get(Servo.class, "serv1");
        bootz.serv2 = hardwareMap.get(Servo.class, "serv2");
        bootz.serv3 = hardwareMap.get(Servo.class, "serv3");

        bootz.pendulo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bootz.pendulo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bootz.pendulo.setTargetPosition(40);
        bootz.pendulo.setPower(1);
        bootz.pendulo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bootz.serv1.setPosition(0);

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        if (vuforia != null) {
            targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
            VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
            redTowerGoalTarget.setName("Red Tower Goal Target");

            allTrackables = new ArrayList<VuforiaTrackable>();
            allTrackables.addAll(targetsUltimateGoal);

            redTowerGoalTarget.setLocation(OpenGLMatrix
                    .translation(halfField, -quadField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

            final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
            final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
            final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

            initTfod();

            if (tfod != null) {
                tfod.setClippingMargins(1000,500,500,200);
                tfod.activate();
            }

            OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

            for (VuforiaTrackable trackable : allTrackables) {
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
            }
        }

        bootz.imu = InitImuSensor();

        bootz.DriveMecanumBySeconds(1,-1,0,100);
        if (vuforia != null) {
            targetsUltimateGoal.activate();
        }
        sleep(1000);

        // Identificar argolas
        String data = "";
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            for (Recognition recognition : updatedRecognitions) {
                data = recognition.getLabel();
                telemetry.addData("# Object Detected", data);
                break;
            }
            telemetry.update();
        }

        bootz.DriveMecanumBySeconds(1,-1,-1,150);

        while (!isStopRequested()) { }

        switch (data) {
            case "":
                bootz.DriveMecanumBySeconds(1,1,0,300);
                bootz.serv3.setPosition(1);
                sleep(700);
                bootz.DriveMecanumBySeconds(1,0,1,100);
                break;
            case "Single":
                break;
            case "Quad":
                break;
        }

        bootz.DriveMecanumBySeconds(0,-1,0,0);
        // Fazer lançamento

        // Guardar o goal pendulo





        // Disable Tracking when we are done;
        targetsUltimateGoal.deactivate();
    }

    public double smoothAngleCorrection (double angle) {
        return 1 - Math.pow(1 - (angle / 30), 3);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public BNO055IMU InitImuSensor() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        BNO055IMU configuredImu = hardwareMap.get(BNO055IMU.class, "imu");
        configuredImu.initialize(parameters);
        return configuredImu;
    }

    public DcMotor InitDcMotor(String motorName, DcMotorSimple.Direction direction, DcMotor.RunMode runMode) {
        DcMotor motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setDirection(direction);
        motor.setMode(runMode);
        return motor;
    }

}
