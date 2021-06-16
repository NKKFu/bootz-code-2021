package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@TeleOp(name="Principal", group ="Concept")
public class ConceptVuforiaUltimateGoalNavigationWebcam extends LinearOpMode {

    private BNO055IMU InitImuSensor() {
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

    private DcMotor InitDcMotor(String motorName, DcMotorSimple.Direction direction, DcMotor.RunMode runMode) {
        DcMotor motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setDirection(direction);
        motor.setMode(runMode);
        return motor;
    }

    DcMotor front_left, front_right, back_left, back_right, collector, esteira, shooter_left, shooter_right, pendulo;
    Servo serv0, serv1, serv2, serv3;

    public void DriveMecanum(double robotSpeed, double drive, double strafe, double twist) {
        double[] speeds = {
                robotSpeed * (drive + strafe + twist),
                robotSpeed * (drive - strafe - twist),
                robotSpeed * (drive - strafe + twist),
                robotSpeed * (drive + strafe - twist)
        };

        double max = Math.abs(speeds[0]);
        for (int i = 0; i < speeds.length; i++) {
            if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
        }

        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        front_left.setPower(speeds[0]);
        front_right.setPower(speeds[1]);
        back_left.setPower(speeds[2]);
        back_right.setPower(speeds[3]);
    }

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "ATX1WMH/////AAABmZRCfAn9XENvtus7/8vEDzAjZUqBTtwyQRDZXSSxrKinG+QxvA51PBQf+MmhxVq5dnpIJ5pQcu8NIAo2ZJdWxJis8ws4Mx9efHxDgVdU4bwtKuOPmlUvdtwLHw8QZXpUA3nWj5G6HW2mX/7aE2hpj3sMsjjQKJMd7ify6hnYkVZwx3Ej1mODMH6FQ9UQTQuTJYN4vUGZQ3ggd/Yh+j1n+eldooxfN3G9SGh2eaa2vvfIsAsrZ7yrgHZUzw/fPDFa3MIk6lML0L02lKeksYGXqStHnzVJOiL/v2XNj+LmwpIvwWiijR4eTK3wl6oXjD0NP3i3bO37veiYrA0lTB+MZ18KpHQYiONQQ8GtTaP1eg3r";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    WebcamName webcamName = null;
    private TFObjectDetector tfod;

    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;
    VuforiaTrackables targetsUltimateGoal = null;
    List<VuforiaTrackable> allTrackables;

    @Override
    public void runOpMode() {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;

        try {
            vuforia = ClassFactory.getInstance().createVuforia(parameters);
        } catch (VuforiaException err) {
        }

        if (vuforia != null) {
            targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
            VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
            redTowerGoalTarget.setName("Red Tower Goal Target");

            // For convenience, gather together all the trackable objects in one easily-iterable collection */
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
                tfod.activate();
                tfod.setClippingMargins(1000,500,500,200);
            }

            OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

            for (VuforiaTrackable trackable : allTrackables) {
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
            }
        }

        BNO055IMU imu = InitImuSensor();
        front_left = InitDcMotor("front_left", DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right = InitDcMotor("front_right", DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left = InitDcMotor("back_left", DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right = InitDcMotor("back_right", DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        esteira = InitDcMotor("esteira", DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter_left = InitDcMotor("shooter_left", DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter_right = InitDcMotor("shooter_right", DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pendulo = InitDcMotor("pendulo", DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // RAMPA
        serv0 = hardwareMap.get(Servo.class, "serv0");

        // TRILHO 1
        serv1 = hardwareMap.get(Servo.class, "serv1");

        // TRILHO 2
        serv2 = hardwareMap.get(Servo.class, "serv2");

        // PENDULO
        serv3 = hardwareMap.get(Servo.class, "serv3");

        if (vuforia != null) {
            targetsUltimateGoal.activate();
        }

        boolean rotationLocker = false, shooter = false, esteiraEnabled = false;
        while (!isStopRequested()) {
            double drive = gamepad1.left_stick_y + gamepad2.left_stick_y;
            double strafe = -gamepad1.left_stick_x + (-gamepad2.left_stick_x);
            double twist = -gamepad1.right_stick_x + (-gamepad2.right_stick_x);

            double robotSpeed = gamepad1.right_trigger + gamepad2.right_trigger != 0 ? 1 : 0.5;
            double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            if (rotationLocker)
                twist = -smoothAngleCorrection(currentAngle);

            DriveMecanum(robotSpeed, drive, strafe, twist);

            if (gamepad1.dpad_down || gamepad1.dpad_down) {
                serv0.setPosition(0.5);
            } else if (gamepad1.dpad_up || gamepad2.dpad_up) {
                serv0.setPosition(1);
            }

            if (gamepad1.a || gamepad2.a) {
                while (gamepad1.a || gamepad2.a) {
                    sleep(10);
                }
                esteiraEnabled = !esteiraEnabled;
            }
            if (esteiraEnabled) {
                serv2.setPosition(0);
                serv3.setPosition(1);
            } else {
                serv2.setPosition(0.5);
                serv3.setPosition(0.5);
            }
            esteira.setPower(esteiraEnabled ? -1 : 0);

            if (gamepad1.x || gamepad2.x) {
                while (gamepad1.x || gamepad2.x) {
                    sleep(10);
                }
                rotationLocker = !rotationLocker;
            }

            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                while (gamepad1.left_bumper || gamepad2.left_bumper) {
                    sleep(10);
                }
                shooter = !shooter;
            }

            if (shooter) {
                shooter_left.setPower(-1);
                shooter_right.setPower(1);
            } else {
                shooter_left.setPower(0);
                shooter_right.setPower(0);
            }

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                    }
                }
            }

            // check all the trackable targets to see which one (if any) is visible.
            double x = 0, y = 0;
            if (vuforia != null)
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());

                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getLastTrackedPoseVuforiaCamera().rotated(DEGREES, 0f, 0f, 0f, 0f);
                        if (robotLocationTransform == null)
                            continue;

                        VectorF translation = robotLocationTransform.getTranslation();
                        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                        x = -1 * ((0.5 * (translation.get(2) / mmPerInch)) - 36);
                        y = -1 * ((0.5 * (translation.get(0) / mmPerInch)));

                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                    }
                }

            telemetry.update();
            if (gamepad1.b || gamepad2.b) {
                DriveMecanum(1,
                        smoothAngleCorrection(x * 1),
                        smoothAngleCorrection(y * 1),
                        -smoothAngleCorrection(currentAngle) * 0.8);
            }
            // Disable Tracking when we are done;
            targetsUltimateGoal.deactivate();
        }
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
}
