package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="Mecanum", group="Main")
public class Mecanum extends LinearOpMode {
    private BNO055IMU InitImuSensor () {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        BNO055IMU configuredImu = hardwareMap.get(BNO055IMU.class, "imu");
        configuredImu.initialize(parameters);
        return configuredImu;
    }

    private DcMotor InitDcMotor (String motorName, DcMotorSimple.Direction direction, DcMotor.RunMode runMode) {
        DcMotor motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setDirection(direction);
        motor.setMode(runMode);
        return motor;
    }

    DcMotor front_left, front_right, back_left, back_right;

    public void DriveMecanum(double robotSpeed, double drive, double strafe, double twist) {
        double[] speeds = {
                robotSpeed * (drive + strafe + twist),
                robotSpeed * (drive - strafe - twist),
                robotSpeed * (drive - strafe + twist),
                robotSpeed * (drive + strafe - twist)
        };

        // Loop through all values in the speeds[] array and find the biggest magnitudes
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        front_left.setPower(speeds[0]);
        front_right.setPower(speeds[1]);
        back_left.setPower(speeds[2]);
        back_right.setPower(speeds[3]);
    }

    @Override public void runOpMode() {
        // Init hardware stuff
        BNO055IMU imu = InitImuSensor();
        front_left    = InitDcMotor("front_left", DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right   = InitDcMotor("front_right", DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left     = InitDcMotor("back_left", DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right    = InitDcMotor("back_right", DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait until we're told to go
        waitForStart();

        // Loop and update the dashboard
        while (opModeIsActive()) {
            // Get user input
            double drive  = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double twist  = -gamepad1.right_stick_x;

            double robotSpeed = gamepad1.right_trigger != 0 ? 1 : 0.5 ;

            DriveMecanum(robotSpeed, drive, strafe, twist);

            while (gamepad1.a) {
                double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                DriveMecanum(1, 0.5, 0, -currentAngle * 0.15);
            }

            ShowTeletryData(imu);
        }
    }

    public void ShowTeletryData (BNO055IMU imu) {
        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        telemetry.addLine()
                .addData("Angle", currentAngle)
                .addData("Left Trigger", gamepad1.right_trigger);

        // Update data on user's screen
        telemetry.update();
    }
}