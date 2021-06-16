package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@Autonomous(name="Look Forward", group="Teste")
public class BasicOpMode_Iterative extends LinearOpMode
{
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

    double angleTarget = 0;
    @Override public void runOpMode() {
        // Init hardware stuff
        BNO055IMU imu = InitImuSensor();
        DcMotor leftMotor = InitDcMotor("left_drive", DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotor rightMotor = InitDcMotor("right_drive", DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        // imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Loop and update the dashboard
        while (opModeIsActive()) {
            // Get values from IMU sensor
            if (gamepad1.left_bumper) {
                while (gamepad1.left_bumper) {}
                angleTarget = angleTarget - 90;
            }
            if (gamepad1.right_bumper) {
                while (gamepad1.right_bumper) {}
                imuOffset = imuOffset + 90;
            }

            // Calculates differences between current and target angles
            double currentAngle = GetCurrentAngleFromIMU(imu);

            double differenceBetweenCurrentAndTarget =
                    currentAngle - angleTarget;

            // Only five per cent of difference
            double forcesToApplyAndGetRightAngle =  differenceBetweenCurrentAndTarget * 0.1;

            // Apply forces
            leftMotor.setPower(0.4 + forcesToApplyAndGetRightAngle * -1);
            rightMotor.setPower(0.4 + forcesToApplyAndGetRightAngle);

            telemetry.addData("current", angleTarget);
            telemetry.update();
        }
    }

    double imuOffset = -179;
    public double GetCurrentAngleFromIMU (BNO055IMU imu) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return imuOffset + AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    public double FormatAngleValue (double angleValue) {
        if (angleValue >= 180)
            return angleValue - 360;
        if (angleValue <= -180)
            return angleValue + 360;
        return angleValue;
    }
}
