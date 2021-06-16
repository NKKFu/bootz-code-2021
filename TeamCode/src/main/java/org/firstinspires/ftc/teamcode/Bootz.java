package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Bootz extends LinearOpMode {
    public DcMotor front_left = null, front_right = null, back_left = null,
            back_right = null, collector = null, esteira = null, shooter_left = null,
            shooter_right = null, pendulo = null;
    public Servo serv0 = null, serv1 = null, serv2 = null, serv3 = null;
    public BNO055IMU imu = null;

    public void runOpMode() { }

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

    public double smoothAngleCorrection (double angle) {
        return 1 - Math.pow(1 - (angle / 30), 3);
    }

    public void DriveMecanumBySeconds(double robotSpeed, double drive, double strafe, long timming) {
        for (int i = 0; i < timming; i++) {
            sleep(1);
            double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            DriveMecanum(robotSpeed, drive, strafe, -smoothAngleCorrection(currentAngle));
        }

        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }
}
