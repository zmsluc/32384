package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "made by the big e himself")
public class MainAuto extends LinearOpMode {
    DcMotor frontRight, frontLeft, backRight, backLeft, shooter;
    CRServo kickerLeft, kickerRight;
    private boolean aToggle, prevA, bToggle, prevB, yToggle, prevY;
    private double shooterPower, shotWaitTime, reversePower;
    private IMU imu;



    void shoot() {
        shooter.setPower(.6);
        for (int i = 0; i < 3; i++) {
            kickerLeft.setPower(1);
            kickerRight.setPower(1);
            sleep(150);
            kickerLeft.setPower(0);
            kickerRight.setPower(0);
            sleep(300);
        }
    }

    void straferight (int ms) {
        frontLeft.setPower(1);
        backLeft.setPower(-1);
        frontRight.setPower(-1);
        backRight.setPower(1);

    }
    void forward (int ms ) {
        frontLeft.setPower(-.5);
        frontRight.setPower(-.5);
        backLeft.setPower(-.5);
        backRight.setPower(-.5);
    }


        public void runOpMode()  throws InterruptedException {
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            backRight = hardwareMap.get(DcMotor.class, "backRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            shooter = hardwareMap.dcMotor.get("launcher");
            kickerLeft = hardwareMap.get(CRServo.class, "left_feeder");
            kickerRight = hardwareMap.get(CRServo.class, "right_feeder");
            kickerLeft.setDirection(CRServo.Direction.REVERSE);
            kickerRight.setDirection(CRServo.Direction.FORWARD);
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.FORWARD);

            waitForStart();
            forward(600);
            sleep(300);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            shoot();
            sleep(300);
            shoot();
            sleep(300);
            shoot();
            straferight(600);
            sleep(300);



    }
}
