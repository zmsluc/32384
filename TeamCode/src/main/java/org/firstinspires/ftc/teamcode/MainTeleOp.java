package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "grook squat TeleOp", group = "32384 Code")
public class MainTeleOp extends LinearOpMode {
    // Batch declares the variables used later in the code
    DcMotor frontRight, frontLeft, backRight, backLeft, shooter;
    private IMU imu;
    CRServo kickerLeft, kickerRight;
    private boolean aToggle, prevA, bToggle, prevB, yToggle, prevY;
    private double shooterPower, shotWaitTime, reversePower;

    ElapsedTime shotTimer = new ElapsedTime(); // Timer for auto-shoot
    @Override
    public void runOpMode() throws InterruptedException {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        backRight=hardwareMap.get(DcMotor.class,"backRight");
        backLeft=hardwareMap.get(DcMotor.class,"backLeft");
        shooter = hardwareMap.dcMotor.get("launcher");
        kickerLeft = hardwareMap.get(CRServo.class, "left_feeder");
        kickerRight = hardwareMap.get(CRServo.class, "right_feeder");
        kickerLeft.setDirection(CRServo.Direction.REVERSE);
        kickerRight.setDirection(CRServo.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
       double y = -gamepad1.left_stick_y;
       double x = gamepad1.left_stick_x*1.1;
               double rx = gamepad1.right_stick_x;
        frontLeft.setPower(y + x + rx);
        backLeft.setPower(y - x + rx);
        frontRight.setPower(y - x - rx);
        backRight.setPower(y + x - rx);
       imu=hardwareMap.get(IMU.class,"imu");




        waitForStart();
        aToggle = prevA = bToggle = prevB = yToggle = prevY = false; // Defaults all toggles to false
        shooterPower = 1.5; // Sets the power of the shooter motor
        reversePower = -.75;

        while (opModeIsActive()) {
            // Inside of the opModeIsActive loop, all code loops every ~5-20ms (50-200 times per second)

            // Reference variables for gamepad joystick power
            double leftY = -gamepad1.left_stick_y; // leftY is flipped to make the joystick power behave as expected
            double leftX = -gamepad1.left_stick_x;
            double rightX = -gamepad1.right_stick_x;

            frontLeft.setPower(leftY - leftX - rightX);
            frontRight.setPower(leftY + leftX + rightX);
            backLeft.setPower(leftY + leftX - rightX);
            backRight.setPower(leftY - leftX + rightX);

            // Updates A and B toggles for kicker and shooter.
            // e.g., if aToggle is true, pressing A will set it false; another press will set it true.
            if (gamepad1.a && !prevA) { // Toggle variable for button A
                aToggle = !aToggle;
            } prevA = gamepad1.a; // Updates A state

            if (gamepad1.y && !prevY) { // Toggle variable for button Y
                yToggle = !yToggle;
            } prevY = gamepad1.y; // Updates Y state

            if (gamepad1.b && !prevB) { // Toggle variable for button B
                bToggle = !bToggle;
            } prevB = gamepad1.b; // Updates B state

            // Toggles shooter and kicker based off of A and B toggles
            if (aToggle) { // If A is toggled, the kicker motors turn on; if it is not, they turn off.
                shooter.setPower(shooterPower);
                shotTimer.reset();
            } else {
                shooter.setPower(0);
            }
            if (yToggle) { // If A is toggled, the kicker motors turn on; if it is not, they turn off.
                shooter.setPower(reversePower);
                kickerLeft.setPower(reversePower);
                kickerRight.setPower(reversePower);
                shotTimer.reset();
            } else {
                shooter.setPower(0);
            }
            if (bToggle) { // If B is toggled, the kicker motors turn on; if it is not, they turn off.
                kickerLeft.setPower(1);
                kickerRight.setPower(1);

            } else {
                kickerLeft.setPower(0);
                kickerRight.setPower(0);
            }

            // Sets the left and right motor powers based off of the reference variables
//            driveLeft.setPower(leftY+rightX);
//            driveRight.setPower(leftY-right


        }
    }


    }

