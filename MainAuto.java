package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="MainAuto", group="Linear OpMode")
public class MainAuto extends LinearOpMode{

    public DcMotor RFMotor = null;
    public DcMotor LFMotor = null;
    public DcMotor LBMotor = null;
    public DcMotor RBMotor = null;

    public DcMotor arm2 = null;
    public CRServo dualModeServo = null;
    public DcMotor pivot1 = null;
    public DcMotor pivot2 = null;

    public final double slowDownStart = 0.5;
    public final double minPower = 0.15;

    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.78;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() {

        RFMotor = hardwareMap.get(DcMotor.class, "motor0");
        LFMotor = hardwareMap.get(DcMotor.class, "motor1");
        LBMotor = hardwareMap.get(DcMotor.class, "motor2");
        RBMotor = hardwareMap.get(DcMotor.class, "motor3");

        arm2 = hardwareMap.get(DcMotor.class, "motor4");
        pivot1 = hardwareMap.get(DcMotor.class, "motor5");
        pivot2 = hardwareMap.get(DcMotor.class, "motor6");
        dualModeServo = hardwareMap.get(CRServo.class, "servo1");

        RFMotor.setDirection(DcMotor.Direction.FORWARD);
        LFMotor.setDirection(DcMotor.Direction.REVERSE);
        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        RBMotor.setDirection(DcMotor.Direction.FORWARD);

        resetEncoders();



        waitForStart();

          if (opModeIsActive()) {

              moveForward(40, 1);

              resetEncoders();

              pivotSlide(1150, 1);

              resetEncoders();

              extendSlide(1700, 0.9);

              resetEncoders();

              moveForward(4, 0.5);

              resetEncoders();

              rotateServo(-1, 1000);

              moveBackward(-4, 0.2);

              resetEncoders();

              retractSlide(4000, 0.3);

              resetEncoders();

             /* strafeLeft(30, 1);

              resetEncoders();

              spinLeft(-21.5, 0.5);

              resetEncoders();

              pivotSlide(134, 0.1);

              resetEncoders();

              rotateServo(1, 5000);

              while (dualModeServo.getPower() != 0) {
                  extendSlide(500, 0.2);
              }

              resetEncoders();

              pivotSlide(-134, 0.5);

              resetEncoders();

              spinLeft(-10.75, 0.5);

              resetEncoders();

              moveForward(30, 0.7);

              while (RFMotor.isBusy() && LFMotor.isBusy() && LBMotor.isBusy() && RBMotor.isBusy()) {
                  extendSlide(4000, 0.4);
              }

              resetEncoders();

              moveForward(5, 0.4);

              resetEncoders();

              rotateServo(-1, 1000);

              moveBackward(-5, 0.4);

              resetEncoders();

              retractSlide(4000, 0.1);

              resetEncoders();

              spinRight(10.75, 0.4);

              resetEncoders();

              strafeRight(30, 1);

              resetEncoders();

              pivotSlide(134, 0.1);

              resetEncoders();

              rotateServo(1, 5000);

              while (dualModeServo.getPower() != 0) {
                  moveForward(3, 0.4);
              }

              resetEncoders();

              pivotSlide(-134, 0.5);

              resetEncoders();

              spinLeft(-10.75, 0.5);

              resetEncoders();

              moveForward(30, 0.4);

              resetEncoders();

              extendSlide(4000, 0.3);

              resetEncoders();

              moveForward(5, 0.2);

              resetEncoders();

              rotateServo(-1, 1000);

              moveBackward(-5, 0.2);

              resetEncoders();

              retractSlide(4000, 0.1);

              resetEncoders();

              spinRight(10.75, 0.4);

              resetEncoders();

              strafeRight(30, 1);

              resetEncoders();

              rotateServo(1, 5000);

              while(dualModeServo.getPower() != 0) {
                  moveForward(3, 0.4);
              }

              resetEncoders();

              pivotSlide(-134, 0.2);

              resetEncoders();

              spinLeft(-15.75, 0.5);

              resetEncoders();

              moveForward(30, 0.4);

              resetEncoders();

              extendSlide(4000, 0.3);

              resetEncoders();

              moveForward(5, 0.2);

              resetEncoders();

              rotateServo(-1, 1000);

              moveBackward(-5, 0.2);

              resetEncoders();

              retractSlide(4000, 0.1);

              resetEncoders();

              spinLeft(-27.25, 0.3);

              resetEncoders();

              moveForward(110, 1);

              resetEncoders();

              strafeRight(10, 0.5);*/

          }
    }

    public void resetEncoders() {
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void extendSlide(int ticks, double power) {
        arm2.setTargetPosition(arm2.getCurrentPosition() - ticks);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setPower(-power);

        while (opModeIsActive() && arm2.isBusy()) {
            telemetry.addData("Extending Slide", "Current Position: %d", arm2.getCurrentPosition());
            telemetry.update();
        }

        arm2.setPower(0);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void retractSlide(int ticks, double power) {
        arm2.setTargetPosition(arm2.getCurrentPosition() + ticks);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setPower(power);

        while (opModeIsActive() && arm2.isBusy()) {
            telemetry.addData("Retracting Slide", "Current Position: %d", arm2.getCurrentPosition());
            telemetry.update();
        }

        arm2.setPower(0);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void pivotSlide(int ticks, double power) {
        pivot1.setTargetPosition(pivot1.getCurrentPosition() + ticks);
        pivot2.setTargetPosition(pivot2.getCurrentPosition() + ticks);

        pivot1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivot2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pivot1.setPower(power);
        pivot2.setPower(power);

        while (opModeIsActive() && (pivot1.isBusy() && pivot2.isBusy())) {
            telemetry.addData("Pivoting Slide", "Pivot1: %d, Pivot2: %d", pivot1.getCurrentPosition(), pivot2.getCurrentPosition());
            telemetry.update();
        }

        pivot1.setPower(0);
        pivot2.setPower(0);

        pivot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void rotateServo(double power, int durationMs) {
        dualModeServo.setPower(power);

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < durationMs) {
            telemetry.addData("Rotating Servo", "Power: %.2f, Time: %d ms", power, timer.milliseconds());
            telemetry.update();
        }

        dualModeServo.setPower(0);
    }


    public void moveForward(double inches, double maxSpeed) {

        RFMotor.setPower(0);
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);

        int newRFTarget = RFMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newLFTarget = LFMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newLBTarget = LBMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newRBTarget = RBMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

        RFMotor.setTargetPosition(newRFTarget);
        LFMotor.setTargetPosition(newLFTarget);
        LBMotor.setTargetPosition(newLBTarget);
        RBMotor.setTargetPosition(newRBTarget);

        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power;

        while (opModeIsActive() && (RFMotor.isBusy() && LFMotor.isBusy() && LBMotor.isBusy() && RBMotor.isBusy())) {

            int distanceRemaining = Math.min(
            Math.abs(newRFTarget - RFMotor.getCurrentPosition()),
            Math.min(Math.abs(newLFTarget - LFMotor.getCurrentPosition()),
            Math.min(Math.abs(newLBTarget - LBMotor.getCurrentPosition()),
            Math.abs(newRBTarget - RBMotor.getCurrentPosition())))
            );

            if (distanceRemaining > (slowDownStart * Math.abs(inches * COUNTS_PER_INCH))) {
                power = maxSpeed;
            } else {
                power = maxSpeed * (distanceRemaining / (double) (Math.abs(inches * COUNTS_PER_INCH)));
            }

            power = Math.max(power, minPower);

            RFMotor.setPower(power);
            LFMotor.setPower(power);
            LBMotor.setPower(power);
            RBMotor.setPower(power);


            telemetry.addData("Target", "Running to %7d :%7d :%7d :%7d", newRFTarget, newLFTarget, newLBTarget, newRBTarget);
            telemetry.addData("Current", "Running at %7d :%7d :%7d :%7d", RFMotor.getCurrentPosition(), LFMotor.getCurrentPosition(), LBMotor.getCurrentPosition(), RBMotor.getCurrentPosition());
            telemetry.addData("Power", "Power: %5.2f", power);
            telemetry.update();
            }

        RFMotor.setPower(0);
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);
    }

    public void moveBackward(double inches, double maxSpeed) {

        RFMotor.setPower(0);
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);

        int newRFTarget = RFMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newLFTarget = LFMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newLBTarget = LBMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newRBTarget = RBMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

        RFMotor.setTargetPosition(newRFTarget);
        LFMotor.setTargetPosition(newLFTarget);
        LBMotor.setTargetPosition(newLBTarget);
        RBMotor.setTargetPosition(newRBTarget);

        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power;

        while (opModeIsActive() && (RFMotor.isBusy() && LFMotor.isBusy() && LBMotor.isBusy() && RBMotor.isBusy())) {

            int distanceRemaining = Math.min(
                    Math.abs(newRFTarget - RFMotor.getCurrentPosition()),
                    Math.min(Math.abs(newLFTarget - LFMotor.getCurrentPosition()),
                            Math.min(Math.abs(newLBTarget - LBMotor.getCurrentPosition()),
                                    Math.abs(newRBTarget - RBMotor.getCurrentPosition())))
            );

            if (distanceRemaining > (slowDownStart * Math.abs(inches * COUNTS_PER_INCH))) {
                power = maxSpeed;
            } else {
                power = maxSpeed * (distanceRemaining / (double) (Math.abs(inches * COUNTS_PER_INCH)));
            }

            power = Math.max(power, minPower);

            RFMotor.setPower(power);
            LFMotor.setPower(power);
            LBMotor.setPower(power);
            RBMotor.setPower(power);


            telemetry.addData("Target", "Running to %7d :%7d :%7d :%7d", newRFTarget, newLFTarget, newLBTarget, newRBTarget);
            telemetry.addData("Current", "Running at %7d :%7d :%7d :%7d", RFMotor.getCurrentPosition(), LFMotor.getCurrentPosition(), LBMotor.getCurrentPosition(), RBMotor.getCurrentPosition());
            telemetry.addData("Power", "Power: %5.2f", power);
            telemetry.update();
        }

        RFMotor.setPower(0);
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);
    }

    public void strafeRight(double inches, double maxSpeed) {

        RFMotor.setPower(0);
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);

        int newRFTarget = RFMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        int newLFTarget = LFMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newLBTarget = LBMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        int newRBTarget = RBMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

        RFMotor.setTargetPosition(newRFTarget);
        LFMotor.setTargetPosition(newLFTarget);
        LBMotor.setTargetPosition(newLBTarget);
        RBMotor.setTargetPosition(newRBTarget);

        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power;

        while (opModeIsActive() && (RFMotor.isBusy() && LFMotor.isBusy() && LBMotor.isBusy() && RBMotor.isBusy())) {

            int distanceRemaining = Math.min(
                    Math.abs(newRFTarget - RFMotor.getCurrentPosition()),
                    Math.min(Math.abs(newLFTarget - LFMotor.getCurrentPosition()),
                            Math.min(Math.abs(newLBTarget - LBMotor.getCurrentPosition()),
                                    Math.abs(newRBTarget - RBMotor.getCurrentPosition())))
            );

            if (distanceRemaining > (slowDownStart * Math.abs(inches * COUNTS_PER_INCH))) {
                power = maxSpeed;
            } else {
                power = maxSpeed * (distanceRemaining / (double) (Math.abs(inches * COUNTS_PER_INCH)));
            }

            power = Math.max(power, minPower);

            RFMotor.setPower(power);
            LFMotor.setPower(power);
            LBMotor.setPower(power);
            RBMotor.setPower(power);


            telemetry.addData("Target", "Running to %7d :%7d :%7d :%7d", newRFTarget, newLFTarget, newLBTarget, newRBTarget);
            telemetry.addData("Current", "Running at %7d :%7d :%7d :%7d", RFMotor.getCurrentPosition(), LFMotor.getCurrentPosition(), LBMotor.getCurrentPosition(), RBMotor.getCurrentPosition());
            telemetry.addData("Power", "Power: %5.2f", power);
            telemetry.update();
        }

        RFMotor.setPower(0);
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);
    }

    public void strafeLeft(double inches, double maxSpeed) {

        RFMotor.setPower(0);
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);

        int newRFTarget = RFMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newLFTarget = LFMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        int newLBTarget = LBMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newRBTarget = RBMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);

        RFMotor.setTargetPosition(newRFTarget);
        LFMotor.setTargetPosition(newLFTarget);
        LBMotor.setTargetPosition(newLBTarget);
        RBMotor.setTargetPosition(newRBTarget);

        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power;

        while (opModeIsActive() && (RFMotor.isBusy() && LFMotor.isBusy() && LBMotor.isBusy() && RBMotor.isBusy())) {

            int distanceRemaining = Math.min(
                    Math.abs(newRFTarget - RFMotor.getCurrentPosition()),
                    Math.min(Math.abs(newLFTarget - LFMotor.getCurrentPosition()),
                            Math.min(Math.abs(newLBTarget - LBMotor.getCurrentPosition()),
                                    Math.abs(newRBTarget - RBMotor.getCurrentPosition())))
            );

            if (distanceRemaining > (slowDownStart * Math.abs(inches * COUNTS_PER_INCH))) {
                power = maxSpeed;
            } else {
                power = maxSpeed * (distanceRemaining / (double) (Math.abs(inches * COUNTS_PER_INCH)));
            }

            power = Math.max(power, minPower);

            RFMotor.setPower(-power);
            LFMotor.setPower(power);
            LBMotor.setPower(-power);
            RBMotor.setPower(power);


            telemetry.addData("Target", "Running to %7d :%7d :%7d :%7d", newRFTarget, newLFTarget, newLBTarget, newRBTarget);
            telemetry.addData("Current", "Running at %7d :%7d :%7d :%7d", RFMotor.getCurrentPosition(), LFMotor.getCurrentPosition(), LBMotor.getCurrentPosition(), RBMotor.getCurrentPosition());
            telemetry.addData("Power", "Power: %5.2f", power);
            telemetry.update();
        }

        RFMotor.setPower(0);
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);
    }

    public void moveFrontRightDiagonal(double inches, double maxSpeed) {

        RFMotor.setPower(0);
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);

        //int newRFTarget = RFMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        int newLFTarget = LFMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        //int newLBTarget = LBMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        int newRBTarget = RBMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

        //RFMotor.setTargetPosition(newRFTarget);
        LFMotor.setTargetPosition(newLFTarget);
        //LBMotor.setTargetPosition(newLBTarget);
        RBMotor.setTargetPosition(newRBTarget);

        //RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power;

        while (opModeIsActive() && LFMotor.isBusy() && RBMotor.isBusy()) {

            int distanceRemaining = Math.min(
                    Math.abs(newLFTarget - LFMotor.getCurrentPosition()),
                            Math.abs(newRBTarget - RBMotor.getCurrentPosition())
            );
            if (distanceRemaining > (slowDownStart * Math.abs(inches * COUNTS_PER_INCH))) {
                power = maxSpeed;
            } else {
                power = maxSpeed * (distanceRemaining / (double) (Math.abs(inches * COUNTS_PER_INCH)));
            }

            power = Math.max(power, minPower);

            RFMotor.setPower(0);
            LFMotor.setPower(power);
            LBMotor.setPower(0);
            RBMotor.setPower(power);


            telemetry.addData("Target", "Running to %7d :%7d", newLFTarget, newRBTarget);
            telemetry.addData("Current", "Running at %7d :%7d :%7d :%7d", RFMotor.getCurrentPosition(), LFMotor.getCurrentPosition(), LBMotor.getCurrentPosition(), RBMotor.getCurrentPosition());
            telemetry.addData("Power", "Power: %5.2f", power);
            telemetry.update();
        }

        RFMotor.setPower(0);
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);
    }

    public void moveFrontLeftDiagonal(double inches, double maxSpeed) {

        RFMotor.setPower(0);
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);

        int newRFTarget = RFMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        //int newLFTarget = LFMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newLBTarget = LBMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        //int newRBTarget = RBMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

        RFMotor.setTargetPosition(newRFTarget);
        //LFMotor.setTargetPosition(newLFTarget);
        LBMotor.setTargetPosition(newLBTarget);
        //RBMotor.setTargetPosition(newRBTarget);

        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power;

        while (opModeIsActive() && (RFMotor.isBusy() && LBMotor.isBusy())) {

            int distanceRemaining = Math.min(
                    Math.abs(newRFTarget - RFMotor.getCurrentPosition()),
                    Math.abs(newLBTarget - LBMotor.getCurrentPosition())
                    );

            if (distanceRemaining > (slowDownStart * Math.abs(inches * COUNTS_PER_INCH))) {
                power = maxSpeed;
            } else {
                power = maxSpeed * (distanceRemaining / (double) (Math.abs(inches * COUNTS_PER_INCH)));
            }

            power = Math.max(power, minPower);

            RFMotor.setPower(power);
            LFMotor.setPower(0);
            LBMotor.setPower(power);
            RBMotor.setPower(0);


            telemetry.addData("Target", "Running to %7d :%7d", newRFTarget, newLBTarget);
            telemetry.addData("Current", "Running at %7d :%7d :%7d :%7d", RFMotor.getCurrentPosition(), LFMotor.getCurrentPosition(), LBMotor.getCurrentPosition(), RBMotor.getCurrentPosition());
            telemetry.addData("Power", "Power: %5.2f", power);
            telemetry.update();
        }

        RFMotor.setPower(0);
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);
    }

    public void moveBackRightDiagonal(double inches, double maxSpeed) {

        RFMotor.setPower(0);
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);

        int newRFTarget = RFMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        //int newLFTarget = LFMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newLBTarget = LBMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        //int newRBTarget = RBMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

        RFMotor.setTargetPosition(newRFTarget);
        //LFMotor.setTargetPosition(newLFTarget);
        LBMotor.setTargetPosition(newLBTarget);
        //RBMotor.setTargetPosition(newRBTarget);

        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power;

        while (opModeIsActive() && (RFMotor.isBusy() && LBMotor.isBusy())) {

            int distanceRemaining = Math.min(
                    Math.abs(newRFTarget - RFMotor.getCurrentPosition()),
                    Math.abs(newLBTarget - LBMotor.getCurrentPosition())
            );

            if (distanceRemaining > (slowDownStart * Math.abs(inches * COUNTS_PER_INCH))) {
                power = maxSpeed;
            } else {
                power = maxSpeed * (distanceRemaining / (double) (Math.abs(inches * COUNTS_PER_INCH)));
            }

            power = Math.max(power, minPower);

            RFMotor.setPower(-power);
            LFMotor.setPower(0);
            LBMotor.setPower(-power);
            RBMotor.setPower(0);


            telemetry.addData("Target", "Running to %7d :%7d", newRFTarget, newLBTarget);
            telemetry.addData("Current", "Running at %7d :%7d :%7d :%7d", RFMotor.getCurrentPosition(), LFMotor.getCurrentPosition(), LBMotor.getCurrentPosition(), RBMotor.getCurrentPosition());
            telemetry.addData("Power", "Power: %5.2f", power);
            telemetry.update();
        }

        RFMotor.setPower(0);
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);
    }

    public void moveBackLeftDiagonal(double inches, double maxSpeed) {

        RFMotor.setPower(0);
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);

        //int newRFTarget = RFMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        int newLFTarget = LFMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        //int newLBTarget = LBMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        int newRBTarget = RBMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

        //RFMotor.setTargetPosition(newRFTarget);
        LFMotor.setTargetPosition(newLFTarget);
        //LBMotor.setTargetPosition(newLBTarget);
        RBMotor.setTargetPosition(newRBTarget);

        //RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power;

        while (opModeIsActive() && LFMotor.isBusy() && RBMotor.isBusy()) {

            int distanceRemaining = Math.min(
                    Math.abs(newLFTarget - LFMotor.getCurrentPosition()),
                    Math.abs(newRBTarget - RBMotor.getCurrentPosition())
            );
            if (distanceRemaining > (slowDownStart * Math.abs(inches * COUNTS_PER_INCH))) {
                power = maxSpeed;
            } else {
                power = maxSpeed * (distanceRemaining / (double) (Math.abs(inches * COUNTS_PER_INCH)));
            }

            power = Math.max(power, minPower);

            RFMotor.setPower(0);
            LFMotor.setPower(power);
            LBMotor.setPower(0);
            RBMotor.setPower(power);


            telemetry.addData("Target", "Running to %7d :%7d", newLFTarget, newRBTarget);
            telemetry.addData("Current", "Running at %7d :%7d :%7d :%7d", RFMotor.getCurrentPosition(), LFMotor.getCurrentPosition(), LBMotor.getCurrentPosition(), RBMotor.getCurrentPosition());
            telemetry.addData("Power", "Power: %5.2f", power);
            telemetry.update();
        }

        RFMotor.setPower(0);
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);
    }

    public void spinRight(double inches, double maxSpeed) {

        RFMotor.setPower(0);
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);

        int newRFTarget = RFMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        int newLFTarget = LFMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newLBTarget = LBMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newRBTarget = RBMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);

        RFMotor.setTargetPosition(newRFTarget);
        LFMotor.setTargetPosition(newLFTarget);
        LBMotor.setTargetPosition(newLBTarget);
        RBMotor.setTargetPosition(newRBTarget);

        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power;

        while (opModeIsActive() && (RFMotor.isBusy() && LFMotor.isBusy() && LBMotor.isBusy() && RBMotor.isBusy())) {

            int distanceRemaining = Math.min(
                    Math.abs(newRFTarget - RFMotor.getCurrentPosition()),
                    Math.min(Math.abs(newLFTarget - LFMotor.getCurrentPosition()),
                            Math.min(Math.abs(newLBTarget - LBMotor.getCurrentPosition()),
                                    Math.abs(newRBTarget - RBMotor.getCurrentPosition())))
            );

            if (distanceRemaining > (slowDownStart * Math.abs(inches * COUNTS_PER_INCH))) {
                power = maxSpeed;
            } else {
                power = maxSpeed * (distanceRemaining / (double) (Math.abs(inches * COUNTS_PER_INCH)));
            }

            power = Math.max(power, minPower);

            // 21.5 INCHES MOVES ROBOT 90 DEGREES TO THE RIGHT

            RFMotor.setPower(power);
            LFMotor.setPower(power);
            LBMotor.setPower(power);
            RBMotor.setPower(power);


            telemetry.addData("Target", "Running to %7d :%7d :%7d :%7d", newRFTarget, newLFTarget, newLBTarget, newRBTarget);
            telemetry.addData("Current", "Running at %7d :%7d :%7d :%7d", RFMotor.getCurrentPosition(), LFMotor.getCurrentPosition(), LBMotor.getCurrentPosition(), RBMotor.getCurrentPosition());
            telemetry.addData("Power", "Power: %5.2f", power);
            telemetry.update();
        }

        RFMotor.setPower(0);
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);
    }

    public void spinLeft(double inches, double maxSpeed) {

        RFMotor.setPower(0);
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);

        int newRFTarget = RFMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        int newLFTarget = LFMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newLBTarget = LBMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int newRBTarget = RBMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);

        RFMotor.setTargetPosition(newRFTarget);
        LFMotor.setTargetPosition(newLFTarget);
        LBMotor.setTargetPosition(newLBTarget);
        RBMotor.setTargetPosition(newRBTarget);

        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power;

        while (opModeIsActive() && (RFMotor.isBusy() && LFMotor.isBusy() && LBMotor.isBusy() && RBMotor.isBusy())) {

            int distanceRemaining = Math.min(
                    Math.abs(newRFTarget - RFMotor.getCurrentPosition()),
                    Math.min(Math.abs(newLFTarget - LFMotor.getCurrentPosition()),
                            Math.min(Math.abs(newLBTarget - LBMotor.getCurrentPosition()),
                                    Math.abs(newRBTarget - RBMotor.getCurrentPosition())))
            );

            if (distanceRemaining > (slowDownStart * Math.abs(inches * COUNTS_PER_INCH))) {
                power = maxSpeed;
            } else {
                power = maxSpeed * (distanceRemaining / (double) (Math.abs(inches * COUNTS_PER_INCH)));
            }

            power = Math.max(power, minPower);

            // NEGATIVE 21.5 INCHES MOVES ROBOT 90 DEGREES TO THE LEFT

            RFMotor.setPower(power);
            LFMotor.setPower(power);
            LBMotor.setPower(power);
            RBMotor.setPower(power);


            telemetry.addData("Target", "Running to %7d :%7d :%7d :%7d", newRFTarget, newLFTarget, newLBTarget, newRBTarget);
            telemetry.addData("Current", "Running at %7d :%7d :%7d :%7d", RFMotor.getCurrentPosition(), LFMotor.getCurrentPosition(), LBMotor.getCurrentPosition(), RBMotor.getCurrentPosition());
            telemetry.addData("Power", "Power: %5.2f", power);
            telemetry.update();
        }

        RFMotor.setPower(0);
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);
    }

}
