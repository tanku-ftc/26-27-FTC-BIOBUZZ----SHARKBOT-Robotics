package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.Mechanisms.TurretMechanism;

@TeleOp(name = "Mecanum + Turret Auto Align", group = "TeleOp")
public class TurretAutoAlignOpmode extends OpMode {

    // ================= DRIVE =================
    private DcMotor frontLeft, backLeft, frontRight, backRight, intakeMotor, rampMotor;
    private DcMotorEx outakeMotor;


    // ================= TURRET =================
    private final TurretMechanism turret = new TurretMechanism();
    private Limelight3A limelight3A;
    private DigitalChannel mgswitch;

    private Follower follower;

    // ================= PID =================
    private final double[] stepSizes = {
            0.1, 0.01, 0.001, 0.0001, 0.00001
    };

    private int stepIndex = 2;

    public boolean isMagSwitchActive() {
        return !mgswitch.getState();
    }

    @Override
    public void init() {

        // Drive Motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        intakeMotor = hardwareMap.get(DcMotor.class, "par");
        rampMotor = hardwareMap.get(DcMotor.class, "perp");
        outakeMotor = hardwareMap.get(DcMotorEx.class, "motorOutake");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rampMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Limelight
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(8);

        // Magnetic Limit Switch
        mgswitch = hardwareMap.get(DigitalChannel.class, "magneticLimitSwitch");
        mgswitch.setMode(DigitalChannel.Mode.INPUT);

        // Turret
        turret.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void start() {
        turret.resetTimer();
        limelight3A.start();
    }

    @Override
    public void loop() {

        follower.update();

        // ================= INTAKE/RAMP/OUTAKE ===================

         double intakePower = gamepad1.left_trigger * -0.9;
         double rampPower = gamepad1.right_trigger * -0.5;
         Pose pose = follower.getPose();
         double robotX = pose.getX();
         double robotY = pose.getY();
         double distanceToGoal = Math.sqrt(Math.pow(robotX + 49, 2) + Math.pow(robotY + 49.5, 2));
         double outakeVelocitySlope = 5.5;
         double outakeVelocityStart = 200;
         double outakeVelocity = ((outakeVelocitySlope * distanceToGoal) + outakeVelocityStart);


        if(gamepad1.left_trigger > 0.01) {
            intakeMotor.setPower(intakePower);
        } else {
            intakeMotor.setPower(0);
        }

        if (gamepad1.right_trigger > 0.01) {
            rampMotor.setPower(rampPower);
        } else {
            rampMotor.setPower(0);
        }

       if(gamepad1.right_bumper) {
           outakeMotor.setVelocity(outakeVelocity);
       } else {
           outakeMotor.setVelocity(0);
       }


        // ================= MECANUM DRIVE =================

        double x = -gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double translationPower = Math.hypot(x, y);
        double translationAngle = Math.atan2(y, x);

        double ADPower = translationPower * Math.sqrt(2) * 0.5 *
                (Math.sin(translationAngle) + Math.cos(translationAngle));

        double BCPower = translationPower * Math.sqrt(2) * 0.5 *
                (Math.sin(translationAngle) - Math.cos(translationAngle));

        double turningScale = Math.max(
                Math.abs(ADPower + turn),
                Math.abs(ADPower - turn)
        );

        turningScale = Math.max(turningScale,
                Math.max(
                        Math.abs(BCPower + turn),
                        Math.abs(BCPower - turn)
                ));

        if (turningScale < 1.0) turningScale = 1.0;

        double frontLeftPower = (ADPower - turn) / turningScale;
        double backLeftPower = (BCPower - turn) / turningScale;
        double frontRightPower = (BCPower + turn) / turningScale;
        double backRightPower = (ADPower + turn) / turningScale;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        // ================= LIMELIGHT =================

        LLResult llresult = limelight3A.getLatestResult();
        turret.update(llresult);

        // ================= PID TUNING =================

        if (gamepad1.bWasPressed())
            stepIndex = (stepIndex + 1) % stepSizes.length;

        if (gamepad1.dpadLeftWasPressed())
            turret.setkP(turret.getkP() - stepSizes[stepIndex]);

        if (gamepad1.dpadRightWasPressed())
            turret.setkP(turret.getkP() + stepSizes[stepIndex]);

        if (gamepad1.dpadUpWasPressed())
            turret.setkD(turret.getkD() + stepSizes[stepIndex]);

        if (gamepad1.dpadDownWasPressed())
            turret.setkD(turret.getkD() - stepSizes[stepIndex]);

        // ================= LIMIT ADJUSTMENT =================

        if (gamepad1.aWasPressed())
            turret.RIGHT_LIMIT += 10;

        if (gamepad1.yWasPressed())
            turret.LEFT_LIMIT -= 10;

        // ================= MAGNETIC SWITCH =================


        // ================= TELEMETRY =================

        if (llresult != null && llresult.isValid()) {
            telemetry.addData("tx", llresult.getTx());
            telemetry.addData("ty", llresult.getTy());
            telemetry.addData("ta", llresult.getTa());
            telemetry.addLine("Target detected");
        } else {
            telemetry.addLine("No tag detected");
        }

        telemetry.addLine("---------------------------------------");

        telemetry.addData("FL", "%.2f", frontLeftPower);
        telemetry.addData("BL", "%.2f", backLeftPower);
        telemetry.addData("FR", "%.2f", frontRightPower);
        telemetry.addData("BR", "%.2f", backRightPower);

        telemetry.addLine("---------------------------------------");

        telemetry.addData("Tuning P", "%.5f", turret.getkP());
        telemetry.addData("Tuning D", "%.5f", turret.getkD());
        telemetry.addData("Step Size", "%.5f", stepSizes[stepIndex]);
        telemetry.addData("Step", stepIndex);

        telemetry.addData("Turret Encoder", turret.rotateMotor.getCurrentPosition());
        telemetry.addData("Left Limit", turret.LEFT_LIMIT);
        telemetry.addData("Right Limit", turret.RIGHT_LIMIT);
        telemetry.addLine("---------------------------------------");
        telemetry.addData("Robot X", robotX);
        telemetry.addData("Robot Y", robotY);
        telemetry.addData("Distance", distanceToGoal);


        telemetry.update();
    }
}