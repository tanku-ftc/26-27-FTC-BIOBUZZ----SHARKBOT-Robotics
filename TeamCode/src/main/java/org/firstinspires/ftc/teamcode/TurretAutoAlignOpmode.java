package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.teamcode.Mechanisms.TurretMechanism;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Mecanum + Turret Auto Align", group = "TeleOp")
public class TurretAutoAlignOpmode extends OpMode {

    // ================= DRIVE =================
    private DcMotor frontLeft, backLeft, frontRight, backRight, intakeMotor, rampMotor;
    private DcMotorEx outakeMotor;


    // ================= TURRET =================
    private final TurretMechanism turret = new TurretMechanism();
    private Limelight3A limelight3A;
    private DigitalChannel mgswitch;

    private double CAMERA_HEIGHT_CM = 36.195;

    private double CAMERA_ANGLE = 8.5;

    private double GOAL_HEIGHT = 74.95;

    private double distanceToGoal = 0;

    private long lastTagTime = 0;
    private static final long TAG_TIMEOUT_MS = 20000; // 20 seconds


    // ================= PD =================
    private final double[] stepSizes = {
            0.1, 0.01, 0.001, 0.0001, 0.00001
    };

    private int stepIndex = 2;

    private double outakeVelocityTest;

    private double outakeAdjust = 0;

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

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        outakeMotor.setDirection(DcMotor.Direction.REVERSE);

        outakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(67, 0, 0, 25);
        outakeMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        // Limelight
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(8);

        // Magnetic Limit Switch
        mgswitch = hardwareMap.get(DigitalChannel.class, "magneticLimitSwitch");
        mgswitch.setMode(DigitalChannel.Mode.INPUT);

        // Turret
        turret.init(hardwareMap);

        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void start() {
        turret.resetTimer();
        limelight3A.start();
    }

    public double getDistance(double ty) {
        double angleToTarget = CAMERA_ANGLE + ty;
        double heightDifference = GOAL_HEIGHT - CAMERA_HEIGHT_CM;

        return heightDifference / Math.tan(Math.toRadians(angleToTarget));
    }

    @Override
    public void loop() {

        LLResult llresult = limelight3A.getLatestResult();

        // ================= INTAKE/RAMP/TURRET STOP ===================


         double intakePower = gamepad1.left_trigger * -0.9;
         double rampPower = gamepad1.right_trigger * -0.5;

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

        if (gamepad1.left_bumper) {
            turret.rotateMotor.setPower(0);
        } else {
            turret.update(llresult);
        }



        // ================= MECANUM DRIVE =================

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;

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

        // ================= LIMELIGHT/OUTAKE =================

        double outakeVelocity;

        if (llresult != null && llresult.isValid()) {

            // Tag detected, reset timer
            lastTagTime = System.currentTimeMillis();

            // Calculate distance
            distanceToGoal = getDistance(llresult.getTy());

            // Calculate speed from distance
            outakeVelocity = 0.099391 * distanceToGoal + 135.10989;

        } else {

            // How long since we last saw the tag?
            long timeSinceTag = System.currentTimeMillis() - lastTagTime;

            if (timeSinceTag < TAG_TIMEOUT_MS && lastTagTime != 0) {

                // Keep using the last calculated distance/speed
                outakeVelocity = 0.099391 * distanceToGoal + 135.10989;

            } else {

                // After 20 seconds with no tag, return to Y-intercept of 20
                outakeVelocity = 135.10989;
            }
        }

        outakeMotor.setVelocity(outakeVelocity);

        if (gamepad1.aWasPressed()) {
            outakeAdjust += 5;
        }

        if (gamepad1.yWasPressed()) {
            outakeAdjust -= 5;
        }

        outakeVelocityTest = 0 + outakeAdjust;




        telemetry.addData("Distance to April Tag", "%.2f", distanceToGoal);
        telemetry.addData("Outtake Velocity", "%.2f", outakeVelocity);
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



    }

}