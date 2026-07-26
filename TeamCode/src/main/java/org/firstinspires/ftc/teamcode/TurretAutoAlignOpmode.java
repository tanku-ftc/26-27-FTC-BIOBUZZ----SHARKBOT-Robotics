package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Mechanisms.TurretMechanism;

@TeleOp
public class TurretAutoAlignOpmode extends OpMode {

    private Limelight3A limelight3A;
    private TurretMechanism turret = new TurretMechanism();

    private DigitalChannel mgswitch;

    public boolean isMagSwitchActive() {
        return mgswitch.getState();
    }




    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
      // Index to select the current step size from the array.
    int stepIndex = 2;

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        mgswitch = hardwareMap.get(DigitalChannel.class, "magneticLimitSwitch");
        mgswitch.setMode(DigitalChannel.Mode.INPUT);
        limelight3A.pipelineSwitch(8);
        turret.init(hardwareMap);

    }
     @Override
    public void start() {
        turret.resetTimer();
       limelight3A.start();
    }



    @Override
    public void loop() {

        LLResult llresult = limelight3A.getLatestResult();

        turret.update(llresult);

          //update P and D on the fly.
          //Press B to cycle between step sizes.
        if(gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1 ) % stepSizes.length; // wraps back to zero.
        }
          // Dpad left right - P gain.
        if (gamepad1.dpadLeftWasPressed()) {
            turret.setkP(turret.getkP() - stepSizes[stepIndex]);
        }
        if (gamepad1.dpadRightWasPressed()) {
            turret.setkP(turret.getkP() + stepSizes[stepIndex]);
        }
          //Dpad up down - D Gain
        if (gamepad1.dpadUpWasPressed()) {
            turret.setkD(turret.getkD() + stepSizes[stepIndex]);
        }
        if (gamepad1.dpadDownWasPressed()) {
            turret.setkD(turret.getkD() - stepSizes[stepIndex]);
        }

        if (gamepad1.aWasPressed()) {
            turret.RIGHT_LIMIT+=10;
        }
        if (gamepad1.yWasPressed()) {
            turret.LEFT_LIMIT -=10;
        }

        if (isMagSwitchActive()) {
            turret.rotateMotor.setMode(
                    DcMotorEx.RunMode.RESET_ENCODERS);
        }



        if (llresult != null && llresult.isValid()) {
            telemetry.addData("tx",llresult.getTx());
            telemetry.addData("ty",llresult.getTy());
            telemetry.addData("ta",llresult.getTa());
            telemetry.addLine("Target detected");
        } else {
            telemetry.addLine("No tag detected");
        }
        telemetry.addLine("---------------------------------------");
        telemetry.addData("Tuning P", "%.5f (D-Pad L/R)", turret.getkP());
        telemetry.addData("Tuning D","%.5f (D-Pad U/D)", turret.getkD());
        telemetry.addData("Step Sizes","%.5f (B Button)", stepSizes[stepIndex]);
        telemetry.addData("Step", stepIndex);
        telemetry.addLine("--------------------------------------------------");
        telemetry.addData("Turret Encoder", turret.rotateMotor.getCurrentPosition());
        telemetry.addData("Left Limit", turret.LEFT_LIMIT);
        telemetry.addData("Right Limit", turret.RIGHT_LIMIT);

        telemetry.update();
    }
}



