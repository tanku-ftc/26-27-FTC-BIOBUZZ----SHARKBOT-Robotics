package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class TurretMechanism {

    public DcMotorEx rotateMotor;

    private double  kP = 0.0001;
    private double kD = 0.0000;

    private final double goalX = 0;
    private  double lastError = 0;
    private final double angleTolerance = 0.2;

    private final double MAX_POWER = 0.5;

    private double power = 0.0;

    private final ElapsedTime timer = new ElapsedTime();


    public int LEFT_LIMIT = -1700;
    public int RIGHT_LIMIT = 1700;

    public void init(HardwareMap hwMap) {

        rotateMotor = hwMap.get(DcMotorEx.class, "rotateMotor");

        rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        timer.reset();
    }

    public void setkP(double newkP) {
        kP = newkP;
    }

    public double getkP() {
        return kP;
    }

    public void setkD(double newkD) {
        kD = newkD;
    }

    public double getkD() {
        return kD;
    }

    public void resetTimer() {
        timer.reset();
    }

    public int getEncoderPosition() {
        return rotateMotor.getCurrentPosition();
    }

    public void update(LLResult llresult) {

        double deltaTime = timer.seconds();
        timer.reset();

        if (llresult == null || !llresult.isValid()) {
            rotateMotor.setPower(0);
            lastError = 0;
            return;
        }

        double error = goalX - llresult.getTx();

        double pTerm = error * kP;

        double dTerm = 0;

        if (deltaTime > 0) {
            dTerm = ((error - lastError) / deltaTime) * kD;
        }

        if (Math.abs(error) < angleTolerance) {
            power = 0;
        } else {
            power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }

        // Encoder limits

        int position = rotateMotor.getCurrentPosition();

        if (power > 0 && position >= RIGHT_LIMIT) {
            power = 0;
        }

        if (power < 0 && position <= LEFT_LIMIT) {
            power = 0;
        }

        rotateMotor.setPower(power);

        lastError = error;
    }
}