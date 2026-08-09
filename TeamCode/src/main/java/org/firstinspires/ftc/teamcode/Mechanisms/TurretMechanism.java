package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class TurretMechanism {

    public DcMotorEx rotateMotor;

    private double  kP = 0.03031;
    private double kD = 0.01001;

    private final double goalX = 0;
    private  double lastError = 0;
    private final double angleTolerance = 3;

    private final double MAX_POWER = 0.8;

    private double power = 0.0;

    private final ElapsedTime timer = new ElapsedTime();


    public int LEFT_LIMIT = -1700;
    public int RIGHT_LIMIT = 1700;

    private double filteredTx = 0;
    private static final double TX_FILTER = 0.5;

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
            filteredTx = 0;
            lastError = 0;
            return;
        }

        filteredTx = TX_FILTER * llresult.getTx() + (1.0 - TX_FILTER) * filteredTx;

        double error = goalX - filteredTx;

        double pTerm = error * kP;

        double dTerm = 0;

        if (deltaTime > 0.01) {
            dTerm = ((error - lastError) / deltaTime) * kD;
        }

        if (Math.abs(error) < angleTolerance) {
            power = 0;
        } else {
            double ff = 0.03 * Math.signum(error);

            power = Range.clip(pTerm + dTerm + ff, -MAX_POWER, MAX_POWER);

            // Minimum power to overcome friction
            if (Math.abs(power) > 0) {
                power = Math.copySign(Math.max(Math.abs(power), 0.08), power);
            }
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