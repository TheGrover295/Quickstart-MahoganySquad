package org.firstinspires.ftc.teamcode.pedroPathing.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem for Intake, Chamber Spinner, and Artifact Transfer (ATM).
 */
public class Intake {

    private DcMotor intakeMotor;
    private DcMotor chamberSpinner;
    private CRServo artifactTransfer;

    private double chamberTargetPos = 0;
    private final double TICKS_PER_STEP = 475.06;

    public void init(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        artifactTransfer = hwMap.get(CRServo.class, "ATM");

        chamberSpinner = hwMap.get(DcMotor.class, "chamberSpinner");
        chamberSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        chamberSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chamberSpinner.setTargetPosition(0);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.5);
    }

    // --- Intake Methods ---
    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    // --- ATM (CRServo) Methods ---
    public void setTransferPower(double power) {
        artifactTransfer.setPower(power);
    }

    // --- Chamber Methods ---
    public void moveChamberStep() {
        chamberTargetPos += TICKS_PER_STEP;
        updateChamber(0.72);
    }

    public void adjustChamber(double delta) {
        chamberTargetPos += delta;
        updateChamber(0.5);
    }

    private void updateChamber(double power) {
        chamberSpinner.setTargetPosition((int) chamberTargetPos);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(power);
    }

    public int getChamberPosition() {
        return chamberSpinner.getCurrentPosition();
    }

    public void stopAll() {
        intakeMotor.setPower(0);
        artifactTransfer.setPower(0);
    }
}