package org.firstinspires.ftc.teamcode.commandbase;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Intake {

    private final Robot robot = Robot.getInstance();

    public void init() {

    }

    public enum IntakePivotState {
        TRANSFER,
        HOVER,
        PICKUP,
        SIDE_DROP,
        OUTTAKE_AVOID
    }

    public enum IntakeTurretState {
        STRAIGHT,
        RIGHT_INTAKE,
        LEFT_INTAKE,
        RIGHT_SIDE_DROP,
        LEFT_SIDE_DROP
    }

    public enum IntakeWristState {
        STRAIGHT,
        RIGHT_90,
        LEFT_90
    }

    public enum IntakeClawState {
        OPEN,
        CLOSED
    }

    public static IntakePivotState intakePivotState = IntakePivotState.TRANSFER;
    public static IntakeTurretState intakeTurretState = IntakeTurretState.STRAIGHT;
    public static IntakeWristState intakeWristState = IntakeWristState.STRAIGHT;
    public static IntakeClawState intakeClawState = IntakeClawState.OPEN;
    private static final PIDFController extendoPIDF = new PIDFController(0,0,0, 0);

    public void setPivot(IntakePivotState intakePivotState) {
        switch(intakePivotState) {
            case TRANSFER:
                robot.intakePivot.setPosition(INTAKE_PIVOT_TRANSFER);
                break;
            case HOVER:
                robot.intakePivot.setPosition(INTAKE_PIVOT_PICKUP_READY);
                break;
            case PICKUP:
                robot.intakePivot.setPosition(INTAKE_PIVOT_PICKUP);
                break;
            case SIDE_DROP:
                robot.intakePivot.setPosition(INTAKE_PIVOT_DROP);
                break;
            case OUTTAKE_AVOID:
                robot.intakePivot.setPosition(INTAKE_PIVOT_AVOID);
                break;
        }
    }

    public void setTurret(IntakeTurretState intakeTurretState) {
        switch(intakeTurretState) {
            case STRAIGHT:
                robot.intakeTurret.setPosition(INTAKE_TURRET_TRANSFER);
                break;
            case RIGHT_INTAKE:
                robot.intakeTurret.setPosition(INTAKE_TURRET_PICKUP_RIGHT);
                break;
            case LEFT_INTAKE:
                robot.intakeTurret.setPosition(INTAKE_TURRET_PICKUP_LEFT);
                break;
            case RIGHT_SIDE_DROP:
                robot.intakeTurret.setPosition(INTAKE_TURRET_DROP_RIGHT);
                break;
            case LEFT_SIDE_DROP:
                robot.intakeTurret.setPosition(INTAKE_TURRET_DROP_LEFT);
                break;
        }
    }

    public void setWrist(IntakeWristState intakeWristState) {
        switch(intakeWristState) {
            case STRAIGHT:
                robot.intakeWrist.setPosition(INTAKE_WRIST_STRAIGHT);
                break;
            case RIGHT_90:
                robot.intakeWrist.setPosition(INTAKE_WRIST_RIGHT90);
                break;
            case LEFT_90:
                robot.intakeWrist.setPosition(INTAKE_WRIST_LEFT90);
                break;
        }
    }

    public void setClaw(IntakeClawState intakeClawState) {
        switch(intakeClawState) {
            case OPEN:
                robot.intakeClaw.setPosition(INTAKE_CLAW_OPEN);
                break;
            case CLOSED:
                robot.intakeClaw.setPosition(INTAKE_CLAW_CLOSE);
                break;
        }
    }

}
