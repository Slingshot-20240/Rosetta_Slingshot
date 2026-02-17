package org.firstinspires.ftc.teamcode.teleop.fsm;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Park;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;


public class FSM {
    // --------------- Robot & States ---------------
    public Robot robot;
    public FSMStates state = FSMStates.BASE_STATE;
    public ControlType type = ControlType.PID_CONTROL;
    private final GamepadMapping gamepad;

    // --------------- SUBSYSTEMS ---------------
    private final Intake intake;
    private final Stopper stopper;
    private final Shooter shooter;
    private final Park park;

    // --------------- MISC ---------------
    public double lastVelo = 800;
    private ControlType savedType;

    public FSM(HardwareMap hardwareMap, GamepadMapping gamepad, Robot robot) {
        this.robot = robot;
        this.gamepad = robot.controls;

        intake = robot.intake;
        stopper = robot.stopper;
        park = robot.park;
        shooter = robot.shooter;

        savedType = ControlType.PID_CONTROL;
    }

    public void update() {
        gamepad.update();

        switch (state) {
            case BASE_STATE:

                // Park toggle
                if (gamepad.park.value()) {
                    state = FSMStates.PARK;
                }

                // Stopper toggle
                if (gamepad.transfer.value() && type == ControlType.PID_CONTROL) {
                    intake.intakeTransferOn();
                    stopper.close();
                } else {
                    stopper.open();
                }

                // Outtake hold
                if (gamepad.outtake.locked()) {
                    state = FSMStates.OUTTAKING;
                }

                // Intake hold
                if (gamepad.intake.locked()) {
                    intake.intakeTransferOn();
                } else {
                    intake.intakeTransferOff();
                }

                // --------------- PID Only ---------------

                if (type == ControlType.PID_CONTROL) {

                    // variables
                    double distance = Robot.cam.getTargetArtifactTravelDistanceX();
                    double targetVelocity = robot.shooter.calculateShooterRPM(distance) + 100;
                    double targetHoodPos;

                    // calculate target
                    if (Robot.cam.getATdist() < 54) {
                        targetHoodPos = robot.shooter.calculateHoodPos(distance) + 0.2;
                    } else {
                        targetHoodPos = robot.shooter.calculateHoodPos(distance) + 0.1;
                    }

                    if (Robot.cam.getATdist() != 0) {
                        lastVelo = targetVelocity;
                    }

                    // This should prevent the shooter from changing hood pos if it can't see the AprilTag (so if it cuts out it's fine)
                    if (Robot.cam.getTargetArtifactTravelDistanceX() == 22) {
                        robot.shooter.setHoodAngle(shooter.variableHood.getPosition());
                        robot.shooter.setShooterVelocity(-lastVelo);
                    } else {
                        robot.shooter.setHoodAngle(targetHoodPos);
                        robot.shooter.setShooterVelocity(-targetVelocity);
                    }

                    // set LED states
                    if (Robot.cam.getTargetArtifactTravelDistanceX() != 22) {
                        robot.ledBoard0.setState(true);
                        robot.ledBoard1.setState(true);
                    } else {
                        robot.ledBoard0.setState(false);
                        robot.ledBoard1.setState(true);
                    }
                }

                // --------------- Hardcoded Only ---------------
                if (gamepad.switchMode.value()) {
                    // if saved is PID (in PID mode), switch to Hardcoded
                    if (savedType == ControlType.PID_CONTROL) {
                        type = ControlType.HARDCODED_CONTROL;
                        savedType = ControlType.HARDCODED_CONTROL;

                    // if saved is Hardcoded (in Hardcoded mode), switch to PID
                    } else if (savedType == ControlType.HARDCODED_CONTROL) {
                        type = ControlType.PID_CONTROL;
                        savedType = ControlType.PID_CONTROL;
                    }
                }

                if (type == ControlType.HARDCODED_CONTROL) {
                    shooter.shootFromFront();
                    shooter.hoodToFront();
                }

                if (gamepad.shootBack.locked() && type == ControlType.HARDCODED_CONTROL) {
                    state = FSMStates.SHOOT_BACK;
                }

                if (gamepad.shootFront.locked() && type == ControlType.HARDCODED_CONTROL) {
                    state = FSMStates.SHOOT_FRONT;
                }

                break;

            case OUTTAKING:
                intake.intakeTransferReverse();

                if (!gamepad.outtake.locked()) {
                    state = FSMStates.BASE_STATE;
                    gamepad.resetMultipleControls(gamepad.transfer);
                }
                break;

            case PID_SHOOT:

                intake.intakeTransferOn();
                stopper.close();

                if (!gamepad.transfer.locked()) {
                    state = FSMStates.BASE_STATE;
                    gamepad.resetMultipleControls(gamepad.transfer, gamepad.outtake);
                }

                break;

            case PARK:
                park.tilt();
                if (!gamepad.park.value()) {
                    state = FSMStates.BASE_STATE;
                    park.unTilt();
                }

                break;

            // --------------- Hardcoded Only ---------------

            case SHOOT_BACK:

                shooter.shootFromBack();
                shooter.hoodToBack();
                intake.intakeTransferOn();

                stopper.close();

                if (!gamepad.shootBack.locked()) {
                    state = FSMStates.BASE_STATE;
                    gamepad.resetMultipleControls(gamepad.transfer);
                }

                break;

            case SHOOT_FRONT:

                intake.intakeTransferOn();
                shooter.shootFromFront();
                shooter.hoodToFront();

                stopper.close();

                if (!gamepad.shootFront.locked()) {
                    state = FSMStates.BASE_STATE;
                    gamepad.resetMultipleControls(gamepad.shootBack, gamepad.shootFront, gamepad.transfer);
                }

                break;
        }
    }

    public void setState(FSMStates newState) {
        state = newState;
    }

    public FSMStates getState() {
        return state;
    }

    public void setControlType(ControlType newCType) {
        type = newCType;
    }

    public ControlType getControlType() {
        return type;
    }

    public enum FSMStates {
        BASE_STATE,
        SHOOT_FRONT,
        SHOOT_BACK,
        OUTTAKING,
        PID_SHOOT,
        PARK
    }

    public enum ControlType {
        HARDCODED_CONTROL,
        PID_CONTROL
    }

    }

