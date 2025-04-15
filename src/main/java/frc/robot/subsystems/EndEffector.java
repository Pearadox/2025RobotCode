// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.RobotContainer;
import frc.robot.util.SmarterDashboard;

public class EndEffector extends SubsystemBase {

    private static final EndEffector END_EFFECTOR = new EndEffector();

    private static final LEDStrip leds = LEDStrip.getInstance();

    public static EndEffector getInstance() {
        return END_EFFECTOR;
    }

    private PearadoxTalonFX endEffector;
    private DigitalInput endSensor;
    private Debouncer debouncer;

    private boolean rumbled = false;
    private boolean isIntaking = false;
    private boolean isCoral = false; // TODO: integrate with arm
    private boolean isHoldingCoral = true;
    private boolean isHoldingAlgae = false;
    private boolean holdSpeed = false;

    private boolean isPosHolding = false;
    private double holdPos = 0;

    private double lastRot = 0;

    private double speedAdjust = 0.0;

    private static enum EEMode {
        INTAKEMODE,
        OUTTAKEMODE
    }

    public EndEffector() {
        endEffector = new PearadoxTalonFX(EndEffectorConstants.END_EFFECTOR_ID, NeutralModeValue.Brake, 60, false);
        endSensor = new DigitalInput(EndEffectorConstants.END_SENSOR_CHANNEL);
        debouncer = new Debouncer(0.125, DebounceType.kRising);

        BaseStatusSignal.setUpdateFrequencyForAll(
                ArmConstants.UPDATE_FREQ,
                endEffector.getPosition(),
                endEffector.getVelocity(),
                endEffector.getDutyCycle(),
                endEffector.getMotorVoltage(),
                endEffector.getTorqueCurrent(),
                endEffector.getSupplyCurrent(),
                endEffector.getStatorCurrent());
        SmarterDashboard.putNumber("EE/EE Speed", isCoral ? -0.15 : 0.1);

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.2;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        endEffector.getConfigurator().apply(slot0Configs);
    }

    @Override
    public void periodic() {
        collectGamePiece();

        // isCoral = true;

        SmarterDashboard.putBoolean("EE/Holding Coral", isHoldingCoral);
        SmarterDashboard.putBoolean("EE/Holding Algae", isHoldingAlgae);
        SmarterDashboard.putBoolean("EE/Has Coral", hasCoral());
        SmarterDashboard.putBoolean("EE/Holding Speed", holdSpeed);

        SmarterDashboard.putNumber(
                "EE/Stator Current", endEffector.getStatorCurrent().getValueAsDouble());
        SmarterDashboard.putNumber(
                "EE/Supply Current", endEffector.getSupplyCurrent().getValueAsDouble());
        SmarterDashboard.putNumber("EE/Volts", endEffector.getMotorVoltage().getValueAsDouble());
        SmarterDashboard.putNumber("EE/lastrot", lastRot);
        SmarterDashboard.putNumber(
                "EE/Angular Velocity", endEffector.getVelocity().getValueAsDouble());
        SmarterDashboard.putNumber("EE/pos", endEffector.getPosition().getValueAsDouble());
    }

    // public void collectCoral() {
    //     if (DriverStation.isTeleop()) {
    //         if (RobotContainer.opController.getRightTriggerAxis() >= 0.25) {
    //             coralIn();
    //             isHolding = false;
    //         } else if (RobotContainer.opController.getLeftTriggerAxis() >= 0.25) {
    //             coralOut();
    //             isHolding = false;
    //         } else if (hasCoral() && isCoral) {
    //             stop();
    //         } else if (!isHolding) {
    //             holdCoral();
    //         }
    //     }
    // }
    public void collectGamePiece() {
        if (DriverStation.isTeleop()) {
            //     if(isCoral) {
            if (RobotContainer.driverController.getLeftBumperButton()) {
                if (isCoral) {
                    coralIn();
                    holdSpeed = false;
                } else if (!isCoral) {
                    algaeIn();
                    holdSpeed = true;
                }
                holdPos = endEffector.getPosition().getValueAsDouble();
                isHoldingCoral = false;
            } else if (RobotContainer.driverController.getRightBumperButton()) {
                if (isCoral) {
                    coralOut();
                } else if (!isCoral) {
                    algaeOut();
                }

                holdSpeed = false;
                isHoldingCoral = false;
            } else if (RobotContainer.driverController.getRightTriggerAxis() > 0.5) {
                fastOut();
            } else {
                if (!holdSpeed) {
                    stop();
                }
                if (isCoral) {
                    holdCoral();
                }
                if (!isPosHolding && hasCoral()) {
                    holdPos = endEffector.getPosition().getValueAsDouble() - 2;
                    isPosHolding = true;
                }
                if (isPosHolding) {
                    passiveCoral();
                    if (!hasCoral()) {
                        isPosHolding = false;
                    }
                }
            }

            // if (endEffector.getStatorCurrent().getValueAsDouble() > 35.0) {
            //     leds.setLEDsFlash();
            //     leds.handleFlashing();
            // } else leds.stopFlashing();
        }
    }

    // else if(!isCoral) {
    //     if(RobotContainer.opController.getRightTriggerAxis() >= 0.25) {
    //         algaeIn();
    //         isHoldingAlgae = true;
    //     }
    //     else if(RobotContainer.opController.getLeftTriggerAxis() >= 0.25) {
    //         algaeOut();
    //         isHoldingAlgae = false;
    //     }
    //     else if(isHoldingAlgae) {
    //         stopAlgae();
    //     }
    // }
    // }

    // scores

    public void eeSpeedAdjust(double adjustBy) {
        speedAdjust += adjustBy;
    }

    public void coralIn() {
        endEffector.set(EndEffectorConstants.PULL_SPEED);
        setLastRot();
    }

    public void algaeIn() {
        endEffector.set(EndEffectorConstants.ALGAE_PULL_SPEED);
    }

    public void algaeOut() {
        endEffector.set(EndEffectorConstants.ALGAE_PUSH_SPEED);
    }

    public void fastOut() {
        endEffector.set(1);
    }

    // intakes
    public void coralOut() {
        endEffector.set(EndEffectorConstants.PUSH_SPEED);
        setLastRot();
    }

    public void holdCoral() {
        // endEffector.set(SmartDashboard.getNumber("EE/EE Speed", isCoral ? -0.15 : 0.1));
        endEffector.set(EndEffectorConstants.HOLD_SPEED);
        setLastRot();
    }

    public void stopCoral() {
        stop();
        isHoldingCoral = true;
    }

    public void stopAlgae() {
        stop();
        isHoldingAlgae = true;
    }

    public void stop() {
        endEffector.set(0);
    }

    public void passiveCoral() {
        double curPos = endEffector.getPosition().getValueAsDouble();
        if (Math.abs(lastRot - curPos) > 20) lastRot = curPos - 2;

        System.out.println("attempting to hold " + lastRot + " rots but now at " + curPos);

        endEffector.setControl(new PositionVoltage(lastRot));

        // if this no work, replace everything above with
        // endEffector.set(-0.15);
    }

    public boolean hasCoral() {
        return debouncer.calculate(endEffector.getStatorCurrent().getValueAsDouble() > 35); // 50
    }

    public boolean getHolding() {
        return isHoldingCoral;
    }

    public void setHolding(boolean hold) {
        isHoldingCoral = hold;
    }

    public void setLastRot() {
        lastRot = endEffector.getPosition().getValueAsDouble();
    }

    public void setCoral() {
        isCoral = true;
    }

    public void setAlgae() {
        isCoral = false;
    }

    public boolean isCoral() {
        return isCoral;
    }
}
