// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimbConstants;
import org.littletonrobotics.junction.Logger;

/** Class to run the rollers over CAN */
public class Climber extends SubsystemBase {
    // private final SparkMax climbMotor;
    private final PearadoxTalonFX climbMotor;

    public Climber() {

        climbMotor = new PearadoxTalonFX(ClimbConstants.CLIMB_MOTOR_ID, NeutralModeValue.Brake, 50, false);

        BaseStatusSignal.setUpdateFrequencyForAll(
                ArmConstants.UPDATE_FREQ,
                climbMotor.getPosition(),
                climbMotor.getMotorVoltage(),
                climbMotor.getTorqueCurrent(),
                climbMotor.getSupplyCurrent(),
                climbMotor.getStatorCurrent());

        climbMotor.optimizeBusUtilization();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Climber/Pos", climbMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Climber/Volts", climbMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput(
                "Climber/Supply Current", climbMotor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput(
                "Climber/Supply Current", climbMotor.getStatorCurrent().getValueAsDouble());
    }

    /** This is a method that makes the roller spin */
    public void runClimb(double forward, double reverse) {
        Logger.recordOutput("Climber/Percent Speed", forward - reverse);
        climbMotor.set(forward - reverse);
    }
}
