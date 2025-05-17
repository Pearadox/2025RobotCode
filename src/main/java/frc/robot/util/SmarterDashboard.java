// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class SmarterDashboard {

    public static void putString(String key, String value) {
        SmartDashboard.putString(key, value);
        Logger.recordOutput(key, value);
    }

    public static void putString(String key, String value, String subsystem) {
        SmartDashboard.putString(key, value);
        Logger.recordOutput(subsystem + "/" + key, value);
    }

    public static void putBoolean(String key, boolean value) {
        SmartDashboard.putBoolean(key, value);
        Logger.recordOutput(key, value);
    }

    public static void putBoolean(String key, boolean value, String subsystem) {
        SmartDashboard.putBoolean(key, value);
        Logger.recordOutput(subsystem + "/" + key, value);
    }

    public static void putNumber(String key, double value) {
        SmartDashboard.putNumber(key, value);
        Logger.recordOutput(key, value);
    }

    public static void putNumber(String key, double value, String subsystem) {
        SmartDashboard.putNumber(key, value);
        Logger.recordOutput(subsystem + "/" + key, value);
    }

    public static void putBooleanArray(String key, boolean[] value, String subsystem) {
        SmartDashboard.putBooleanArray(key, value);
        Logger.recordOutput(subsystem + "/" + key, value);
    }

    public static void putNumberArray(String key, double[] value, String subsystem) {
        SmartDashboard.putNumberArray(key, value);
        Logger.recordOutput(subsystem + "/" + key, value);
    }

    public static void putStringArray(String key, String[] value, String subsystem) {
        SmartDashboard.putStringArray(key, value);
        Logger.recordOutput(subsystem + "/" + key, value);
    }

    public static <T extends WPISerializable> void putData(String key, T value) {
        SmartDashboard.putString(key, value.toString());
        Logger.recordOutput(key, value);
    }

    public static <T extends WPISerializable> void putData(String key, T value, String subsystem) {
        SmartDashboard.putString(key, value.toString());
        Logger.recordOutput(subsystem + "/" + key, value);
    }
}
