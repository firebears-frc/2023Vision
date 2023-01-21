// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int CHASSIS_FRONT_LEFT_MOTOR_CAN_ID = 2;
  public static final int CHASSIS_REAR_LEFT_MOTOR_CAN_ID = 3;
  public static final int CHASSIS_FRONT_RIGHT_MOTOR_CAN_ID = 4;
  public static final int CHASSIS_REAR_RIGHT_MOTOR_CAN_ID = 5;

  public static final int CHASSIS_STALL_CURRENT_LIMIT = 65;
  public static final int CHASSIS_FREE_CURRENT_LIMIT = 65;
  public static final boolean DEBUG = false;
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
