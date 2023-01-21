// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Chassis;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  private final Chassis m_chassis;
  private Joystick joystick;

  public DriveCommand(Chassis m, Joystick j) {
    m_chassis = m;
    joystick = j;
    addRequirements(m_chassis);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double speed = joystick.getY();
    double rotation = joystick.getZ();

    m_chassis.arcadeDrive(speed, rotation * MathUtil.clamp((-joystick.getRawAxis(3)+1.0)/2,0,1));
    SmartDashboard.putNumber("Axis 3", joystick.getRawAxis(3));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
