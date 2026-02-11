// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Turret;

public class RobotContainer {
  private final Turret turret = new Turret();
  private final CommandXboxController joystick = new CommandXboxController(0);
  
  public RobotContainer() {
    SmartDashboard.putNumber("Shooter Voltage", 0);
    SmartDashboard.putNumber("Shooter Hood Angle", 0.5);
    configureBindings();
  }

  private void configureBindings() {
    joystick.a().whileTrue(turret.runHoodAndShooterCommand(
        () -> -SmartDashboard.getNumber("Shooter Voltage", 0),
        () -> joystick.getLeftY()
    ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
