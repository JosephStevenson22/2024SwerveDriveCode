// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);

  public RobotContainer() {
      swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
              swerveSubsystem,
              () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
              () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
              () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
              () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

      configureButtonBindings();
  }

  private void configureButtonBindings() {
      new JoystickButton(driverJoytick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
  }
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
