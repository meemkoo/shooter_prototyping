// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private Shooter shooter = new Shooter();

  public CommandXboxController dctl = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // dctl.a().whileTrue(shooter.setVelocity(RPM.of(2900)));
    dctl.a().whileTrue(shooter.setVelocity(() -> {
        System.out.println("axis detected%f".formatted(dctl.getRawAxis(0)));
        return RPM.of(3000*dctl.getRawAxis(0));
    }));
    dctl.a().whileFalse(shooter.setVelocity(() -> RPM.of(0)));
    dctl.b().whileTrue(shooter.setSpeed(0.5));
    dctl.b().whileFalse(shooter.setSpeed(0));
    // shooter.setDefaultCommand(shooter.setVelocity(RPM.of(0)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
