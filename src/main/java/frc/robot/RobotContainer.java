// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.logging.Mainlog;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;

public class RobotContainer {
    public Mainlog logging = new Mainlog();

    public CommandXboxController dctl = new CommandXboxController(0);

    public Flywheel shooter = new Flywheel();
    // public Hood hood = new Hood();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        dctl.a().whileTrue(shooter.spinUp).whileFalse(shooter.spinDown);
        dctl.b().whileTrue(shooter.kickUp).whileFalse(shooter.kickDown);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
