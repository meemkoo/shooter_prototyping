// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.logging.Mainlog;
import frc.robot.subsystems.RawShooter;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    public Mainlog logging = new Mainlog();

    public CommandXboxController dctl = new CommandXboxController(0);

    public RawShooter shooter = new RawShooter();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
