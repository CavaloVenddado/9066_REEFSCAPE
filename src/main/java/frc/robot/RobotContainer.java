// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

import frc.robot.subsystems.DriveSubsystem;

// import frc.robot.commands.CommandName;

public class RobotContainer {
  private final DriveSubsystem drive = new DriveSubsystem();

  private final SendableChooser<Command> autoChooser;

  CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandJoystick SecondController = new CommandJoystick(OIConstants.kSecondControllerPort);

  RobotConfig config;{
  try{
    config = RobotConfig.fromGUISettings();
  } catch (Exception e) {
    // Handle exception as needed
    e.printStackTrace();}
  }

  public RobotContainer() {
    AutoBuilder.configure(
        drive::getPose,
        drive::resetOdometry,
        drive::getSpeeds,
        (speeds, feedfowards) -> {
          drive.drive(
              speeds.vxMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
              speeds.vyMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
              speeds.omegaRadiansPerSecond / DriveConstants.kMaxAngularSpeed,
              false,
              false);
        },
        new PPHolonomicDriveController(
          new PIDConstants(5.0),
          new PIDConstants(5),
          0.417193
        ),
        config,
        () -> {
          
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        drive);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);

    drive.setDefaultCommand(
        new RunCommand(
            () -> drive.drive(
                -MathUtil.applyDeadband(driverController.getRawAxis(1), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getRawAxis(0), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getRawAxis(4), OIConstants.kDriveDeadband),
                true, true),
            drive));

    configureButtonBindings();

    // NamedCommands.registerCommand("CommandName", new CommandName(necessary subsystens input));
    
    // NamedCommands.registerCommand("CommandName", new InstantCommand(() -> action desired));
  }

  private void configureButtonBindings() {
   
    // driverController.button().when(what it does);

    //---------------

    // secondController.button().when(what it does)

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    
    // return AutoBuilder.buildAuto("SL-N1N4");
  }
}
