// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class drive extends CommandBase {
  private  DriveTrain m_drivetrainSubsystem;
  private  DoubleSupplier m_translationXSupplier;
  private  DoubleSupplier m_translationYSupplier;
  private  DoubleSupplier m_rotationSupplier;
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
    public drive(DriveTrain drivetrainsubsytem,DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier)
    {
    m_translationXSupplier = translationXSupplier;
    m_translationYSupplier =  translationYSupplier;
    m_rotationSupplier = rotationSupplier;
    this.m_drivetrainSubsystem = drivetrainsubsytem;
    //this.m_translationXSupplier = translationXSupplier;
    //this.m_translationYSupplier = translationYSupplier;
    //this.m_rotationSupplier = rotationSupplier;

    addRequirements(drivetrainsubsytem);

    }
    

    @Override
    public void execute() {
      m_drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          modifyAxis(m_translationXSupplier.getAsDouble()), 
          modifyAxis(m_translationYSupplier.getAsDouble()), 
          modifyAxis(m_rotationSupplier.getAsDouble()), 
          m_drivetrainSubsystem.getGyroscopeRotation()
        )
      );
    }

    @Override
    public void end(boolean interrupted) {
      m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
  
}
