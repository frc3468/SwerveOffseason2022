// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class drive extends CommandBase {
  private  DriveTrain m_drivetrainSubsystem;
  private  DoubleSupplier m_translationXSupplier;
  private  DoubleSupplier m_translationYSupplier;
  private  DoubleSupplier m_rotationSupplier;

    public drive(DriveTrain drivetrainsubsytem, XboxController mController)//aDoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier
    {
    double translationXSupplier = -modifyAxis(mController.getY(GenericHID.Hand.kLeft)) *drivetrainsubsytem.maxVelocityMetersPerSecond;
    double translationYSupplier = -modifyAxis(mController.getX(GenericHID.Hand.kLeft));
    double rotationSupplier = -modifyAxis(mController.getX(GenericHID.Hand.kRight));
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
          m_translationXSupplier.getAsDouble(), 
          m_translationYSupplier.getAsDouble(), 
          m_rotationSupplier.getAsDouble(), 
          m_drivetrainSubsystem.getGyroscopeRotation()
        )
      );
    }

    @Override
    public void end(boolean interrupted) {
      m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
  
}
