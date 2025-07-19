// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Feeder;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaserCanSubsystem extends SubsystemBase {
  private LaserCan f_LaserCan;
  /** Creates a new LaserCanSubsystem. */
  public LaserCanSubsystem() {
    f_LaserCan = new LaserCan(20);
    try {
      f_LaserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      f_LaserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      f_LaserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (au.grapplerobotics.ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  // public void InspectionLaser(){
  //  LaserCan.Measurement measurement = f_LaserCan.getMeasurement();
  //  if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
  //    System.out.println("The target is " + measurement.distance_mm + "mm away!");
  //  } else {
  //    System.out.println("Oh no! The target is out of range, or we can't get a reliable
  // measurement!");
  //    // You can still use distance_mm in here, if you're ok tolerating a clamped value or an
  // unreliable measurement.
  //  }
  // }

  public boolean isTargetClose() {
    LaserCan.Measurement measurement = f_LaserCan.getMeasurement();
    return measurement != null && measurement.distance_mm < 12;
  }

  public LaserCan.Measurement getLaserMeasurement() {
    return f_LaserCan.getMeasurement();
  }

  public double getDistance() {
    LaserCan.Measurement measurement = f_LaserCan.getMeasurement();
    return measurement != null ? measurement.distance_mm : -1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double distance = getDistance();
    boolean isTargetClose = isTargetClose();

    SmartDashboard.putNumber("Laser Distance (mm)", distance);
    SmartDashboard.putBoolean("Is Target Close", isTargetClose);
  }
}
