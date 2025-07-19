// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Feeder;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class i_LaserCanSubsystem extends SubsystemBase {
  /** Creates a new i_LaserCanSubsystem. */
  private LaserCan i_LaserCan;

  public i_LaserCanSubsystem() {
    i_LaserCan = new LaserCan(21);
    try {
      i_LaserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      i_LaserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      i_LaserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (au.grapplerobotics.ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  public boolean isTargetClose() {
    LaserCan.Measurement measurement = i_LaserCan.getMeasurement();
    return measurement != null && measurement.distance_mm > 200;
  }

  public LaserCan.Measurement getLaserMeasurement() {
    return i_LaserCan.getMeasurement();
  }

  public double getDistance() {
    LaserCan.Measurement measurement = i_LaserCan.getMeasurement();
    return measurement != null ? measurement.distance_mm : -1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
