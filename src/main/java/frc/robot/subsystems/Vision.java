// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.text.Format;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private class VisionTarget {
    private int FID = -1;
    private Transform3d TargetTrans;
  
    public VisionTarget(int ID) {
      FID = ID;
    }

    public boolean HasSeenTarget(int TargetID,Transform3d TargetTransform){
      SmartDashboard.putString(String.format("%d Active", FID), "No");
      if(TargetID == FID){
        TargetTrans = TargetTransform;
        SmartDashboard.putString(String.format("%d Active", FID), "Yes");
        return true;
      }
      return false;
    }

    public void periodic(){
      if(TargetTrans != null) {
        SmartDashboard.putString(String.format("%d Target", FID), TargetTrans.getX() + "," + TargetTrans.getY() + ',' + TargetTrans.getZ());
      }
      else{
        SmartDashboard.putString(String.format("%d Target", FID), "Target Not Here Cuh");
      }
    }
  }

  PhotonCamera Camera;
  VisionTarget[] Targets = new VisionTarget[8];

  /** Creates a new Vision. */
  public Vision(String CamName) {
    Camera = new PhotonCamera(CamName);
    for (int i=0; i<7; ++i) {
      Targets[i] = new VisionTarget(i);
    }
  }

  private void WhereRWe(){

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("PhotonVision Works", Camera.isConnected());
    
    PhotonPipelineResult result = Camera.getLatestResult();
    if(result.hasTargets()){
      SmartDashboard.putBoolean("PhotonVision Active", true);
      for(PhotonTrackedTarget target : result.getTargets()){
        if(target.getFiducialId() < Targets.length && Targets[target.getFiducialId()] != null){
          Targets[target.getFiducialId()].HasSeenTarget(target.getFiducialId(),target.getBestCameraToTarget());
        }
      }
    }
    else SmartDashboard.putBoolean("PhotonVision Active", false);

    for(VisionTarget vt : Targets){
      if(vt != null) vt.periodic();
    }

    WhereRWe();
  }
}
