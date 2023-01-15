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
      if(TargetID == FID){
        TargetTrans = TargetTransform;
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
    Targets[0] = new VisionTarget(0);
    Targets[1] = new VisionTarget(1);
    Targets[2] = new VisionTarget(6);
    Targets[3] = new VisionTarget(4);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    PhotonPipelineResult result = Camera.getLatestResult();
    if(result.hasTargets()){
      for(PhotonTrackedTarget target : result.getTargets()){
        if(Targets.length < (target.getFiducialId()-1) && Targets[target.getFiducialId()] != null){
          Targets[target.getFiducialId()].HasSeenTarget(target.getFiducialId(),target.getBestCameraToTarget());
        }
      }
    }

    for(VisionTarget vt : Targets){
      if(vt != null) vt.periodic();
    }
  }
}
