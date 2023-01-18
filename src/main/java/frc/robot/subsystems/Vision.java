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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private class VisionTarget {
    private int FID = -1;
    private Transform3d TargetTrans;
  
    public VisionTarget(int ID) {
      FID = ID;
    }

    private Double Round(Double x){
      return Math.round(x * 100.0) / 100.0;
    }

    public void HasSeenTarget(Transform3d TargetTransform){
      TargetTrans = TargetTransform;

      if(TargetTrans != null) {
        SmartDashboard.putString(String.format("%d Target", FID), Round((Double)TargetTrans.getX()) + "," + Round((Double)TargetTrans.getY()) + ',' + Round((Double)TargetTrans.getZ()));
      }
      else{
        SmartDashboard.putString(String.format("%d Target", FID), "Target Not Here Cuh");
      }
    }
  }

  private class VisionMap {
    private VisionTarget[] Targets = new VisionTarget[8];

    public VisionMap(){
      Targets[0] = new VisionTarget(0);
      Targets[1] = new VisionTarget(1);
      Targets[2] = new VisionTarget(2);
      Targets[3] = new VisionTarget(3);
      Targets[4] = new VisionTarget(4);
      Targets[5] = new VisionTarget(5);
      Targets[6] = new VisionTarget(6);
      Targets[7] = new VisionTarget(7);
    }

    public void onVisionTargetSeen(PhotonTrackedTarget Target){
      int FID = Target.getFiducialId();
      if(FID < Targets.length){
        Targets[FID].HasSeenTarget(Target.getBestCameraToTarget());
      }
    }
  }

  PhotonCamera Camera;
  VisionMap VM;

  /** Creates a new Vision. */
  public Vision(String CamName) {
    Camera = new PhotonCamera(CamName);
    VM = new VisionMap();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("PhotonVision Works", Camera.isConnected());
    
    PhotonPipelineResult result = Camera.getLatestResult();
    if(result.hasTargets()){
      SmartDashboard.putBoolean("PhotonVision Active", true);
      for(PhotonTrackedTarget targ : result.getTargets()){
        VM.onVisionTargetSeen(targ);
      }
    }
    else SmartDashboard.putBoolean("PhotonVision Active", false);
  }
}
