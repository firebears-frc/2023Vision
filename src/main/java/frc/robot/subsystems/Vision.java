// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.text.Format;

import org.ejml.dense.block.VectorOps_FDRB;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private Double Round(Double x){
    return Math.round(x * 100.0) / 100.0;
  }
  private class Vector2D {
    private int x;
    private int y;

    public int getX() { return x; }
    public int getY() { return y; }

    public Vector2D(int nx,int ny){
      x = nx;
      y = ny;
    }

    public void setVector(int nx,int ny){
      x = nx;
      y = ny;
    }
  }
  private class VisionTarget {
    private int FID = -1;
    private Transform3d TargetTrans;
    private Vector2D TargetGlobalPos;
    private float LastSeen;
  
    public VisionTarget(int ID, int Xcm, int Ycm) {
      FID = ID;
      TargetGlobalPos = new Vector2D(Xcm, Ycm);
    }

    public void HasSeenTarget(Transform3d TargetTransform,float LastTimeSaw){
      TargetTrans = TargetTransform;
      LastSeen = LastTimeSaw;

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
    private int LastSeen;
    private Vector2D OurPosition;
    private Field2d F2d;

    public VisionMap(){
      Targets[0] = new VisionTarget(0, -689, 109);
      Targets[3] = new VisionTarget(3, -689, 280);
      Targets[5] = new VisionTarget(5, -689, 451);
      OurPosition = new Vector2D(0, 0);
      F2d = new Field2d();
    }

    public void onVisionTargetSeen(PhotonTrackedTarget Target){
      int FID = Target.getFiducialId();
      if(FID < Targets.length){
        Targets[FID].HasSeenTarget(Target.getBestCameraToTarget(),System.currentTimeMillis());
        LastSeen = FID;
      }

      //Get / Update Our Position Based On Targets Seen
      Transform3d pos = Targets[LastSeen].TargetTrans;
      double x;
      double y;

      if(Targets[LastSeen].TargetGlobalPos.x < 0){
        x = Targets[LastSeen].TargetGlobalPos.x + (pos.getX()*100);
        y = Targets[LastSeen].TargetGlobalPos.y + (pos.getY()*100);
      }
      else{
        x = Targets[LastSeen].TargetGlobalPos.x - (pos.getX()*100);
        y = Targets[LastSeen].TargetGlobalPos.y - (pos.getY()*100);
      }

      OurPosition.setVector((int)x, (int)y);
      SmartDashboard.putString("Our Robot Position", "(" + Round(x) + "cm ," + Round(y) + "cm )");
      F2d.setRobotPose(new Pose2d((x/100) + 8,y/100,new Rotation2d()));
      SmartDashboard.putData(F2d);

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
      VM.onVisionTargetSeen(result.getBestTarget());
    }
  }
}
