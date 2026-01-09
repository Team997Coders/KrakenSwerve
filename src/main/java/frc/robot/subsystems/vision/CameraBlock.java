package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class CameraBlock 
{
    ArrayList<Camera> cameraList;

    public CameraBlock(ArrayList<Camera> cameraList)
    {
        this.cameraList = cameraList;
    }

    public void update(SwerveDrivePoseEstimator poseEstimator)
    {
        for (Camera camera: this.cameraList)
        {
            camera.update(poseEstimator, camera.getResults());
        }
    }
}