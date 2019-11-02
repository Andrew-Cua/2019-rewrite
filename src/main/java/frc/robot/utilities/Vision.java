/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;
//haha secret comment
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * class to control vision stream from the limelight 2 
 */
public class Vision 
{   
    private NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
    //Horizontal offset from the crosshair to target(-27 to 27 deg)
    private NetworkTableEntry m_tx = m_limelight.getEntry("tx");
    //Vertical offset from the crosshair to target(-20.5 to 20.5 deg)
    private NetworkTableEntry m_ty = m_limelight.getEntry("ty");
    //Whether the limelight has a valid target (0 or 1)
    private NetworkTableEntry m_tv = m_limelight.getEntry("tv");
    //target area(% of image)
    private NetworkTableEntry m_ta = m_limelight.getEntry("ta");
    private NetworkTableEntry m_tp = m_limelight.getEntry("pipeline");

    private double m_x, m_y, m_area,m_v, m_currentPipeline;
    public static enum PipelineMode
    {
        kNormal(1), kGoal(2), kBall(3);
        private int pipeline;
        PipelineMode(int pipeline)
        {
            this.pipeline = pipeline;
        }
        int getPipeline()
        {
            return pipeline;
        }
    }

    public Vision()
    {

    }

// button randomizer
    /**
     * function called in periodicRobot that updates vision values
     */
    public void updateVision()
    {
        m_x = m_tx.getDouble(0.0);
        m_y = m_ty.getDouble(0.0);
        m_area = m_ta.getDouble(0.0);
        m_v = m_tv.getDouble(0.0);
        m_currentPipeline = m_tp.getDouble(0.0);
    }

    /**
     * sets the camera mode of the limelight - never use
     * @param mode - number of the camera mode
     */
    public void setCamMode(int mode)
    {
        m_limelight.getEntry("camMode").setNumber(mode);
    }

    public void setTrackTarget(PipelineMode target)
    {
        switch (target) 
        {
            case kNormal:
                if(m_currentPipeline != 1){setPipeline(target.getPipeline());}
                m_currentPipeline = 1;
                SmartDashboard.putNumber("Pipeline", target.getPipeline());
                break;
            case kBall:
                if(m_currentPipeline != 3){setPipeline(target.getPipeline());}
                m_currentPipeline = 3;
                SmartDashboard.putNumber("Pipeline", target.getPipeline());
                break;
                //secret comment
            case kGoal: 
                if(m_currentPipeline !=2){setPipeline(target.getPipeline());}
                m_currentPipeline = 2;
                SmartDashboard.putNumber("Pipeline", target.getPipeline());
                break;
        }
    }

    /**
     * function that changes the pipeline to the desired pipeline
     * @param pipeline - the number representation of what pipeline its changing to
     */
    private void setPipeline(double pipeline)
    {
        m_limelight.getEntry("pipeline").setNumber(pipeline);
    }


    //accessor functions 
    public double getV()
    {return m_v;}

    public double getX()
    {return m_x;}

    public double getY()
    {return m_y;}

    public double getA()
    {return m_area;}

    

}
