package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LowerArm extends SubsystemBase {
    private final DcMotor thrustMaster;

    public LowerArm(HardwareMap ahwMap) {
        thrustMaster = ahwMap.get(DcMotor.class, "thrustMaster");
    }


    public void stopMotor() {
        thrustMaster.setPower(0);
    }

    public void setLowerArmPower(double power) {

        thrustMaster.setPower(power);

        }
    }
