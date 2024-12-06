package org.firstinspires.ftc.teamcode.Opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.Subsystems.HighArm;
import org.firstinspires.ftc.teamcode.Subsystems.LowerArm;
import org.firstinspires.ftc.teamcode.Subsystems.InOuttake;


@TeleOp
public class Skibiuda extends LinearOpMode {

    private GamepadEx driverOP;
    private GamepadEx buttonOP;
    private Drivebase drivebase;

    private Arm arm;

    private HighArm highArm;

    private LowerArm lowerArm;

    private InOuttake inOuttake;

    @Override
    public void runOpMode() throws InterruptedException {
        driverOP = new GamepadEx(gamepad1);
        buttonOP = new GamepadEx(gamepad2);

        arm = new Arm(hardwareMap);
        highArm = new HighArm(hardwareMap);
        lowerArm = new LowerArm(hardwareMap);
        drivebase = new Drivebase(hardwareMap);
        inOuttake = new InOuttake(hardwareMap);

        //GamepadButton bothArmUp = new GamepadButton(buttonOP, GamepadKeys.Button.X);
        //GamepadButton bothArmDown = new GamepadButton(buttonOP, GamepadKeys.Button.Y);
        GamepadButton lowerArmUp = new GamepadButton(buttonOP, GamepadKeys.Button.B);
        GamepadButton lowerArmDown = new GamepadButton(buttonOP,GamepadKeys.Button.X);
        GamepadButton higherArmUp = new GamepadButton(buttonOP, GamepadKeys.Button.DPAD_UP);
        GamepadButton higherArmDown = new GamepadButton(buttonOP, GamepadKeys.Button.DPAD_DOWN);
        GamepadButton rollerIn = new GamepadButton(buttonOP, GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton rollerOut = new GamepadButton(buttonOP, GamepadKeys.Button.RIGHT_BUMPER);

        //bothArmUp.whenPressed(new InstantCommand(() -> arm.bothArmUp(), arm));
        lowerArmUp.whenPressed(new InstantCommand(() -> lowerArm.setLowerArmPower(-1), lowerArm)).whenReleased(new InstantCommand(() -> lowerArm.setLowerArmPower(0), lowerArm));
        lowerArmDown.whenPressed(new InstantCommand(() -> lowerArm.setLowerArmPower(1), lowerArm)).whenReleased(new InstantCommand(() -> lowerArm.setLowerArmPower(0), lowerArm));
        higherArmUp.whenPressed(new InstantCommand(() -> highArm.setPower(-1), highArm)).whenReleased(new InstantCommand(() -> highArm.setPower(0), highArm));
        higherArmDown.whenPressed(new InstantCommand(() -> highArm.setPower(1), highArm)).whenReleased(new InstantCommand(() -> highArm.setPower(0), highArm));

        rollerIn.whenPressed(new InstantCommand(() -> inOuttake.increaseGrip(), inOuttake));
        rollerOut.whenPressed(new InstantCommand(() -> inOuttake.decreaseGrip(), inOuttake));




        drivebase.setDefaultCommand(new RunCommand(() ->
                drivebase.driveRobotCentric(driverOP.getLeftX(), driverOP.getLeftY() * -1, driverOP.getRightX()),
                drivebase
        ));

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            CommandScheduler.getInstance().run();

            telemetry.addData("Encoder Position", "Eddy is the best");
            telemetry.update();
        }

        CommandScheduler.getInstance().reset();
    }

    /*
    public class EncoderOpmode extends Skibiuda {
        @Override
        public void runOpMode() throws InterruptedException {
            // Find a motor in the hardware map named "Arm Motor"
            DcMotor motor = hardwareMap.dcMotor.get("thrustMaster");

            // Reset the motor encoder so that it reads zero ticks
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            waitForStart();

            while (opModeIsActive()) {
                //30 is a random number so needs to be changed
                double CPR = 30;

                // Get the current position of the motor
                int position = motor.getCurrentPosition();
                double revolutions = position/CPR;

                double angle = revolutions * 360;
                double angleNormalized = angle % 360;

                // Show the position of the motor on telemetry
                telemetry.addData("Encoder Position", position);
                telemetry.addData("Encoder Revolutions", revolutions);
                telemetry.addData("Encoder Angle (Degrees)", angle);
                telemetry.addData("Encoder Angle - Normalized (Degrees)", angleNormalized);
                telemetry.update();
            }
        }
    }*/
}


