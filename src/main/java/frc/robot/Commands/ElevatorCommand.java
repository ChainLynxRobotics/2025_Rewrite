package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static frc.robot.RobotConfig.ElevatorConfig.kBottomHeight;
import static frc.robot.RobotConfig.ElevatorConfig.kMaxHeight;
import static frc.robot.RobotConfig.ElevatorConfig.kMiddleHeight;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command{

    private ElevatorSubsystem elevator;
    
    private XboxController controller = new XboxController(0);
    
    public enum ElevatorState{
        BOTTOM(kBottomHeight),MID(kMiddleHeight),HIGH(kMaxHeight);

        private double height;

        ElevatorState (double height){
            this.height = height;
        }

        public double getHeight(){
            return height;
        } 
    }

    public ElevatorCommand(ElevatorSubsystem elevator){
        this.elevator = elevator;
        this.addRequirements(elevator);
    }

    @Override
    public void execute() {
        if(controller.getAButtonPressed()){
            elevator.setHeight(kBottomHeight);
        }
        else if(controller.getBButtonPressed()){
            elevator.setHeight(kMiddleHeight);
        }
        else if(controller.getYButtonPressed()){
            elevator.setHeight(kMaxHeight);
        }
    }
    
    public Command setElevatorHeight(ElevatorState height){
        return Commands.run(()->{
                elevator.setHeight(height.getHeight());
            });
    }
}
