"""
Dance module - executes predefined dance routines
"""
import time
from science_robot import config


class DanceController:
    """Controls dance routine execution"""
    
    def __init__(self, motor_controller):
        """
        Initialize dance controller
        
        Args:
            motor_controller: MotorController instance
        """
        self.motor_controller = motor_controller
        self.dance_speed = config.MOTOR_DANCE_SPEED
        self.move_duration = config.DANCE_MOVE_DURATION
        self.is_dancing = False
    
    def execute_dance(self):
        """
        Execute the complete dance routine
        
        Returns:
            True if dance completed successfully
        """
        if self.is_dancing:
            return False
        
        self.is_dancing = True
        print("Starting dance routine...")
        
        try:
            # Dance sequence
            self._spin_right()
            self._move_forward_back()
            self._spin_left()
            self._figure_eight()
            self._final_spin()
            
            # Return to stopped position
            self.motor_controller.stop()
            print("Dance routine completed!")
            return True
            
        except Exception as e:
            print(f"Error during dance: {e}")
            self.motor_controller.stop()
            return False
        finally:
            self.is_dancing = False
    
    def _spin_right(self):
        """Spin clockwise"""
        print("  - Spinning right...")
        self.motor_controller.turn_right(self.dance_speed)
        time.sleep(self.move_duration)
        self.motor_controller.stop()
        time.sleep(0.2)
    
    def _spin_left(self):
        """Spin counter-clockwise"""
        print("  - Spinning left...")
        self.motor_controller.turn_left(self.dance_speed)
        time.sleep(self.move_duration)
        self.motor_controller.stop()
        time.sleep(0.2)
    
    def _move_forward_back(self):
        """Move forward then backward"""
        print("  - Moving forward...")
        self.motor_controller.move_forward(self.dance_speed)
        time.sleep(self.move_duration)
        self.motor_controller.stop()
        time.sleep(0.1)
        
        print("  - Moving backward...")
        self.motor_controller.move_backward(self.dance_speed)
        time.sleep(self.move_duration)
        self.motor_controller.stop()
        time.sleep(0.2)
    
    def _figure_eight(self):
        """Perform a figure-8 pattern"""
        print("  - Figure-8 pattern...")
        # Simplified figure-8: turn right, forward, turn left, forward
        self.motor_controller.turn_right(self.dance_speed * 0.7)
        time.sleep(self.move_duration * 0.5)
        self.motor_controller.move_forward(self.dance_speed)
        time.sleep(self.move_duration * 0.5)
        self.motor_controller.turn_left(self.dance_speed * 0.7)
        time.sleep(self.move_duration * 0.5)
        self.motor_controller.move_forward(self.dance_speed)
        time.sleep(self.move_duration * 0.5)
        self.motor_controller.stop()
        time.sleep(0.2)
    
    def _final_spin(self):
        """Final spin to finish dance"""
        print("  - Final spin...")
        self.motor_controller.turn_right(self.dance_speed)
        time.sleep(self.move_duration * 0.5)
        self.motor_controller.turn_left(self.dance_speed)
        time.sleep(self.move_duration * 0.5)
        self.motor_controller.stop()
        time.sleep(0.2)
    
    def is_dance_in_progress(self):
        """Check if dance is currently in progress"""
        return self.is_dancing

